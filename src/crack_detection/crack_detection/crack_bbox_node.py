"""Live U-Net crack segmentation with optional depth projection for RViz."""

from collections import deque
import copy
import json
import signal
import time
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from crack_detection.camera_calibration import transform_matrix, transform_points
from crack_detection.crack_onnx import CrackSegmenter
from crack_detection.crack_projection import crack_surface_points


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def decode_image(message):
    """Decode supported ROS images, including row padding and depth endianness."""
    formats = {
        "rgb8": (np.dtype("u1"), 3), "bgr8": (np.dtype("u1"), 3),
        "rgba8": (np.dtype("u1"), 4), "bgra8": (np.dtype("u1"), 4),
        "mono8": (np.dtype("u1"), 1),
        "16UC1": (np.dtype(">u2" if message.is_bigendian else "<u2"), 1),
        "mono16": (np.dtype(">u2" if message.is_bigendian else "<u2"), 1),
        "32FC1": (np.dtype(">f4" if message.is_bigendian else "<f4"), 1),
    }
    if message.encoding not in formats:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    dtype, channels = formats[message.encoding]
    if (message.height < 1 or message.width < 1
            or message.step < message.width * channels * dtype.itemsize
            or len(message.data) < message.height * message.step):
        raise ValueError("Invalid image dimensions, step, or data length")
    shape = (message.height, message.width, channels)
    view = np.ndarray(shape, dtype=dtype, buffer=message.data,
                      strides=(message.step, channels * dtype.itemsize, dtype.itemsize))
    return view[:, :, 0] if channels == 1 else view


def color_bgr(message):
    image = decode_image(message)
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR,
                   "bgra8": cv2.COLOR_BGRA2BGR, "mono8": cv2.COLOR_GRAY2BGR}
    if message.encoding == "bgr8":
        return image.copy()
    if message.encoding not in conversions:
        raise ValueError(f"Expected an RGB image, got {message.encoding}")
    return cv2.cvtColor(image, conversions[message.encoding])


def image_message(array, header, encoding):
    array = np.ascontiguousarray(array)
    return Image(header=header, height=array.shape[0], width=array.shape[1], encoding=encoding,
                 is_bigendian=0, step=array.strides[0], data=array.tobytes())


def detection_message(boxes, header):
    """Preserve the team's existing 2D JSON interface, including empty frames."""
    payload = {
        "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
        "frame_id": header.frame_id,
        "count": len(boxes),
        "boxes": boxes,
    }
    return String(data=json.dumps(payload))


def boxes_from_mask(mask, probability, min_area, min_length=1):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    # Sum each component once rather than scanning the entire image per box.
    sums = np.bincount(labels.ravel(), weights=probability.ravel(), minlength=count)
    boxes = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area < min_area or max(width, height) < min_length:
            continue
        boxes.append({
            "x": int(x), "y": int(y), "width": int(width), "height": int(height),
            "area": float(area), "score": round(float(sums[index] / area), 3),
        })
    boxes.sort(key=lambda box: box["area"], reverse=True)
    return boxes


def draw_crack_boxes(frame, boxes, max_boxes):
    """Draw boxes only; segmentation pixels are published separately."""
    debug = frame.copy()
    for box in boxes[:max_boxes]:
        x, y, width, height = (box[key] for key in ("x", "y", "width", "height"))
        cv2.rectangle(debug, (x, y), (x + width, y + height), (0, 0, 0), 5)
        cv2.rectangle(debug, (x, y), (x + width, y + height), (0, 255, 255), 2)
    return debug


def point_cloud_message(points, header):
    packed = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    packed["x"], packed["y"], packed["z"] = points[:, 0], points[:, 1], points[:, 2]
    packed["rgb"] = 0xFFFF00
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(("x", "y", "z"))]
    fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
    return PointCloud2(header=header, height=1, width=len(points), fields=fields,
                       is_bigendian=False, point_step=16, row_step=packed.nbytes,
                       is_dense=True, data=packed.tobytes())


class CrackBBoxNode(Node):
    def __init__(self):
        super().__init__("crack_bbox_node")
        models = Path(get_package_share_directory("crack_detection")) / "models"
        parameters = {
            "image_topic": "/camera/camera/color/image_raw",
            "debug_topic": "/crack_detection/debug_image",
            "detections_topic": "/crack_detection/detections",
            "color_info_topic": "/camera/camera/color/camera_info",
            "depth_topic": "/camera/camera/depth/image_rect_raw",
            "depth_info_topic": "/camera/camera/depth/camera_info",
            "model_path": str(models / "crack_unet16_v1.onnx"),
            "config_path": "", "num_threads": 2,
            "threshold": -1.0, "max_rate_hz": 5.0,
            "min_area": 150.0, "min_length": 1, "max_boxes": 16,
            "publish_cloud": True, "publish_markers": True,
            "target_frame": "", "sync_tolerance_s": 0.05,
            "zmin_m": 0.30, "zmax_m": 1.60, "depth_scale_m": 0.001,
            "cloud_stride": 2, "max_cloud_points": 5000, "marker_lifetime_s": 0.5,
            "color_robot_mask": str(models / "robot_mask_color.png"),
            "robot_mask": str(models / "robot_mask_depth.png"),
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in parameters}
        p = self.options
        finite_parameters = ("max_rate_hz", "zmin_m", "zmax_m", "sync_tolerance_s",
                             "depth_scale_m", "marker_lifetime_s", "min_area", "threshold")
        if (not np.isfinite([p[name] for name in finite_parameters]).all() or p["max_rate_hz"] <= 0
                or not 0 < p["zmin_m"] < p["zmax_m"] or p["sync_tolerance_s"] <= 0
                or p["depth_scale_m"] <= 0 or p["cloud_stride"] < 1
                or p["max_cloud_points"] < 1 or p["max_boxes"] < 0 or p["min_area"] < 1 or p["min_length"] < 1
                or p["marker_lifetime_s"] <= 0 or p["num_threads"] < 1
                or (p["threshold"] != -1 and not 0 < p["threshold"] < 1)):
            raise ValueError("Invalid crack node rate, depth range, synchronization, or display parameters")
        self.seg = CrackSegmenter(
            p["model_path"], meta_path=p["config_path"] or None, num_threads=p["num_threads"],
            threshold=None if p["threshold"] == -1 else p["threshold"],
        )
        if not 0 < self.seg.threshold < 1:
            raise ValueError("Model probability threshold must be between 0 and 1")
        self.color_robot_mask = cv2.imread(p["color_robot_mask"], cv2.IMREAD_GRAYSCALE)
        if self.color_robot_mask is None:
            raise ValueError(f"Colour robot mask not readable: {p['color_robot_mask']}")
        self.robot_mask = None
        if p["robot_mask"]:
            mask = cv2.imread(p["robot_mask"], cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise ValueError(f"Robot mask not readable: {p['robot_mask']}")
            self.robot_mask = mask > 0
        self.latest_image = None
        self.input_lock = Lock()
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.depth_frames = deque(maxlen=30)
        self.color_info = self.depth_info = None
        self.received = self.processed = 0
        self.inference_ms = 0.0
        self.project_3d = p["publish_cloud"] or p["publish_markers"]
        # A single pending image prevents inference from building a camera backlog.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.mask_pub = self.create_publisher(Image, "/crack_detection/mask", 1)
        self.debug_pub = self.create_publisher(Image, p["debug_topic"], 1)
        self.detections_pub = self.create_publisher(String, p["detections_topic"], 1)
        self.info_pub = self.create_publisher(CameraInfo, "/crack_detection/camera_info", 1)
        self.create_subscription(Image, p["image_topic"], self.on_image, sensor_qos, callback_group=self.input_group)
        self.create_subscription(CameraInfo, p["color_info_topic"], self.on_color_info, sensor_qos, callback_group=self.input_group)
        if self.project_3d:
            self.cloud_pub = self.create_publisher(PointCloud2, "/crack_detection/points", 1) if p["publish_cloud"] else None
            self.marker_pub = self.create_publisher(MarkerArray, "/crack_detection/markers", 1) if p["publish_markers"] else None
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Image, p["depth_topic"], self.on_depth, sensor_qos, callback_group=self.input_group)
            self.create_subscription(CameraInfo, p["depth_info_topic"], self.on_depth_info, sensor_qos, callback_group=self.input_group)
        self.create_timer(1.0 / p["max_rate_hz"], self.process_latest)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"U-Net threshold {self.seg.threshold:g}; RGB {p['image_topic']}; "
            f"up to {p['max_rate_hz']:g} Hz; 3D projection {self.project_3d}"
        )

    def on_image(self, message):
        with self.input_lock:
            self.latest_image = message
            self.received += 1

    def on_depth(self, message):
        with self.input_lock:
            if self.depth_frames and stamp_ns(message.header) < stamp_ns(self.depth_frames[-1].header):
                self.depth_frames.clear()  # A looping bag or clock reset.
            self.depth_frames.append(message)

    def on_color_info(self, message):
        with self.input_lock:
            self.color_info = message

    def on_depth_info(self, message):
        with self.input_lock:
            self.depth_info = message

    def lookup_matrix(self, target, source, stamp):
        transform = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration())
        t, q = transform.transform.translation, transform.transform.rotation
        return transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def process_latest(self):
        with self.input_lock:
            message = self.latest_image
            self.latest_image = None
            color_info = self.color_info
        if message is None:
            return
        try:
            bgr = color_bgr(message)
            if self.color_robot_mask.shape != bgr.shape[:2]:
                raise ValueError("Colour robot mask must match RGB dimensions; supply a mask for this camera")
            start = time.perf_counter()
            probability, mask = self.seg.predict(bgr, bgr=True, ground_mask=self.color_robot_mask)
            self.inference_ms = (time.perf_counter() - start) * 1000
            boxes = boxes_from_mask(mask, probability, self.options["min_area"], self.options["min_length"])
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Crack inference failed: {exc}", throttle_duration_sec=5.0)
            return
        self.mask_pub.publish(image_message(mask, message.header, "mono8"))
        debug = draw_crack_boxes(bgr, boxes, max_boxes=self.options["max_boxes"])
        self.debug_pub.publish(image_message(debug, message.header, "bgr8"))
        self.detections_pub.publish(detection_message(boxes, message.header))
        if color_info is not None:
            info = copy.deepcopy(color_info)
            info.header = message.header
            self.info_pub.publish(info)
        self.processed += 1
        if self.project_3d:
            try:
                points, header = self.project_points(mask, message.header)
            except (ValueError, TransformException) as exc:
                self.get_logger().warn(f"3D crack projection unavailable: {exc}", throttle_duration_sec=5.0)
                points, header = np.empty((0, 3), np.float32), message.header
            self.publish_points(points, header)

    def project_points(self, mask, rgb_header):
        with self.input_lock:
            depth_frames = tuple(self.depth_frames)
            ci, di = self.color_info, self.depth_info
        if not depth_frames or ci is None or di is None:
            raise ValueError("waiting for depth and both CameraInfo topics; 2D outputs remain available")
        depth = min(depth_frames, key=lambda item: abs(stamp_ns(item.header) - stamp_ns(rgb_header)))
        offset = abs(stamp_ns(depth.header) - stamp_ns(rgb_header)) * 1e-9
        if offset > self.options["sync_tolerance_s"]:
            raise ValueError(f"RGB/depth timestamps differ by {offset:.3f} s")
        if ((ci.height, ci.width) != mask.shape or (di.height, di.width) != (depth.height, depth.width)
                or ci.header.frame_id != rgb_header.frame_id or di.header.frame_id != depth.header.frame_id):
            raise ValueError("CameraInfo dimensions or frame IDs do not match the images")
        depth_m = decode_image(depth).astype(np.float32)
        if depth.encoding in ("16UC1", "mono16"):
            depth_m *= self.options["depth_scale_m"]
        elif depth.encoding != "32FC1":
            raise ValueError(f"Unsupported depth encoding: {depth.encoding}")
        robot_mask = self.robot_mask
        if robot_mask is not None and robot_mask.shape != depth_m.shape:
            raise ValueError("Robot mask dimensions differ from depth; supply the correct mask or disable it")
        extrinsics = self.lookup_matrix(rgb_header.frame_id, depth.header.frame_id, depth.header.stamp)
        points = crack_surface_points(
            mask, depth_m, (di.k[0], di.k[4], di.k[2], di.k[5]),
            (ci.k[0], ci.k[4], ci.k[2], ci.k[5]), extrinsics,
            valid_mask=robot_mask, zmin=self.options["zmin_m"], zmax=self.options["zmax_m"],
            stride=self.options["cloud_stride"], max_points=self.options["max_cloud_points"],
            distortion=ci.d, distortion_model=ci.distortion_model,
        )
        header = copy.deepcopy(depth.header)
        target = self.options["target_frame"]
        if target and target != header.frame_id:
            points = transform_points(points, self.lookup_matrix(target, header.frame_id, header.stamp)).astype(np.float32)
            header.frame_id = target
        return points, header

    def publish_points(self, points, header):
        if self.cloud_pub is not None:
            self.cloud_pub.publish(point_cloud_message(points, header))
        if self.marker_pub is not None:
            clear = Marker(header=header, ns="crack_detection", action=Marker.DELETEALL)
            marker = Marker(header=header, ns="crack_detection", id=0, type=Marker.POINTS, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.01
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            marker.lifetime = Duration(seconds=self.options["marker_lifetime_s"]).to_msg()
            marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
            self.marker_pub.publish(MarkerArray(markers=[clear, marker]))

    def report(self):
        if not self.received:
            self.get_logger().warn("Waiting for RGB images; check image_topic, playback, and use_sim_time")
        else:
            self.get_logger().info(
                f"RGB frames received {self.received}; processed {self.processed}; "
                f"last inference {self.inference_ms:.1f} ms"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = CrackBBoxNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch and the terminal may both send SIGINT. Let an active inference
        # finish and release ROS handles without a second interrupt mid-cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
