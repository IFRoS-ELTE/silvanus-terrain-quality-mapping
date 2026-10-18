#!/usr/bin/env python3
"""
ground_hazard_node: the depth hazard pipeline, as a live ROS 2 node.

This wraps the measurement code in ground_probe.py so it runs against the
robot, or against a bag replay, instead of a folder of frames. The maths is
unchanged - RANSAC ground plane, signed residual per point, connected
components over the points that stand off the plane.

Subscribes
    <depth_topic>          sensor_msgs/Image       16UC1, millimetres
    <info_topic>           sensor_msgs/CameraInfo  intrinsics

Publishes
    /ground/hazards          vision_msgs/Detection3DArray
    /ground/hazard_markers   visualization_msgs/MarkerArray
    /ground/unevenness       std_msgs/Float32          mm RMS on the plane
    /ground/residual_cloud   sensor_msgs/PointCloud2   optional, see below

Run it
    python3 ground_hazard_node.py
    python3 ground_hazard_node.py --ros-args -p publish_cloud:=true

Against a bag, in another terminal:
    ros2 bag play <bag> --loop
    python3 ground_hazard_node.py --ros-args -p use_sim_time:=true

Two things worth knowing before changing anything here:

The camera publishes with BEST_EFFORT reliability (SensorDataQoS). Subscribing
with the default RELIABLE profile silently receives nothing at all - no error,
no warning, just an empty topic. That is why the QoS below is explicit.

RANSAC on a 848x480 frame does not keep up with 30 Hz on a laptop. Rather than
let the callback queue grow without bound, process_every_n drops frames on
purpose. Measuring every third frame at 10 Hz is honest; pretending to run at
30 Hz while falling further behind is not.
"""

import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float32
from vision_msgs.msg import (BoundingBox3D, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from ground_probe import (deproject, find_hazards, fit_ground_plane,
                          plane_basis, plane_pose, refine_plane)


def rotation_to_quaternion(m):
    """3x3 rotation matrix -> (x, y, z, w). Shepperd's method, branchless cases."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s)


class GroundHazardNode(Node):

    def __init__(self):
        super().__init__("ground_hazard_node")

        self.declare_parameter("depth_topic",
                               "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("info_topic",
                               "/camera/camera/depth/camera_info")
        # 15 mm: about three times the 4.5 mm RMS noise floor measured on flat
        # tile at 0.85 m. Below this a reading cannot be told from sensor noise.
        self.declare_parameter("threshold_mm", 15.0)
        self.declare_parameter("min_area_cm2", 25.0)
        # Beyond 1.6 m the D435's per-pixel noise is larger than the 20 mm step
        # being measured, so anything further away is not evidence. Without this
        # clip the far end of a corridor gets reported as a metres-deep hole,
        # which is long-range noise taken at face value.
        self.declare_parameter("zmin_m", 0.30)
        self.declare_parameter("zmax_m", 1.60)
        # A depth dropout at the edge of the floor deprojects to a point far
        # below the plane and is clustered as one enormous hole. Nothing this
        # robot can drive over is a third of a metre deep, so cap it rather
        # than publish a marker the size of the room.
        self.declare_parameter("max_size_mm", 300.0)
        # When less than this share of the points lies on the plane, the ground
        # is not one flat surface (wood chips: 42 % of frames) and the residual
        # turns into half-image "holes". Skip the hazards, keep the unevenness.
        self.declare_parameter("min_inliers", 0.5)
        # On grass and wood chips (unevenness about 7 mm) a fixed 15 mm boxes
        # tufts, chips and leaves. The threshold is raised to this many times
        # the frame's unevenness; on tile and asphalt (3.5-4 mm) it stays 15 mm.
        self.declare_parameter("rough_factor", 3.0)
        # The D435 is fixed to the gripper, so the gripper blocks the same pixels
        # in every arm pose. The edge of that region produces flying pixels that
        # were read as ~100 mm bumps and false holes. Empty string disables it.
        self.declare_parameter("robot_mask", os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "robot_mask_d435_gripper.png"))
        self.declare_parameter("process_every_n", 3)
        self.declare_parameter("publish_cloud", False)
        self.declare_parameter("cloud_stride", 4)
        self.declare_parameter("marker_lifetime_s", 0.5)

        g = lambda n: self.get_parameter(n).value
        self.threshold_m = float(g("threshold_mm")) / 1000.0
        self.min_area_cm2 = float(g("min_area_cm2"))
        self.zmin = float(g("zmin_m"))
        self.zmax = float(g("zmax_m"))
        self.max_size_mm = float(g("max_size_mm"))
        self.min_inliers = float(g("min_inliers"))
        self.rough_factor = float(g("rough_factor"))
        self.every_n = max(1, int(g("process_every_n")))
        self.publish_cloud = bool(g("publish_cloud"))
        self.cloud_stride = max(1, int(g("cloud_stride")))
        self.marker_lifetime = float(g("marker_lifetime_s"))
        self.mask_ok = None     # True where the image shows ground, not the robot
        path = str(g("robot_mask"))
        if path:
            mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                self.get_logger().warn(f"robot_mask {path} not readable; running without it")
            else:
                self.mask_ok = mask > 0

        self.k = None           # (fx, fy, ppx, ppy), filled by the info callback
        self.frame_count = 0
        self.processed = 0
        self.rng = np.random.default_rng(0)
        self.prev_plane = None  # (normal, d) from the last frame, for tracking

        self.pub_haz = self.create_publisher(Detection3DArray, "/ground/hazards", 10)
        self.pub_mark = self.create_publisher(MarkerArray, "/ground/hazard_markers", 10)
        self.pub_un = self.create_publisher(Float32, "/ground/unevenness", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/ground/residual_cloud", 1)

        self.create_subscription(CameraInfo, g("info_topic"),
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, g("depth_topic"),
                                 self.on_depth, qos_profile_sensor_data)

        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"listening on {g('depth_topic')} | threshold {g('threshold_mm')} mm | "
            f"every {self.every_n} frame(s)")

    # -- inputs ------------------------------------------------------------

    def on_info(self, msg):
        if self.k is None:
            self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info(
                f"intrinsics fx={self.k[0]:.1f} fy={self.k[1]:.1f} "
                f"ppx={self.k[2]:.1f} ppy={self.k[3]:.1f}")

    def on_depth(self, msg):
        self.frame_count += 1
        if self.frame_count % self.every_n:
            return
        if self.k is None:
            return

        depth_m = self.decode(msg)
        if depth_m is None:
            return
        self.processed += 1
        self.process(depth_m, msg.header)

    @staticmethod
    def decode(msg):
        """sensor_msgs/Image -> float32 depth in metres, 0 where invalid."""
        if msg.encoding in ("16UC1", "mono16"):
            raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            return raw.astype(np.float32) * 0.001
        if msg.encoding == "32FC1":
            d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
            return np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)
        return None

    # -- the measurement ---------------------------------------------------

    def process(self, depth_m, header):
        fx, fy, ppx, ppy = self.k
        cloud = deproject(depth_m, fx, fy, ppx, ppy)
        valid = (depth_m > self.zmin) & (depth_m < self.zmax)
        if self.mask_ok is not None and self.mask_ok.shape == valid.shape:
            valid &= self.mask_ok

        pts = cloud[valid]
        if len(pts) < 500:
            return

        # Fit on a sample, not the whole frame: 6000 points pin a plane just as
        # well as 300000 and keep RANSAC inside a frame interval. Track the
        # previous plane first, since consecutive frames barely differ, and only
        # fall back to a full search when that no longer fits.
        sample = pts[self.rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = refine_plane(sample, *self.prev_plane) if self.prev_plane else None
        if fit is None:
            fit = fit_ground_plane(sample, rng=self.rng)
        if fit is None:
            self.prev_plane = None
            self.get_logger().warn("no ground plane in this frame",
                                   throttle_duration_sec=5.0)
            return
        nrm, d, inlier_frac = fit
        self.prev_plane = (nrm, d)

        resid = ((cloud @ nrm) + d).astype(np.float32)
        resid[~valid] = 0.0
        # Depth noise is independent per pixel and grows with range; a 5x5
        # median removes most of it without rounding off a real step. Eroding
        # the valid mask afterwards drops the filter's edge artefacts.
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8),
                          np.ones((5, 5), np.uint8)).astype(bool)
        resid[~valid] = 0.0

        # Unevenness is the roughness of the ground itself, so it is measured
        # only over the points that belong to the plane. Including the hazards
        # would report the book, not the floor it sits on.
        flat = valid & (np.abs(resid) < self.threshold_m)
        unevenness_mm = (float(np.sqrt(np.mean(resid[flat] ** 2))) * 1000.0
                         if flat.any() else 0.0)
        self.pub_un.publish(Float32(data=unevenness_mm))

        if inlier_frac < self.min_inliers:
            hazards = []
        else:
            thresh = max(self.threshold_m, self.rough_factor * unevenness_mm / 1000.0)
            hazards = find_hazards(cloud, valid, resid, nrm, d,
                                   thresh, self.min_area_cm2, fx, fy,
                                   max_height=self.max_size_mm / 1000.0)
            hazards = [h for h in hazards if h["peak_mm"] <= self.max_size_mm]

        self.publish_detections(hazards, nrm, inlier_frac, header)
        self.publish_markers(hazards, nrm, header)
        if self.publish_cloud:
            self.publish_residual_cloud(cloud, valid, resid, header)

        if hazards:
            h = hazards[0]
            height, tilt = plane_pose(nrm, d)
            self.get_logger().info(
                f"{len(hazards)} hazard(s), largest {h['type']} {h['size_mm']:.0f} mm "
                f"({h['length_mm']:.0f}x{h['width_mm']:.0f} mm) at {h['ahead_m']:.2f} m "
                f"| plane {height:.2f} m / {tilt:.0f} deg, {inlier_frac*100:.0f}% inliers "
                f"| unevenness {unevenness_mm:.1f} mm",
                throttle_duration_sec=1.0)

    # -- outputs -----------------------------------------------------------

    def hazard_orientation(self, nrm):
        """A box frame whose z is the plane normal, so markers lie flat on the ground."""
        fwd, right = plane_basis(nrm)
        m = np.column_stack((right, fwd, nrm))
        x, y, z, w = rotation_to_quaternion(m)
        return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

    def publish_detections(self, hazards, nrm, inlier_frac, header):
        out = Detection3DArray(header=header)
        q = self.hazard_orientation(nrm)
        for i, h in enumerate(hazards):
            det = Detection3D(header=header)
            det.id = f"{h['type']}_{i}"

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = h["type"]
            hyp.hypothesis.score = float(inlier_frac)
            det.results.append(hyp)

            bbox = BoundingBox3D()
            cx, cy, cz = h["centroid"]
            bbox.center.position = Point(x=float(cx), y=float(cy), z=float(cz))
            bbox.center.orientation = q
            bbox.size = Vector3(x=float(h["width_mm"]) / 1000.0,
                                y=float(h["length_mm"]) / 1000.0,
                                z=float(h["size_mm"]) / 1000.0)
            det.bbox = bbox
            out.detections.append(det)
        self.pub_haz.publish(out)

    def publish_markers(self, hazards, nrm, header):
        arr = MarkerArray()
        q = self.hazard_orientation(nrm)
        life = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()

        # One DELETEALL, so hazards that have left the view do not linger in RViz.
        clear = Marker(header=header, action=Marker.DELETEALL)
        clear.ns = "ground_hazards"
        arr.markers.append(clear)

        for i, h in enumerate(hazards):
            cx, cy, cz = h["centroid"]
            bump = h["type"] == "bump"
            colour = (ColorRGBA(r=0.89, g=0.29, b=0.28, a=0.55) if bump
                      else ColorRGBA(r=0.16, g=0.47, b=0.84, a=0.55))

            box = Marker(header=header, action=Marker.ADD, type=Marker.CUBE)
            box.ns, box.id, box.lifetime = "ground_hazards", i * 2, life
            box.pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
            box.pose.orientation = q
            box.scale = Vector3(x=max(float(h["width_mm"]) / 1000.0, 0.01),
                                y=max(float(h["length_mm"]) / 1000.0, 0.01),
                                z=max(float(h["size_mm"]) / 1000.0, 0.005))
            box.color = colour
            arr.markers.append(box)

            text = Marker(header=header, action=Marker.ADD,
                          type=Marker.TEXT_VIEW_FACING)
            text.ns, text.id, text.lifetime = "ground_hazards", i * 2 + 1, life
            text.pose.position = Point(x=float(cx), y=float(cy),
                                       z=float(cz) - 0.05)
            text.pose.orientation = Quaternion(w=1.0)
            text.scale = Vector3(x=0.0, y=0.0, z=0.04)
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            text.text = f"{h['type']} {h['size_mm']:.0f} mm"
            arr.markers.append(text)

        self.pub_mark.publish(arr)

    def publish_residual_cloud(self, cloud, valid, resid, header):
        """XYZRGB cloud coloured by signed residual: blue below, grey flat, red above."""
        s = self.cloud_stride
        c = cloud[::s, ::s].reshape(-1, 3)
        r = resid[::s, ::s].reshape(-1)
        v = valid[::s, ::s].reshape(-1)
        c, r = c[v], r[v]
        if not len(c):
            return

        # Full scale at +/- 40 mm, matching the figures in the project plan.
        t = np.clip(r / 0.040, -1.0, 1.0)
        up = np.clip(t, 0.0, 1.0)[:, None]
        down = np.clip(-t, 0.0, 1.0)[:, None]
        flat = np.array([[0.94, 0.94, 0.93]], dtype=np.float32)
        red = np.array([[0.89, 0.29, 0.28]], dtype=np.float32)
        blue = np.array([[0.16, 0.47, 0.84]], dtype=np.float32)
        rgb = flat + (red - flat) * up + (blue - flat) * down

        rgb8 = (np.clip(rgb, 0, 1) * 255).astype(np.uint32)
        packed = (rgb8[:, 0] << 16) | (rgb8[:, 1] << 8) | rgb8[:, 2]

        buf = np.zeros(len(c), dtype=[("x", "<f4"), ("y", "<f4"),
                                      ("z", "<f4"), ("rgb", "<u4")])
        buf["x"], buf["y"], buf["z"] = c[:, 0], c[:, 1], c[:, 2]
        buf["rgb"] = packed

        msg = PointCloud2(header=header, height=1, width=len(buf),
                          is_dense=True, is_bigendian=False,
                          point_step=buf.itemsize, row_step=buf.nbytes,
                          data=buf.tobytes())
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.pub_cloud.publish(msg)

    def report(self):
        if self.frame_count and not self.processed:
            self.get_logger().warn(
                f"{self.frame_count} depth frames in, none processed - "
                "no CameraInfo yet?" if self.k is None else "frames too small")
        elif not self.frame_count:
            self.get_logger().warn(
                "no depth frames received. Check the topic name, and that the "
                "camera is actually publishing (ros2 topic hz ...).")


def main():
    rclpy.init()
    node = GroundHazardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
