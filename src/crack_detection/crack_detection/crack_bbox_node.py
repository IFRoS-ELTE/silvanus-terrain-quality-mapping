"""Detect dark crack-like regions in RGB images and draw bounding boxes."""

import json
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CrackBBoxNode(Node):
    def __init__(self) -> None:
        super().__init__('crack_bbox_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('debug_topic', '/crack_detection/debug_image')
        self.declare_parameter('detections_topic', '/crack_detection/detections')
        self.declare_parameter('dark_threshold', 80)
        self.declare_parameter('min_area', 120.0)
        self.declare_parameter('max_area', 12000.0)
        self.declare_parameter('min_width', 12)
        self.declare_parameter('min_height', 4)
        self.declare_parameter('roi_y_min_ratio', 0.30)
        self.declare_parameter('edge_margin_ratio', 0.04)
        self.declare_parameter('morph_kernel_size', 5)

        self.bridge = CvBridge()

        image_topic = self.get_parameter('image_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        detections_topic = self.get_parameter('detections_topic').value

        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        self.create_subscription(
            Image,
            image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'waiting for images on {image_topic}; debug output on {debug_topic}'
        )

    def on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'failed to convert image: {exc}')
            return

        boxes = self.detect_cracks(frame)
        debug = self.draw_debug(frame, boxes)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        payload = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
            'count': len(boxes),
            'boxes': boxes,
        }
        self.detections_pub.publish(String(data=json.dumps(payload)))

        if boxes:
            self.get_logger().info(
                f'detected {len(boxes)} crack candidate(s)',
                throttle_duration_sec=1.0,
            )

    def detect_cracks(self, frame: np.ndarray) -> list[dict[str, Any]]:
        dark_threshold = int(self.get_parameter('dark_threshold').value)
        min_area = float(self.get_parameter('min_area').value)
        max_area = float(self.get_parameter('max_area').value)
        min_width = int(self.get_parameter('min_width').value)
        min_height = int(self.get_parameter('min_height').value)
        roi_y_min_ratio = float(self.get_parameter('roi_y_min_ratio').value)
        edge_margin_ratio = float(self.get_parameter('edge_margin_ratio').value)
        morph_kernel_size = int(self.get_parameter('morph_kernel_size').value)

        height, width = frame.shape[:2]
        roi_y_min = int(max(0.0, min(1.0, roi_y_min_ratio)) * height)
        edge_margin = int(max(0.0, min(0.5, edge_margin_ratio)) * width)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(
            gray,
            dark_threshold,
            255,
            cv2.THRESH_BINARY_INV,
        )

        mask[:roi_y_min, :] = 0

        kernel_size = max(1, morph_kernel_size)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        boxes: list[dict[str, Any]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue

            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < min_width or box_height < min_height:
                continue
            if x <= edge_margin or x + box_width >= width - edge_margin:
                continue

            aspect = box_width / max(1, box_height)
            if aspect < 1.2 and box_height < 30:
                continue

            boxes.append({
                'x': int(x),
                'y': int(y),
                'width': int(box_width),
                'height': int(box_height),
                'area': area,
                'score': round(min(1.0, area / 3000.0), 3),
            })

        boxes.sort(key=lambda item: item['area'], reverse=True)
        return boxes

    @staticmethod
    def draw_debug(
        frame: np.ndarray,
        boxes: list[dict[str, Any]],
    ) -> np.ndarray:
        debug = frame.copy()
        for index, box in enumerate(boxes, start=1):
            x = box['x']
            y = box['y']
            width = box['width']
            height = box['height']
            cv2.rectangle(
                debug,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2,
            )
            label = f'crack {index}'
            cv2.putText(
                debug,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return debug


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrackBBoxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
