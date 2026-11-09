"""Publish the surface type under the robot on /ground/rgb_terrain.

Subscribes to the colour camera, classifies the newest frame a few times per
second and publishes the JSON from docs/schema.md:

    {"label": "grass_dirt", "confidence": 0.87}

The surface changes slowly compared with 30 Hz video, so we keep only the
newest frame and classify it on a timer (2 Hz by default) instead of on
every image. One inference takes about 30 ms on one laptop CPU thread.

Below `min_confidence` the label is "unknown". The model only knows five
surfaces; on ground it has never seen (the drain grate in garden_surfaces_1536)
its confidence drops to about 0.3, and "unknown" is more useful to fusion than
a guess. It does not catch everything: smooth concrete is called asphalt with
high confidence, because the two look alike and there is no concrete class.
"""

import json
from pathlib import Path
from threading import Lock

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rgb_terrain.terrain_onnx import TerrainClassifier


def image_to_bgr(msg) -> np.ndarray:
    """rgb8/bgr8 sensor_msgs/Image -> (H, W, 3) BGR, handling padded rows."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"expected rgb8 or bgr8, got {msg.encoding}")
    data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return img[..., ::-1] if msg.encoding == "rgb8" else img


class RgbTerrainNode(Node):
    def __init__(self):
        super().__init__("rgb_terrain_node")
        models = Path(get_package_share_directory("rgb_terrain")) / "models"
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/ground/rgb_terrain")
        self.declare_parameter("model_path", str(models / "rgb_terrain_v1.onnx"))
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("num_threads", 1)
        self.declare_parameter("min_confidence", 0.5)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.clf = TerrainClassifier(g("model_path"), num_threads=g("num_threads"))
        self.min_confidence = float(g("min_confidence"))
        self.lock = Lock()
        self.latest = None
        self.pub = self.create_publisher(String, g("output_topic"), 10)
        # Camera topics are BEST_EFFORT; a RELIABLE subscriber would get nothing.
        self.create_subscription(Image, g("image_topic"), self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / g("rate_hz"), self.classify_latest)
        self.get_logger().info(f"Classifying {g('image_topic')} into {self.clf.classes} "
                               f"at {g('rate_hz'):g} Hz on {g('output_topic')}")

    def on_image(self, msg):
        with self.lock:
            self.latest = msg

    def classify_latest(self):
        with self.lock:
            msg, self.latest = self.latest, None
        if msg is None:
            return
        try:
            label, confidence, _ = self.clf.predict(image_to_bgr(msg), bgr=True)
        except ValueError as exc:
            self.get_logger().error(f"Terrain classification failed: {exc}", throttle_duration_sec=5.0)
            return
        if confidence < self.min_confidence:
            label = "unknown"
        self.pub.publish(String(data=json.dumps({"label": label, "confidence": round(confidence, 3)})))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RgbTerrainNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
