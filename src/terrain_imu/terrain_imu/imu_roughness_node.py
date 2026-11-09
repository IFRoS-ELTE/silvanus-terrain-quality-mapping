"""Publish the IMU roughness on /terrain/imu_roughness at 4 Hz.

    ros2 run terrain_imu imu_roughness_node
    ros2 launch terrain_imu imu_roughness.launch.py use_sim_time:=true   # with ros2 bag play --clock

Reads the vertical free acceleration from the Xsens (/filter/free_acceleration,
gravity already removed by the Xsens filter) and the forward speed from
/odom. Publishes the fast/slow band ratio from roughness.py as std_msgs/Float32,
NaN when the robot is outside 0.25-0.50 m/s or the window is not full yet.
Publishing NaN instead of nothing matters: a consumer that keeps the latest
value would otherwise carry an old number into a stretch where it means
nothing.

/terrain/imu_roughness_debug carries the same value with the raw ratio and the
speed as JSON, for plotting and for checking a bag replay.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String

from terrain_imu.roughness import RoughnessWindow


class ImuRoughnessNode(Node):
    def __init__(self):
        super().__init__("imu_roughness_node")
        self.declare_parameter("accel_topic", "/filter/free_acceleration")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_topic", "/terrain/imu_roughness")
        self.declare_parameter("rate_hz", 4.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.window = RoughnessWindow()
        self.pub = self.create_publisher(Float32, g("output_topic"), 10)
        self.debug_pub = self.create_publisher(String, g("output_topic") + "_debug", 10)
        # BEST_EFFORT subscriptions receive from both best-effort and reliable publishers.
        self.create_subscription(Vector3Stamped, g("accel_topic"), self.on_accel, qos_profile_sensor_data)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / g("rate_hz"), self.publish)
        self.received = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"IMU roughness from {g('accel_topic')} and {g('odom_topic')} "
                               f"on {g('output_topic')} at {g('rate_hz'):g} Hz")

    def on_accel(self, msg):
        self.window.add_accel(msg.vector.z)
        self.received += 1

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.window.add_speed(t, msg.twist.twist.linear.x)

    def publish(self):
        value = self.window.roughness()
        self.pub.publish(Float32(data=value))
        raw, speed = self.window.ratio(), self.window.mean_speed()
        self.debug_pub.publish(String(data=json.dumps({
            "roughness": None if math.isnan(value) else round(value, 4),
            "ratio": None if math.isnan(raw) else round(raw, 4),
            "speed": None if math.isnan(speed) else round(speed, 3),
        })))

    def report(self):
        if not self.received:
            self.get_logger().warn("No acceleration yet: is the Xsens publishing "
                                   "/filter/free_acceleration (needs the 100 Hz setup)?")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuRoughnessNode()
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
