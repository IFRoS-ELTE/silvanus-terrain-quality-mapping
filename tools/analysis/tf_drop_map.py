#!/usr/bin/env python3
"""Relay a bag's /tf (played as /tf_bag) without the transforms whose parent is map.

Bags recorded while Nav2 ran hold AMCL's map -> odom. Offline SLAM publishes its own, and two
publishers of one transform make TF jump between them. run_slam.sh plays the bag through this.
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class Relay(Node):
    def __init__(self):
        super().__init__("tf_drop_map", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_bag", self.relay, 100)

    def relay(self, m):
        keep = [t for t in m.transforms if t.header.frame_id != "map"]
        if keep:
            self.pub.publish(TFMessage(transforms=keep))


rclpy.init()
try:
    rclpy.spin(Relay())
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
