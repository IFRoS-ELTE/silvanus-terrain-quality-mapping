#!/usr/bin/env python3
"""Republish /scan as /scan_clean with the floor ring removed: beams between LO and HI metres become +inf.
The lab's scan band (-0.10 to 0.60 m in base_footprint) includes the floor, which the lidar's lowest
beams hit at about 1.4 m. Anything beyond that ring is already lost, because the scan keeps the closest point."""
import math, sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
LO, HI = float(sys.argv[1]), float(sys.argv[2])
class Clean(Node):
    def __init__(self):
        super().__init__("clean_scan", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(LaserScan, "/scan_clean", 10)
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
    def cb(self, m):
        m.ranges = [math.inf if LO <= r <= HI else r for r in m.ranges]; self.pub.publish(m)
rclpy.init(); rclpy.spin(Clean())
