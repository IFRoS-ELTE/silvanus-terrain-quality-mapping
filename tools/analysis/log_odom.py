#!/usr/bin/env python3
"""Write /odom and /odometry/filtered positions to CSV (sim time)."""
import sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
f = open(sys.argv[1], "w"); f.write("src,t,x,y\n")
class L(Node):
    def __init__(self):
        super().__init__("odom_logger", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: self.w("ekf", m), 50)
        self.create_subscription(Odometry, "/odom", lambda m: self.w("wheel", m), 50)
    def w(self, s, m):
        f.write(f"{s},{m.header.stamp.sec + m.header.stamp.nanosec*1e-9:.3f},{m.pose.pose.position.x:.4f},{m.pose.pose.position.y:.4f}\n")
rclpy.init(); rclpy.spin(L())
