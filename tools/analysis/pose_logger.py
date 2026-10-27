#!/usr/bin/env python3
"""Log map->base_footprint (SLAM) and odom->base_footprint (wheels) every 0.2 s of sim time to a CSV."""
import math, sys, rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
out = open(sys.argv[1], "w"); out.write("t,map_x,map_y,odom_x,odom_y,map_yaw\n")
class L(Node):
    def __init__(self):
        super().__init__("pose_logger", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.buf = Buffer(); TransformListener(self.buf, self); self.create_timer(0.2, self.tick)
    def tick(self):
        try:
            a = self.buf.lookup_transform("map", "base_footprint", Time()); b = self.buf.lookup_transform("odom", "base_footprint", Time())
        except Exception: return
        q = a.transform.rotation
        out.write(f"{a.header.stamp.sec + a.header.stamp.nanosec*1e-9:.2f},{a.transform.translation.x:.3f},{a.transform.translation.y:.3f},"
                  f"{b.transform.translation.x:.3f},{b.transform.translation.y:.3f},"
                  f"{math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)):.4f}\n"); out.flush()
rclpy.init(); rclpy.spin(L())
