#!/usr/bin/env python3
"""A stand-in for Silvanus on a saved map: wheels and the floor-free lidar scan, nothing else.

Drives on /cmd_vel like the Scout base (plain Twist, acceleration limited), publishes /odom and
odom -> base_footprint with a small, realistic drift (the real wheels lose 1.5-2 m per 20 m loop),
and /scan_fixed by ray casting the map from where the robot really is. Enough to run the real
Nav2 parameters, the speed governor and the coverage sweep on the laptop, without the robot:

    python3 tools/sim/fake_robot.py MAP.yaml --start X Y YAW_DEG
    # then Nav2 with navigation/nav2_robot*.yaml, the governor, the sweep, as on the robot

Ground truth (where the robot really is in the map frame) goes to /sim/true_pose for checking.
"""

import argparse
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

V_MAX, W_MAX = 1.0, 1.5            # m/s, rad/s
ACC_V, ACC_W = 0.6, 1.2            # m/s^2, rad/s^2
CMD_TIMEOUT = 0.5                  # the base stops when commands stop
ODOM_SCALE_V, ODOM_SCALE_W = 1.02, 1.05   # wheel odometry over-reads distance and turn: skid-steer slip
SCAN_N, SCAN_INC, SCAN_MAX = 722, 0.0087, 20.0


def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class FakeRobot(Node):
    def __init__(self, map_yaml, start, odom_error=(ODOM_SCALE_V, ODOM_SCALE_W)):
        super().__init__("fake_robot")
        meta = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta["image"])))
        self.occ = np.flipud(img < 128)                       # row 0 = lowest y
        self.res, (self.ox, self.oy) = meta["resolution"], meta["origin"][:2]
        self.true = np.array(start, float)                     # x, y, yaw in the map frame
        self.true[2] = math.radians(self.true[2])
        self.odom = np.zeros(3)                                # the wheels' own estimate, starts at 0
        self.v = self.w = 0.0
        self.cmd, self.cmd_t = (0.0, 0.0), -1e9
        self.rng = np.random.default_rng(0)
        self.scale_v, self.scale_w = odom_error
        self.angles = -math.pi + np.arange(SCAN_N) * SCAN_INC
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan_fixed", qos_profile_sensor_data)
        self.pub_true = self.create_publisher(PoseStamped, "/sim/true_pose", 10)
        self.tfb = TransformBroadcaster(self)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.scan)

    def on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def free(self, x, y):
        c, r = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        return 0 <= r < self.occ.shape[0] and 0 <= c < self.occ.shape[1] and not self.occ[r, c]

    def step(self):
        v_cmd, w_cmd = self.cmd if self.now_s() - self.cmd_t < CMD_TIMEOUT else (0.0, 0.0)
        v_cmd, w_cmd = np.clip(v_cmd, -V_MAX, V_MAX), np.clip(w_cmd, -W_MAX, W_MAX)
        self.v += np.clip(v_cmd - self.v, -ACC_V * self.dt, ACC_V * self.dt)
        self.w += np.clip(w_cmd - self.w, -ACC_W * self.dt, ACC_W * self.dt)
        x, y, th = self.true
        nx, ny = x + self.v * math.cos(th) * self.dt, y + self.v * math.sin(th) * self.dt
        if self.free(nx, ny):                                  # a wall stops the robot, it does not pass through
            self.true[:2] = nx, ny
        else:
            self.v = 0.0
        self.true[2] = th + self.w * self.dt
        ov, ow = self.v * self.scale_v, self.w * self.scale_w
        self.odom += (ov * math.cos(self.odom[2]) * self.dt, ov * math.sin(self.odom[2]) * self.dt, ow * self.dt)

        stamp = self.get_clock().now().to_msg()
        qz, qw = quat(self.odom[2])
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = float(self.odom[0]), float(self.odom[1])
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tfb.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.odom[0]), float(self.odom[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(ov), float(ow)
        self.pub_odom.publish(o)
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, "map"
        p.pose.position.x, p.pose.position.y = float(self.true[0]), float(self.true[1])
        p.pose.orientation.z, p.pose.orientation.w = quat(self.true[2])
        self.pub_true.publish(p)

    def scan(self):
        x, y, th = self.true
        a = th + self.angles
        steps = np.arange(0.2, SCAN_MAX, self.res / 2)
        px = x + np.outer(np.cos(a), steps)
        py = y + np.outer(np.sin(a), steps)
        c = ((px - self.ox) / self.res).astype(int)
        r = ((py - self.oy) / self.res).astype(int)
        inside = (r >= 0) & (r < self.occ.shape[0]) & (c >= 0) & (c < self.occ.shape[1])
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[r[inside], c[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, steps[np.maximum(first, 0)], np.inf)
        ranges = ranges + self.rng.normal(0, 0.015, ranges.shape)
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "base_footprint"
        m.angle_min, m.angle_increment = -math.pi, SCAN_INC
        m.angle_max = m.angle_min + (SCAN_N - 1) * SCAN_INC
        m.scan_time, m.range_min, m.range_max = 0.1, 0.2, 100.0
        m.ranges = [float(v) for v in ranges]
        self.pub_scan.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("map_yaml")
    ap.add_argument("--start", type=float, nargs=3, default=(3.037, -3.203, 66.34), metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--odom-error", type=float, nargs=2, default=(ODOM_SCALE_V, ODOM_SCALE_W), metavar=("V_SCALE", "W_SCALE"),
                    help="how much the wheels over-read distance and turn (1 1 = perfect odometry)")
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = FakeRobot(a.map_yaml, a.start, a.odom_error)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
