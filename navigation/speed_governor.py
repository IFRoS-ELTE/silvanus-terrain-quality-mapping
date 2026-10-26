#!/usr/bin/env python3
"""Speed governor: slow down before a hazard so the IMU can feel it, stop before an obstacle.

Sits between whatever drives the robot (teleop, Nav2) and the base:

    teleop / Nav2 --/cmd_vel_in--> speed_governor --/cmd_vel--> Scout base

    python3 navigation/speed_governor.py --ros-args -p enable:=true
    # teleop into the governor:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_in

What it does with the forward speed (turning and reversing always pass):
  stop   a bump or hole of stop_height_m or more on the robot's track, within stop_distance_m
         of the front wheels, or a lidar return within stop_range_m of the front bumper
  slow   any other hazard from the depth node on the track ahead, until the rear wheels have
         passed it: confirm_speed (0.20 m/s). On the 2026-10-08 runs the IMU felt rigid
         objects from 15 mm at 0.2-0.3 m/s, while above 0.4 m/s the robot's own shaking on
         tile hid every object (docs/PROGRESS.md section 2)
  free   otherwise, up to max_speed

While the wheels cross a hazard it measures the vertical jolt (/filter/free_acceleration
minus its 0.5 s running median) and publishes on /governor/felt whether the hazard was felt
(class C) or not (class A), with the rule of tools/analysis/hazard_map.py.

Inputs: /cmd_vel_in (Twist), /ground/hazards (Detection3DArray, depth node), /odom,
/filter/free_acceleration, /scan_fixed (LaserScan in base_footprint), TF odom <- camera.
Outputs: /cmd_vel (plain Twist: the Scout driver ignores TwistStamped), /governor/state
and /governor/felt (JSON strings).
With enable false (the default) nothing is published on /cmd_vel, only the state.
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

FRONT_AXLE, REAR_AXLE = 0.249, -0.249          # scout_v2.xacro wheelbase 0.498 m
BUMPER = 0.47                                   # front bumper ahead of the robot centre, m
TRACK_HALF = 0.2915 + 0.06 + 0.10               # half track + half tyre + margin: what the wheels can touch


def quat_matrix(q, t):
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def decide(hazards, scan_min, p):
    """Speed limit for the hazards around the robot, in the robot frame (x forward from the centre, y left).

    hazards: dicts with name, h (height or depth, m), bx, by (centre, m), half_len (m).
    scan_min: nearest lidar return in front of the bumper, m. Returns (limit, mode, reason).
    """
    limit, mode, why = p["max_speed"], "free", ""
    for hz in hazards:
        if abs(hz["by"]) > TRACK_HALF + hz["half_len"]:
            continue                                  # beside the track: the wheels cannot touch it
        gap = hz["bx"] - FRONT_AXLE - hz["half_len"]  # front wheels to the near edge
        if hz["h"] >= p["stop_height_m"] and 0 < gap < p["stop_distance_m"]:
            return 0.0, "stop", f"{hz['name']} {gap:.2f} m ahead of the front wheels"
        if hz["bx"] - FRONT_AXLE < p["slow_ahead_m"] and limit > p["confirm_speed"]:
            limit, mode, why = p["confirm_speed"], "slow", f"{hz['name']}, until the rear wheels are past it"
    if scan_min < p["stop_range_m"]:
        return 0.0, "stop", f"obstacle {scan_min:.2f} m in front of the bumper (lidar)"
    return limit, mode, why


class SpeedGovernor(Node):
    def __init__(self):
        super().__init__("speed_governor")
        defaults = dict(enable=False, input_topic="/cmd_vel_in", output_topic="/cmd_vel", scan_topic="/scan_fixed",
                        max_speed=0.5, confirm_speed=0.20, stop_height_m=0.10, stop_distance_m=0.35,
                        stop_range_m=0.40, scan_half_angle_deg=35.0, slow_ahead_m=1.6, input_timeout_s=0.5,
                        odom_frame="odom", base_frame="base_footprint", merge_m=0.3)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.hazards = []                 # tracked in odom: dict(x, y, h, kind, ...)
        self.cmd, self.cmd_time = Twist(), None
        self.scan_min, self.jolt = math.inf, deque(maxlen=1500)   # 15 s at 100 Hz: (t, |a - median|)
        self.az = deque(maxlen=51)
        self.used_jolts = []              # one jolt confirms one hazard: wheels crossing two tufts feel one bump
        self.pub_cmd = self.create_publisher(Twist, self.p["output_topic"], 10) if self.p["enable"] else None
        self.pub_state = self.create_publisher(String, "/governor/state", 10)
        self.pub_felt = self.create_publisher(String, "/governor/felt", 10)
        self.create_subscription(Twist, self.p["input_topic"], self.on_cmd, 10)
        self.create_subscription(Detection3DArray, "/ground/hazards", self.on_hazards, 10)
        self.create_subscription(Vector3Stamped, "/filter/free_acceleration", self.on_acc, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p["scan_topic"], self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.mode = None
        self.get_logger().warn(
            "enable is false: /cmd_vel is NOT published, only /governor/state. Re-run with -p enable:=true "
            "with an operator at the e-stop." if not self.p["enable"] else
            f"Driving {self.p['input_topic']} -> {self.p['output_topic']}: slow to {self.p['confirm_speed']} m/s "
            f"at hazards, stop at {self.p['stop_height_m'] * 1000:.0f} mm or {self.p['stop_range_m']} m from the bumper")

    # ---- inputs
    def on_cmd(self, msg):
        self.cmd, self.cmd_time = msg, self.get_clock().now()

    def on_scan(self, msg):
        half = math.radians(self.p["scan_half_angle_deg"])
        a = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        r = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= msg.range_min) & (np.abs(np.arctan2(np.sin(a), np.cos(a))) <= half)
        ahead = r[ok] * np.cos(a[ok]) - BUMPER          # distance in front of the bumper (scan is in base_footprint)
        self.scan_min = float(ahead.min()) if len(ahead) else math.inf

    def on_acc(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.az.append(msg.vector.z)
        self.jolt.append((t, abs(msg.vector.z - float(np.median(self.az)))))

    def on_hazards(self, msg):
        if not msg.detections:
            return
        try:
            tr = self.tf.lookup_transform(self.p["odom_frame"], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            try:        # a depth frame a few ms newer than the last odometry: the latest pose is close enough
                tr = self.tf.lookup_transform(self.p["odom_frame"], msg.header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(f"hazards not placed: {exc}", throttle_duration_sec=5.0)
                return
        M = quat_matrix(tr.transform.rotation, tr.transform.translation)
        for det in msg.detections:
            c = det.bbox.center.position
            x, y, _ = (M @ np.array([c.x, c.y, c.z, 1.0]))[:3]
            kind = det.results[0].hypothesis.class_id if det.results else "bump"
            h, length = det.bbox.size.z, det.bbox.size.y
            for hz in self.hazards:
                if hz["kind"] == kind and math.hypot(hz["x"] - x, hz["y"] - y) < self.p["merge_m"]:
                    n = hz["seen"]; hz["x"] = (hz["x"] * n + x) / (n + 1); hz["y"] = (hz["y"] * n + y) / (n + 1)
                    hz["h"] = max(hz["h"], h); hz["seen"] += 1
                    break
            else:
                self.hazards.append(dict(id=f"G{len(self.hazards) + 1}", kind=kind, x=x, y=y, h=h, length=length,
                                         seen=1, crossing=None, done=False))

    # ---- control
    def robot_frame(self):
        tr = self.tf.lookup_transform(self.p["odom_frame"], self.p["base_frame"], Time())
        M = quat_matrix(tr.transform.rotation, tr.transform.translation)
        return np.linalg.inv(M)

    def tick(self):
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        fresh = self.cmd_time is not None and (now - self.cmd_time).nanoseconds * 1e-9 < self.p["input_timeout_s"]
        out = Twist()
        if fresh:
            out.angular.z = self.cmd.angular.z
            out.linear.x = min(self.cmd.linear.x, self.p["max_speed"])
        try:
            Minv = self.robot_frame()
        except TransformException:
            Minv = None
        ahead = []
        if Minv is not None:
            t = now.nanoseconds * 1e-9
            for hz in self.hazards:
                if hz["done"]:
                    continue
                bx, by, _ = (Minv @ np.array([hz["x"], hz["y"], 0.0, 1.0]))[:3]
                half_len = min(hz["length"] / 2, 0.6)
                if bx < REAR_AXLE - half_len - 0.2:          # the rear wheels are past it
                    self.finish(hz)
                    continue
                on_track = abs(by) <= TRACK_HALF + half_len
                if on_track and (abs(bx - FRONT_AXLE) < half_len + 0.15 or abs(bx - REAR_AXLE) < half_len + 0.15):
                    hz["crossing"] = (hz["crossing"] or (t, t))[0], t
                ahead.append(dict(name=f"{hz['kind']} {hz['h'] * 1000:.0f} mm ({hz['id']})", h=hz["h"],
                                  bx=bx, by=by, half_len=half_len))
        limit, mode, why = decide(ahead, self.scan_min, self.p)
        if out.linear.x > limit:
            out.linear.x = limit
        if mode != self.mode:
            self.get_logger().info(f"{mode.upper()}: {why}" if why else mode.upper())
            self.mode = mode
        self.pub_state.publish(String(data=json.dumps({"mode": mode, "limit": limit, "reason": why,
                                                       "command": round(self.cmd.linear.x, 2) if fresh else None,
                                                       "out": round(out.linear.x, 2)})))
        if self.pub_cmd is not None and (fresh or self.cmd_time is not None):
            self.pub_cmd.publish(out)                    # zeros once the input stops: never keep the last speed

    def finish(self, hz):
        """Rear wheels past: was it felt? Same rule as tools/analysis/hazard_map.py."""
        hz["done"] = True
        if hz["crossing"] is None or len(self.jolt) < 200:
            return
        J = np.array(self.jolt)
        t0, t1 = hz["crossing"]
        w = (J[:, 0] >= t0 - 0.3) & (J[:, 0] <= t1 + 0.3)
        before = (J[:, 0] < t0 - 1.0) & (J[:, 0] > t0 - 5.0)
        if not w.any() or before.sum() < 50:
            return
        i = np.flatnonzero(w)[np.argmax(J[w, 1])]
        peak, tj = float(J[i, 1]), float(J[i, 0])
        p75, p95 = np.percentile(J[before, 1], [75, 95])
        felt = bool(peak > max(2.0 * p75, 1.3 * p95, 0.75)) and not any(abs(tj - u) < 0.3 for u in self.used_jolts)
        if felt:
            self.used_jolts.append(tj)
        res = dict(id=hz["id"], type=hz["kind"], height_mm=round(hz["h"] * 1000), x=round(hz["x"], 2), y=round(hz["y"], 2),
                   imu_peak=round(peak, 2), felt=felt, **{"class": "C" if felt else "A"})
        self.pub_felt.publish(String(data=json.dumps(res)))
        self.get_logger().info(f"{hz['id']} {hz['kind']} {res['height_mm']} mm: jolt {peak:.2f} m/s^2, "
                               f"{'felt (C)' if felt else 'not felt (A)'}")

    def stop(self):
        """Zero the base on the way out. After Ctrl-C the context may already be gone."""
        if self.pub_cmd is None:
            return
        try:
            for _ in range(5):
                self.pub_cmd.publish(Twist())
        except Exception:
            pass


def main():
    rclpy.init()
    node = SpeedGovernor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
