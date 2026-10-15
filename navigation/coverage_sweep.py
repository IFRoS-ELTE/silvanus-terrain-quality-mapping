#!/usr/bin/env python3
"""
coverage_sweep: drive a boustrophedon (zigzag) sweep over a rectangle with Nav2.

Lanes run along x, spaced along y, alternating direction. The robot is sent
through them with Nav2's FollowWaypoints, so it needs a running Nav2 with a
map frame (SLAM Toolbox or a saved map). The robot's path is logged to CSV from
TF (map -> base_footprint) so the coverage can be plotted afterwards.

    python3 coverage_sweep.py --x0 0.8 --y0 -1.6 --length 2.4 --width 3.2 --spacing 0.8
    (in simulation add: --ros-args -p use_sim_time:=true)

Spacing should be at most the width of ground the camera sees (about 1 m at
the current arm pose) so neighbouring lanes overlap.
"""
import argparse, csv, math, sys, time
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener


def lanes(x0, y0, length, width, spacing):
    """Waypoints (x, y, yaw) for a zigzag over [x0, x0+length] x [y0, y0+width]."""
    n = max(1, int(round(width / spacing)) + 1)
    pts = []
    for i in range(n):
        y = y0 + i * width / (n - 1) if n > 1 else y0
        xs = (x0, x0 + length) if i % 2 == 0 else (x0 + length, x0)
        yaw = 0.0 if i % 2 == 0 else math.pi
        pts += [(xs[0], y, yaw), (xs[1], y, yaw)]
    return pts


def pose(nav, x, y, yaw, frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x0", type=float, default=0.8)
    ap.add_argument("--y0", type=float, default=-1.6)
    ap.add_argument("--length", type=float, default=2.4, help="lane length along x, m")
    ap.add_argument("--width", type=float, default=3.2, help="extent across lanes along y, m")
    ap.add_argument("--spacing", type=float, default=0.8, help="distance between lanes, m")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--log", default="coverage_path.csv")
    a, _ = ap.parse_known_args()       # ROS arguments (--ros-args -p use_sim_time:=true) pass through

    rclpy.init(args=sys.argv)
    nav = BasicNavigator()
    buf = Buffer(); TransformListener(buf, nav)
    wps = lanes(a.x0, a.y0, a.length, a.width, a.spacing)
    nav.get_logger().info(f"{len(wps)} waypoints over {a.length} x {a.width} m, lanes {a.spacing} m apart")
    with open(a.log.replace(".csv", "_waypoints.csv"), "w") as f:
        csv.writer(f).writerows([("x", "y", "yaw")] + wps)

    nav.followWaypoints([pose(nav, x, y, yaw, a.frame) for x, y, yaw in wps])
    out = open(a.log, "w"); out.write("t,x,y\n")
    t0 = time.time(); last_fb = -1
    while not nav.isTaskComplete():
        try:
            tr = buf.lookup_transform(a.frame, "base_footprint", Time(), timeout=Duration(seconds=0.1))
            out.write(f"{time.time() - t0:.2f},{tr.transform.translation.x:.3f},{tr.transform.translation.y:.3f}\n")
        except Exception:
            pass
        fb = nav.getFeedback()
        if fb and fb.current_waypoint != last_fb:
            last_fb = fb.current_waypoint
            nav.get_logger().info(f"waypoint {last_fb + 1}/{len(wps)}")
        time.sleep(0.2)
    res = nav.getResult()
    nav.get_logger().info(f"sweep finished: {'SUCCEEDED' if res == TaskResult.SUCCEEDED else res} in {time.time() - t0:.0f} s")
    out.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
