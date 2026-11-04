#!/usr/bin/env python3
"""
coverage_sweep: drive a boustrophedon (zigzag) sweep over a rectangle with Nav2.

Lanes run along x, spaced along y, alternating direction. The robot is sent
through them with Nav2's FollowWaypoints, so it needs a running Nav2 with a
map frame (SLAM Toolbox or a saved map). The robot's path is logged to CSV from
TF (map -> base_footprint) so the coverage can be plotted afterwards.

    python3 coverage_sweep.py --x0 0.8 --y0 -1.6 --length 2.4 --width 3.2 --spacing 0.8
    (in simulation add: --ros-args -p use_sim_time:=true)

--yaw turns the rectangle about (x0, y0). On the indoor_loop3_1316 map the corridor runs at 66 deg:
    python3 coverage_sweep.py --x0 3.875 --y0 -0.294 --yaw 66.34 --length 5 --width 1.6 --spacing 0.8

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


def lanes(x0, y0, length, width, spacing, rot=0.0):
    """Waypoints (x, y, yaw) for a zigzag over a length x width rectangle with a corner at (x0, y0),
    its lanes turned by rot (rad) from the map x axis, e.g. to run along a corridor.

    Each corner faces the next leg, so at a lane end the robot turns on the spot towards the next
    lane, drives straight across and turns again. With the lane's own heading at every corner the
    next lane start sat beside the robot, which a skid-steer robot that only drives forwards could
    not settle on (2026-10-27: 190 s hovering 0.23-0.5 m from it, goal tolerance 0.20 m)."""
    n = max(1, int(round(width / spacing)) + 1)
    c, s = math.cos(rot), math.sin(rot)
    xy = []
    for i in range(n):
        v = i * width / (n - 1) if n > 1 else 0.0
        us = (0.0, length) if i % 2 == 0 else (length, 0.0)
        xy += [(x0 + u * c - v * s, y0 + u * s + v * c) for u in us]
    yaws = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(xy, xy[1:])]
    return [(x, y, yaw) for (x, y), yaw in zip(xy, yaws + yaws[-1:])]


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
    ap.add_argument("--yaw", type=float, default=0.0, help="lane direction from the map x axis, deg")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--log", default="coverage_path.csv")
    a, _ = ap.parse_known_args()       # ROS arguments (--ros-args -p use_sim_time:=true) pass through

    rclpy.init(args=sys.argv)
    nav = BasicNavigator()
    buf = Buffer(); TransformListener(buf, nav)
    wps = lanes(a.x0, a.y0, a.length, a.width, a.spacing, math.radians(a.yaw))
    nav.get_logger().info(f"{len(wps)} waypoints over {a.length} x {a.width} m, lanes {a.spacing} m apart")
    with open(a.log.replace(".csv", "_waypoints.csv"), "w") as f:
        csv.writer(f).writerows([("x", "y", "yaw")] + wps)

    # FollowWaypoints, not goThroughPoses: Nav2 checks only the last pose of a through-poses goal, and
    # in a zigzag that lies near the start, so the sweep "succeeded" without driving the lanes (sim)
    nav.followWaypoints([pose(nav, x, y, yaw, a.frame) for x, y, yaw in wps])
    out = open(a.log, "w"); out.write("t,x,y\n")
    t0 = time.time(); last_fb = -1
    try:
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
    except KeyboardInterrupt:          # Ctrl-C must stop the robot, not only this script
        nav.cancelTask()
        nav.get_logger().info("sweep cancelled")
    res = nav.getResult()
    nav.get_logger().info(f"sweep finished: {'SUCCEEDED' if res == TaskResult.SUCCEEDED else res} in {time.time() - t0:.0f} s")
    out.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
