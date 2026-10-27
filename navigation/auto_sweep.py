#!/usr/bin/env python3
"""auto_sweep: map the area and sweep it, with no saved map and nobody driving.

Needs live SLAM (map -> odom, /map) and Nav2 running, and the robot standing where the sweep should
start, facing the area. Then:
  1. it turns once on the spot, so the lidar maps everything around it
  2. it fits lanes into the free floor of the live map, up to --length m ahead and --width m across
     (navigation/lane_planner.py: at least 0.6 m from walls and unknown space)
  3. it drives them through Nav2 (FollowWaypoints, every corner facing the next leg)
and logs the robot's path. Nothing else is needed: through the speed governor the robot still slows
at hazards and stops at obstacles.

    python3 auto_sweep.py --length 5 --width 5            # plan, then sweep
    python3 auto_sweep.py --length 5 --width 5 --dry      # turn and plan only, print the lanes
"""
import argparse, csv, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lane_planner as lp


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def robot_pose(nav, buf):
    tr = buf.lookup_transform("map", "base_footprint", Time(), timeout=Duration(seconds=1.0))
    q = tr.transform.rotation
    return (tr.transform.translation.x, tr.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=5.0, help="how far ahead the area reaches, m")
    ap.add_argument("--width", type=float, default=5.0, help="how wide the area is, m, centred on the robot")
    ap.add_argument("--side", type=float, default=0.0, help="shift the area left (+) or right (-), m")
    ap.add_argument("--start", type=float, default=0.3, help="where the lanes begin, m ahead of the robot")
    ap.add_argument("--no-spin", action="store_true", help="skip the turn on the spot")
    ap.add_argument("--dry", action="store_true", help="plan and print, do not drive")
    ap.add_argument("--log", default="auto_sweep.csv")
    a, _ = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    nav = BasicNavigator("auto_sweep")
    buf = Buffer(); TransformListener(buf, nav)
    grid = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    nav.create_subscription(OccupancyGrid, "/map", lambda m: grid.update(m=m), qos)
    log = nav.get_logger()

    t0 = time.time()
    while ("m" not in grid or not buf.can_transform("map", "base_footprint", Time())) and time.time() - t0 < 30:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if "m" not in grid:
        log.error("no /map after 30 s: is SLAM running?"); sys.exit(1)
    start = robot_pose(nav, buf)

    if not a.no_spin:
        log.info("turning once on the spot to map the surroundings")
        nav.spin(spin_dist=2 * math.pi, time_allowance=90)
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.2)
        t1 = time.time()
        while time.time() - t1 < 3.0:              # let SLAM publish the map with what it saw
            rclpy.spin_once(nav, timeout_sec=0.2)

    m = grid["m"]
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    origin = (m.info.origin.position.x, m.info.origin.position.y)
    here = robot_pose(nav, buf)
    # lanes along the heading the robot was placed with, from where it stands now
    wps, lanes = lp.plan(g, m.info.resolution, origin, (here[0], here[1], start[2]), a.length, a.width,
                         start=a.start, side=a.side)
    d = lp.clearance_map(g, m.info.resolution)
    log.info(f"map {m.info.width}x{m.info.height} cells at {m.info.resolution} m, origin ({origin[0]:.2f}, {origin[1]:.2f}); "
             f"robot at ({here[0]:.2f}, {here[1]:.2f}); free {int((g == 0).sum() * m.info.resolution ** 2)} m2, "
             f"of it {int((d >= lp.CLEARANCE).sum() * m.info.resolution ** 2)} m2 at least {lp.CLEARANCE} m from walls")
    if not lanes:
        log.error("no free floor for a lane in that area"); sys.exit(1)
    total = sum(math.dist(p, q) for p, q in lanes)
    log.info(f"{len(lanes)} lanes, {total:.1f} m of lanes, about {lp.covered_area(lanes):.0f} m2 of floor seen")
    for k, (p, q) in enumerate(lanes):
        log.info(f"  lane {k + 1}: ({p[0]:.2f}, {p[1]:.2f}) -> ({q[0]:.2f}, {q[1]:.2f}), {math.dist(p, q):.1f} m")
    with open(a.log.replace(".csv", "_plan.json"), "w") as f:
        json.dump(dict(start=start, area=dict(length=a.length, width=a.width, side=a.side),
                       lanes=lanes, waypoints=wps), f, indent=1)
    if a.dry:
        rclpy.shutdown()
        return

    nav.followWaypoints([pose(nav, x, y, yw) for x, y, yw in wps])
    out = open(a.log, "w"); out.write("t,x,y\n")
    t0 = time.time(); last = -1
    try:
        while not nav.isTaskComplete():
            try:
                x, y, _ = robot_pose(nav, buf)
                out.write(f"{time.time() - t0:.2f},{x:.3f},{y:.3f}\n")
            except Exception:
                pass
            fb = nav.getFeedback()
            if fb and fb.current_waypoint != last:
                last = fb.current_waypoint
                log.info(f"waypoint {last + 1}/{len(wps)}")
            time.sleep(0.2)
    except KeyboardInterrupt:          # Ctrl-C must stop the robot, not only this script
        nav.cancelTask()
        log.info("sweep cancelled")
    res = nav.getResult()
    log.info(f"sweep finished: {'SUCCEEDED' if res == TaskResult.SUCCEEDED else res} in {time.time() - t0:.0f} s")
    out.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
