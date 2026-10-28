#!/usr/bin/env python3
"""auto_sweep: map the area and sweep it, with no saved map and nobody driving.

Needs live SLAM (map -> odom, /map) and Nav2 running, and the robot standing where the sweep should
start, facing the area. Then:
  1. it turns once on the spot, so the lidar maps everything around it (only with 1.25 m free all
     round: the arm's camera reaches 1.05 m ahead)
  2. it fits lanes into the free floor of the live map, up to --length m ahead and --width m across
     (navigation/lane_planner.py: at least 0.6 m from walls and unknown space)
  3. it drives them through Nav2 (FollowWaypoints, every corner facing the next leg)
and logs the robot's path. Nothing else is needed: through the speed governor the robot still slows
at hazards and stops at obstacles.

With --room there is no area to give: it sweeps the whole room it stands in (lane_planner: the open
floor it can reach without passing a door), then plans again on the map it has built meanwhile and
sweeps the floor its camera has not seen yet, along or across, up to --rounds passes or until
nothing is left.

    python3 auto_sweep.py --length 5 --width 5            # plan, then sweep
    python3 auto_sweep.py --length 5 --width 5 --dry      # turn and plan only, print the lanes
    python3 auto_sweep.py --room                          # the whole room
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
    ap.add_argument("--room", action="store_true", help="the whole room instead of --length x --width")
    ap.add_argument("--rounds", type=int, default=3, help="with --room: passes at most, each over floor not yet seen")
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
        m0 = grid["m"]
        g0 = np.array(m0.data, dtype=np.int16).reshape(m0.info.height, m0.info.width)
        near = lp.nearest_obstacle(g0, m0.info.resolution, (m0.info.origin.position.x, m0.info.origin.position.y), start[:2])
        if near < lp.TURN_CLEARANCE:
            log.error(f"something {near:.2f} m from the robot centre: turning on the spot with the arm out needs "
                      f"{lp.TURN_CLEARANCE} m all round. Move the robot to open floor, or use --no-spin.")
            sys.exit(1)
        log.info("turning once on the spot to map the surroundings")
        nav.spin(spin_dist=2 * math.pi, time_allowance=90)
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.2)
        t1 = time.time()
        while time.time() - t1 < 3.0:              # let SLAM publish the map with what it saw
            rclpy.spin_once(nav, timeout_sec=0.2)

    out = None if a.dry else open(a.log, "w")
    if out:
        out.write("t,x,y,yaw\n")
    camera = []                        # where the camera has looked at the floor, map frame
    t0 = time.time()
    for rnd in range(1, (a.rounds if a.room else 1) + 1):
        m = grid["m"]
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        origin = (m.info.origin.position.x, m.info.origin.position.y)
        here = robot_pose(nav, buf)
        seen = lp.seen_mask(g.shape, m.info.resolution, origin, camera) if camera else None
        # lanes along the heading the robot was placed with, from where it stands now; later passes
        # also try lanes across, which reach the ends of the earlier lanes and the floor beside furniture
        best = None
        for heading in [start[2]] + ([start[2] + math.pi / 2] if rnd > 1 else []):
            plan = lp.plan(g, m.info.resolution, origin, (here[0], here[1], heading),
                           None if a.room else a.length, a.width, start=a.start, side=a.side, seen=seen)
            if best is None or sum(math.dist(p, q) for p, q in plan[1]) > sum(math.dist(p, q) for p, q in best[1]):
                best = plan
        wps, lanes = best
        d = lp.clearance_map(g, m.info.resolution)
        log.info(f"map {m.info.width}x{m.info.height} cells at {m.info.resolution} m, origin ({origin[0]:.2f}, {origin[1]:.2f}); "
                 f"robot at ({here[0]:.2f}, {here[1]:.2f}); free {int((g == 0).sum() * m.info.resolution ** 2)} m2, "
                 f"of it {int((d >= lp.CLEARANCE).sum() * m.info.resolution ** 2)} m2 at least {lp.CLEARANCE} m from walls")
        total = sum(math.dist(p, q) for p, q in lanes)
        if rnd > 1 and not lanes:
            log.info(f"pass {rnd}: nothing left that the camera has not seen and the arm has room to reach")
            break
        if not lanes:
            log.error("no free floor for a lane in that area"); sys.exit(1)
        log.info(f"pass {rnd}: {len(lanes)} lanes, {total:.1f} m of lanes, about {lp.covered_area(lanes):.0f} m2 of floor seen")
        for k, (p, q) in enumerate(lanes):
            log.info(f"  lane {k + 1}: ({p[0]:.2f}, {p[1]:.2f}) -> ({q[0]:.2f}, {q[1]:.2f}), {math.dist(p, q):.1f} m")
        with open(a.log.replace(".csv", "_plan.json" if rnd == 1 else f"_plan{rnd}.json"), "w") as f:
            json.dump(dict(start=start, area="room" if a.room else dict(length=a.length, width=a.width, side=a.side),
                           lanes=lanes, waypoints=wps), f, indent=1)
        if a.dry:
            break

        nav.followWaypoints([pose(nav, x, y, yw) for x, y, yw in wps])
        last, prev = -1, None
        try:
            while not nav.isTaskComplete():
                try:
                    x, y, yw = robot_pose(nav, buf)
                    t = time.time() - t0
                    out.write(f"{t:.2f},{x:.3f},{y:.3f},{yw:.3f}\n")
                    # the floor counts as seen only while driving, as on the inspection page: frames
                    # taken while turning on the spot or creeping are left out there
                    if prev and t > prev[0]:
                        v = math.hypot(x - prev[1], y - prev[2]) / (t - prev[0])
                        w = abs((yw - prev[3] + math.pi) % (2 * math.pi) - math.pi) / (t - prev[0])
                        if v >= 0.05 and w <= 0.15 + v:
                            camera.append((x + lp.CAMERA_AHEAD * math.cos(yw), y + lp.CAMERA_AHEAD * math.sin(yw)))
                    prev = (t, x, y, yw)
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
            break
        res = nav.getResult()
        log.info(f"pass {rnd} finished: {'SUCCEEDED' if res == TaskResult.SUCCEEDED else res} after {time.time() - t0:.0f} s")
        t1 = time.time()
        while time.time() - t1 < 3.0:      # the map with what the last pass saw
            rclpy.spin_once(nav, timeout_sec=0.2)
    if out:
        log.info(f"sweep finished after {time.time() - t0:.0f} s")
        out.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
