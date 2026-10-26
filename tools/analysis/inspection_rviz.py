#!/usr/bin/env python3
"""Show an inspection map (inspection_map.py output) in RViz, in 3D.

    inspection_rviz.py OUT_DIR            then: rviz2 -d tools/analysis/inspection.rviz

Publishes in frame `map`, latched, so RViz can start before or after:
  /inspection/ground   sensor_msgs/PointCloud2  the stitched ground photo as coloured points (2 cm),
                       cracks painted orange
  /inspection/hazards  visualization_msgs/MarkerArray  bumps (red) and holes (blue) as columns with
                       their height drawn 5x, a ring for high priority (red) and check on site
                       (violet), a label with ID and size
  /inspection/route    nav_msgs/Path  the driven route
Run it with ROS_DOMAIN_ID set to something other than the robot's 42 when on the robot network.
"""
import json, os, sys
import numpy as np, cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

STEP = 4              # ground photo pixels per point: 4 x 5 mm = 2 cm
EXAGGERATE = 5.0      # hazard heights drawn 5x so a 20 mm bump is visible next to a 30 m route
COL = {"bump": (0.89, 0.29, 0.28), "hole": (0.16, 0.47, 0.84), "crack": (1.0, 0.59, 0.08)}
RING = {"high": (0.78, 0.21, 0.16), "check": (0.37, 0.31, 0.78)}


class InspectionViz(Node):
    def __init__(self, out_dir):
        super().__init__("inspection_viz")
        self.d = json.load(open(os.path.join(out_dir, "inspection.json")))
        res, x0, y1 = self.d["m_per_px"], self.d["origin"]["x0"], self.d["origin"]["y1"]
        self.world = lambda px, py: (x0 + px * res, y1 - py * res)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_g = self.create_publisher(PointCloud2, "/inspection/ground", qos)
        self.pub_h = self.create_publisher(MarkerArray, "/inspection/hazards", qos)
        self.pub_r = self.create_publisher(Path, "/inspection/route", qos)
        self.msgs = (self.ground(out_dir), self.hazards(), self.route())
        self.publish()
        self.create_timer(2.0, self.publish)
        n = sum(h["priority"] != "low" for h in self.d["hazards"])
        self.get_logger().info(f"{self.d['run']}: {self.msgs[0].width} ground points, {n} hazards (low priority hidden), "
                               f"{self.d['crack_length_m']} m of cracks. Open RViz with tools/analysis/inspection.rviz")

    def publish(self):
        now = self.get_clock().now().to_msg()
        for m in (self.msgs[0], self.msgs[2]):
            m.header.stamp = now
        for m in self.msgs[1].markers:
            m.header.stamp = now
        self.pub_g.publish(self.msgs[0]); self.pub_h.publish(self.msgs[1]); self.pub_r.publish(self.msgs[2])

    def ground(self, out_dir):
        g = cv2.imread(os.path.join(out_dir, "ground.jpg"))[::STEP, ::STEP]
        cover = cv2.imread(os.path.join(out_dir, "cover.png"), cv2.IMREAD_UNCHANGED)[::STEP, ::STEP, 3] > 0
        crack = cv2.imread(os.path.join(out_dir, "cracks.png"), cv2.IMREAD_UNCHANGED)
        crack = cv2.dilate((crack[:, :, 3] > 0).astype(np.uint8), np.ones((STEP, STEP), np.uint8))[::STEP, ::STEP] > 0
        g[crack] = (20, 150, 255)                         # BGR orange
        py, px = np.nonzero(cover)
        x, y = self.world(px * STEP, py * STEP)
        b, gg, r = g[py, px, 0].astype(np.uint32), g[py, px, 1].astype(np.uint32), g[py, px, 2].astype(np.uint32)
        rgb = ((r << 16) | (gg << 8) | b).view(np.float32)
        pts = np.zeros(len(px), dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)])
        pts["x"], pts["y"], pts["z"], pts["rgb"] = x, y, 0.0, rgb
        msg = PointCloud2()
        msg.header.frame_id = "map"; msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(("x", "y", "z", "rgb"))]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(pts), True
        msg.data = pts.tobytes()
        return msg

    def hazards(self):
        arr, i = MarkerArray(), 0
        def mk(kind, ns):
            nonlocal i
            m = Marker(); m.header.frame_id = "map"; m.ns = ns; m.id = i; m.type = kind; m.action = Marker.ADD
            m.pose.orientation.w = 1.0; i += 1; return m
        for h in self.d["hazards"]:
            if h["priority"] == "low" or h["type"] == "crack" and h["priority"] != "high":
                continue
            x, y = self.world(h["px"], h["py"])
            if h["type"] != "crack" and h["class"] != "B":
                hgt = max(h["height_mm"], 5) / 1000.0 * EXAGGERATE
                m = mk(Marker.CYLINDER, "hazards")
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.position.z = hgt / 2 if h["type"] == "bump" else -hgt / 2
                m.scale.x = m.scale.y = float(np.clip(min(h["length_mm"], h["width_mm"] * 2) / 1000.0, 0.06, 0.3))
                m.scale.z = hgt
                m.color.r, m.color.g, m.color.b = COL[h["type"]]; m.color.a = 1.0 if h["class"] == "C" else 0.55
                arr.markers.append(m)
            if h["priority"] in RING:
                m = mk(Marker.CYLINDER, "priority")
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.003
                m.scale.x = m.scale.y = 0.42; m.scale.z = 0.004
                m.color.r, m.color.g, m.color.b = RING[h["priority"]]; m.color.a = 0.45
                arr.markers.append(m)
            t = mk(Marker.TEXT_VIEW_FACING, "labels")
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.45
            size = (f"{h['length_mm'] / 1000:.1f} m crack" if h["type"] == "crack" else
                    "felt, not seen" if h["class"] == "B" else f"{h['height_mm']} mm {h['type']}")
            t.text = f"{h['id']}  {size}"; t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = 1.0; t.color.a = 0.95
            arr.markers.append(t)
        return arr

    def route(self):
        p = Path(); p.header.frame_id = "map"
        for px, py, *_ in self.d["path"]:
            ps = PoseStamped(); ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = self.world(px, py); ps.pose.position.z = 0.02
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p


def main():
    rclpy.init()
    node = InspectionViz(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
