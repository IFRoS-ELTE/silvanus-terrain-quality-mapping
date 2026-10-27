#!/usr/bin/env python3
"""The inspection page while the robot drives: a live run for tools/inspection_viewer.

Runs on the robot, next to live SLAM and the speed governor, and every 2 s writes into OUT_DIR the
same files as inspection_map.py: ground.jpg / cover.png (the floor stitched from the colour camera),
map.png (the live lidar map), cracks.png (empty: the crack model runs after the run), crops/ and
inspection.json with the route, where the governor slowed or stopped, and the governor's hazards with
their felt / not felt verdicts. Serve OUT_DIR's viewer folder over http and open it on the laptop:
only these files cross the Wi-Fi, never the camera images.

Simpler than the offline map on purpose: the floor is taken as the z = 0 plane of the map frame
(flat ground, camera pose from TF) instead of a plane fitted in every depth frame, and about two
colour frames a second are used. inspection_map.py on the recording stays the accurate version.

    python3 inspection_live.py --out /bags/live_web/runs/live
"""
import argparse, json, math, os, sys, time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

RES = 0.005           # m per pixel, as inspection_map.py
GROW = 1.0            # m added on a side when the floor runs off the canvas
KEEP_ROWS = 400       # colour rows that show floor; below are the gripper fingers (inspection_map.py without the crack mask)
STRIDE = 3            # every 3rd colour pixel: about 4 mm on the floor at 0.85 m, under the 5 mm grid


def quat_matrix(q, t):
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def priority(h):
    """inspection_map.priority for bumps and holes, with the governor's live verdict."""
    height, felt = h["height_mm"], h.get("class") == "C"
    if height >= 50 or (felt and height >= 30):
        return "high"
    if height >= 30 or felt:
        return "medium"
    return "low"


def write_atomic(path, data_or_fn):
    tmp = path + ".tmp" + os.path.splitext(path)[1]
    if callable(data_or_fn):
        data_or_fn(tmp)
    else:
        with open(tmp, "w") as f:
            f.write(data_or_fn)
    os.replace(tmp, path)


class Live(Node):
    def __init__(self, a):
        super().__init__("inspection_live")
        self.a, self.out = a, a.out
        os.makedirs(os.path.join(self.out, "crops"), exist_ok=True)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.K = self.rays = self.uv = self.wgt = None
        self.x0 = self.y1 = None; self.W = self.H = 0
        self.ground = self.best = None
        self.path, self.mode, self.map, self.haz, self.crops = [], None, None, {}, {}
        self.last_frame = 0.0
        self.create_subscription(CameraInfo, a.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, a.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, "/governor/state", lambda m: setattr(self, "mode", json.loads(m.data)["mode"]), 10)
        self.create_subscription(String, "/governor/hazards", self.on_hazards, 10)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.2, self.on_pose)
        self.create_timer(a.period, self.write)
        self.get_logger().info(f"live inspection into {self.out}")

    # ---- inputs
    def on_info(self, m):
        if self.K is not None:
            return
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.size = (m.width, m.height)
        v, u = np.mgrid[0:min(KEEP_ROWS, m.height):STRIDE, 0:m.width:STRIDE]
        self.uv = np.stack([u.ravel(), v.ravel()], 1)
        self.rays = np.stack([(self.uv[:, 0] - self.K[2]) / self.K[0], (self.uv[:, 1] - self.K[3]) / self.K[1],
                              np.ones(len(self.uv))], 1)
        self.wgt = ((self.uv[:, 0] - m.width / 2) ** 2 + (self.uv[:, 1] - m.height / 2) ** 2).astype(np.float32)

    def lookup(self, target, source, stamp=None):
        try:
            return quat_matrix(*self._tr(target, source, stamp))
        except TransformException:
            if stamp is None:
                return None
            try:
                return quat_matrix(*self._tr(target, source, None))
            except TransformException:
                return None

    def _tr(self, target, source, stamp):
        t = self.tf.lookup_transform(target, source, Time.from_msg(stamp) if stamp else Time()).transform
        return t.rotation, t.translation

    def on_pose(self):
        M = self.lookup(self.a.frame, "base_footprint")
        if M is None:
            return
        x, y = M[0, 3], M[1, 3]
        if not self.path or math.hypot(x - self.path[-1][0], y - self.path[-1][1]) > 0.05 or self.mode != self.path[-1][2]:
            self.path.append((x, y, self.mode))

    def on_hazards(self, m):
        d = json.loads(m.data)
        M = self.lookup(self.a.frame, d["frame"])
        if M is None:
            return
        for h in d["hazards"]:
            p = M @ np.array([h["x"], h["y"], 0.0, 1.0])
            h["mx"], h["my"] = float(p[0]), float(p[1])
            self.haz[h["id"]] = h

    def grow(self, xs, ys):
        """Make the canvas cover these points, adding GROW m on a side at a time."""
        if self.x0 is None:
            self.x0, self.x1 = xs.min() - GROW, xs.max() + GROW
            self.y0, self.y1 = ys.min() - GROW, ys.max() + GROW
        else:
            nx0 = min(self.x0, xs.min() - GROW) if xs.min() < self.x0 else self.x0
            nx1 = max(self.x1, xs.max() + GROW) if xs.max() > self.x1 else self.x1
            ny0 = min(self.y0, ys.min() - GROW) if ys.min() < self.y0 else self.y0
            ny1 = max(self.y1, ys.max() + GROW) if ys.max() > self.y1 else self.y1
            if (nx0, nx1, ny0, ny1) == (self.x0, self.x1, self.y0, self.y1):
                return
            self.x0, self.x1, self.y0, self.y1, old = nx0, nx1, ny0, ny1, (self.ground, self.best, self.ox0, self.oy1)
        W, H = int(math.ceil((self.x1 - self.x0) / RES)), int(math.ceil((self.y1 - self.y0) / RES))
        ground, best = np.full((H, W, 3), 238, np.uint8), np.full((H, W), np.inf, np.float32)
        if self.ground is not None:
            c0, r0 = int(round((old[2] - self.x0) / RES)), int(round((self.y1 - old[3]) / RES))
            h, w = old[1].shape
            ground[r0:r0 + h, c0:c0 + w], best[r0:r0 + h, c0:c0 + w] = old[0], old[1]
        self.ground, self.best, self.W, self.H, self.ox0, self.oy1 = ground, best, W, H, self.x0, self.y1

    def on_image(self, m):
        now = time.time()
        if self.rays is None or now - self.last_frame < 1.0 / self.a.rate:
            return
        M = self.lookup(self.a.frame, m.header.frame_id, m.header.stamp)
        if M is None:
            return
        self.last_frame = now
        R, t = M[:3, :3], M[:3, 3]
        d = self.rays @ R.T
        with np.errstate(divide="ignore", invalid="ignore"):
            s = -t[2] / d[:, 2]
        ok = (s > 0.2) & (s < 2.5)
        if not ok.any():
            return
        P = t[:2] + d[ok, :2] * s[ok, None]
        self.grow(P[:, 0], P[:, 1])
        px = ((P[:, 0] - self.x0) / RES).astype(int); py = ((self.y1 - P[:, 1]) / RES).astype(int)
        inb = (px >= 0) & (px < self.W) & (py >= 0) & (py < self.H)
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        rgb = img if m.encoding == "rgb8" else img[:, :, ::-1]
        u, v, w = self.uv[ok][inb, 0], self.uv[ok][inb, 1], self.wgt[ok][inb]
        px, py = px[inb], py[inb]
        better = w < self.best[py, px]
        self.ground[py[better], px[better]] = rgb[v[better], u[better]]
        self.best[py[better], px[better]] = w[better]
        self.photo(rgb, M)

    def photo(self, rgb, M):
        """Keep, per hazard, the frame where it sits nearest the image centre, cut out around it."""
        Minv = np.linalg.inv(M)
        fx, fy, cx, cy = self.K
        for hid, h in self.haz.items():
            p = Minv @ np.array([h["mx"], h["my"], 0.0, 1.0])
            if p[2] < 0.2:
                continue
            u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
            half = (max(h.get("length_mm", 150), 150) / 2 + 120) / 1000 * fx / p[2]
            if not (half <= u <= self.size[0] - half and half <= v <= KEEP_ROWS - half):
                continue
            score = (u - self.size[0] / 2) ** 2 + (v - self.size[1] / 2) ** 2
            if hid in self.crops and self.crops[hid] <= score:
                continue
            crop = rgb[int(v - half):int(v + half), int(u - half):int(u + half), ::-1]
            if min(crop.shape[:2]) < 40:
                continue
            crop = cv2.resize(crop, (360, int(360 * crop.shape[0] / crop.shape[1])), interpolation=cv2.INTER_AREA)
            write_atomic(os.path.join(self.out, "crops", f"{hid}.jpg"), lambda f: cv2.imwrite(f, crop))
            self.crops[hid] = score

    # ---- output
    def map_rgba(self):
        rgba = np.zeros((self.H, self.W, 4), np.uint8)
        if self.map is None:
            return rgba
        info = self.map.info
        g = np.array(self.map.data, np.int16).reshape(info.height, info.width)
        c = ((self.x0 + (np.arange(self.W) + 0.5) * RES - info.origin.position.x) / info.resolution).astype(int)
        r = ((self.y1 - (np.arange(self.H) + 0.5) * RES - info.origin.position.y) / info.resolution).astype(int)
        cin, rin = (c >= 0) & (c < info.width), (r >= 0) & (r < info.height)
        cell = np.full((self.H, self.W), -1, np.int16)
        cell[np.ix_(rin, cin)] = g[np.ix_(r[rin], c[cin])]
        rgba[(cell >= 0) & (cell < 50)] = (246, 244, 240, 255)
        rgba[cell >= 50] = (52, 52, 56, 255)
        return rgba

    def write(self):
        if self.ground is None:
            return
        to_px = lambda x, y: (int((x - self.x0) / RES), int((self.y1 - y) / RES))
        filled = np.isfinite(self.best)
        cover = cv2.dilate(filled.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0     # close the stride gaps
        f = filled.astype(np.float32)        # gap pixels take the mean of their filled neighbours
        avg = cv2.blur(self.ground.astype(np.float32) * f[:, :, None], (3, 3)) / np.maximum(cv2.blur(f, (3, 3)), 1e-3)[:, :, None]
        ground = np.where(filled[:, :, None], self.ground, avg.astype(np.uint8))
        o = self.out
        write_atomic(os.path.join(o, "ground.jpg"), lambda f: cv2.imwrite(f, ground[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85]))
        mask = np.zeros((self.H, self.W, 4), np.uint8); mask[cover, 3] = 255
        write_atomic(os.path.join(o, "cover.png"), lambda f: cv2.imwrite(f, mask))
        write_atomic(os.path.join(o, "map.png"), lambda f: cv2.imwrite(f, cv2.cvtColor(self.map_rgba(), cv2.COLOR_RGBA2BGRA)))
        write_atomic(os.path.join(o, "cracks.png"), lambda f: cv2.imwrite(f, np.zeros((self.H, self.W, 4), np.uint8)))

        P = np.array([(x, y) for x, y, _ in self.path]) if self.path else np.zeros((0, 2))
        s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))] if len(P) > 1 else np.zeros(len(P))
        items = []
        for hid, h in sorted(self.haz.items(), key=lambda kv: int(kv[0][1:])):
            if h.get("part_of"):
                continue                       # one object: shown once, under the piece that was felt
            j = dict(id=hid, type=h["type"], height_mm=h["height_mm"], length_mm=h["length_mm"], width_mm=h["length_mm"],
                     seen=h["seen"], imu_peak=h.get("imu_peak"), crossing_speed=None,
                     **{"class": h.get("class", "A")})
            j["priority"] = priority(j)
            j["px"], j["py"] = to_px(h["mx"], h["my"])
            if len(P):
                i = int(np.argmin(np.hypot(P[:, 0] - h["mx"], P[:, 1] - h["my"])))
                k = min(i + 1, len(P) - 1); a = math.atan2(P[k, 1] - P[max(k - 1, 0), 1], P[k, 0] - P[max(k - 1, 0), 0])
                off = -math.sin(a) * (h["mx"] - P[i, 0]) + math.cos(a) * (h["my"] - P[i, 1])
                j.update(route_m=round(float(s[i]), 1), offset_m=round(off, 2),
                         side="left" if off > 0.15 else "right" if off < -0.15 else "on the path")
            else:
                j.update(route_m=0.0, offset_m=0.0, side="on the path")
            if not h["done"]:
                j["note"] = "not crossed yet: felt or not is decided when the rear wheels are past it"
            j["crop"] = f"crops/{hid}.jpg" if hid in self.crops else None
            items.append(j)
        data = {
            "run": "live", "live": True, "stamp": round(time.time(), 1),
            "length_m": round(float(s[-1]), 1) if len(s) else 0.0, "m_per_px": RES,
            "width_px": self.W, "height_px": self.H, "origin": {"x0": round(self.x0, 3), "y1": round(self.y1, 3)},
            "map": "map.png", "positions": "SLAM (lidar)",
            "path": [[*to_px(x, y), None, None, m] for x, y, m in self.path],
            "crack_length_m": 0.0, "hazards": items,
            "rules": {"class": "A seen by the camera only, C seen and felt by the IMU (live: the governor's verdict)",
                      "priority": "high from 50 mm, or felt and from 30 mm; medium from 30 mm or felt",
                      "rough_ground": "not judged live"},
        }
        write_atomic(os.path.join(o, "inspection.json"), json.dumps(data))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="/bags/live_web/runs/live")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--rate", type=float, default=2.0, help="colour frames per second laid on the floor")
    ap.add_argument("--period", type=float, default=2.0, help="seconds between writes")
    ap.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--info-topic", default="/camera/camera/color/camera_info")
    a, _ = ap.parse_known_args()
    rclpy.init(args=sys.argv)
    node = Live(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.write()


if __name__ == "__main__":
    main()
