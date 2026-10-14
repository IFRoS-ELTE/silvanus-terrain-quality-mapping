#!/usr/bin/env python3
"""Render an outdoor run as a video: camera with detections, height map, surface roughness and path.

    render_outdoor.py BAG OUT_PREFIX

Top: the colour image with the detected bumps and holes, and the height above the
fitted ground (ground_probe, robot mask on). Bottom: the path so far (wheel speed +
Xsens heading, coloured by the logged surface), the two roughness numbers of
docs/PROGRESS.md over time, and their current values:
  camera unevenness  RMS height of the pixels within 15 mm of the ground plane
  IMU fast/slow      power of vertical free acceleration at 8-50 Hz over 0.5-8 Hz,
                     last 1.8 s; only comparable at 0.25-0.50 m/s, grey otherwise
Surfaces are the labels of docs/DATA.md. Writes OUT_PREFIX.mp4 at real time.
"""
import os, sys, subprocess
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ground_demo"))
import ground_probe as gp
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

# docs/DATA.md, seconds from the start of the bag
SURFACES = {
    "garden_surfaces_1548": [("asphalt", 0, 10), ("grass/dirt", 12, 55), ("cracked asphalt", 57, 72), ("concrete", 72, 75), ("asphalt", 76, 87)],
    "garden_surfaces_1618": [("cracked asphalt", 0, 36), ("grass/dirt", 38, 60), ("red rubber path", 62, 75), ("wood chips", 77, 89)],
    "garden_surfaces_1628": [("red rubber path", 0, 21), ("kerb", 22, 26), ("wood chips", 27, 45), ("grass/dirt", 46, 65), ("asphalt", 67, 89)],
}
COLOURS = {"asphalt": (200, 150, 90), "cracked asphalt": (230, 200, 150), "concrete": (200, 200, 200), "grass/dirt": (70, 170, 70),
           "red rubber path": (70, 70, 210), "wood chips": (40, 120, 180), "kerb": (0, 220, 255), None: (90, 90, 90)}
REFERENCE = [("asphalt", 3.9, 2.2), ("red rubber path", 3.6, 0.51), ("wood chips", 7.5, 0.41), ("grass/dirt", 6.8, 0.29)]  # PROGRESS.md

DEPTH, DINFO = "/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"
COLOR, CINFO = "/camera/camera/color/image_raw", "/camera/camera/color/camera_info"
DEPTH_TO_COLOR_X, SPAN, THRESH, GRIP_ROW = 0.015, 0.04, 0.015, 340
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE, GREY, BLACK, PANEL = (255, 255, 255), (170, 170, 170), (0, 0, 0), (25, 26, 26)

bag, out = sys.argv[1].rstrip("/"), sys.argv[2]
name = os.path.basename(bag)
segs = SURFACES.get(name, [])
mask_ok = cv2.imread(os.path.join(os.path.dirname(gp.__file__), "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0


def surface(t):
    return next((s for s, a, b in segs if a <= t <= b), None)


def open_bag(topics):
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=topics))
    return r


def label(img, text, org, scale, col, thick=2):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thick)
    x, y = max(0, min(org[0], img.shape[1] - w - 4)), max(h + 4, min(org[1], img.shape[0] - base - 2))
    cv2.rectangle(img, (x - 3, y - h - 4), (x + w + 3, y + base + 2), BLACK, -1)
    cv2.putText(img, text, (x, y), FONT, scale, col, thick, cv2.LINE_AA)


# ---- pass 1: odometry, IMU heading and free acceleration -> path and IMU ratio
t_start = open_bag([]).get_metadata().starting_time.nanoseconds * 1e-9
O, Y, F = [], [], []
r = open_bag(["/odom", "/imu/data", "/filter/free_acceleration"])
while r.has_next():
    tp, data, st = r.read_next(); t = st * 1e-9 - t_start
    if tp == "/odom":
        m = deserialize_message(data, Odometry); O.append((t, m.twist.twist.linear.x))
    elif tp == "/imu/data":
        q = deserialize_message(data, Imu).orientation
        Y.append((t, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    else:
        F.append((t, deserialize_message(data, Vector3Stamped).vector.z))
O, Y, F = np.array(O), np.array(Y), np.array(F)
yaw = np.interp(O[:, 0], Y[:, 0], np.unwrap(Y[:, 1]) - Y[0, 1])
dt = np.diff(O[:, 0], prepend=O[0, 0])
px, py = np.cumsum(O[:, 1] * dt * np.cos(yaw)), np.cumsum(O[:, 1] * dt * np.sin(yaw))

N, FS = 180, 100.0
fr = np.fft.rfftfreq(N, 1 / FS); W = np.hanning(N)
imu_t, imu_r, imu_ok = [], [], []
for t in np.arange(2.0, O[-1, 0], 0.25):
    x = F[F[:, 0] <= t, 1][-N:]
    if len(x) < N:
        continue
    P = np.abs(np.fft.rfft((x - x.mean()) * W)) ** 2
    v = np.mean(np.abs(O[(O[:, 0] > t - 1.8) & (O[:, 0] <= t), 1]))
    imu_t.append(t); imu_r.append(P[(fr >= 8) & (fr < 50)].sum() / P[(fr >= 0.5) & (fr < 8)].sum()); imu_ok.append(0.25 <= v <= 0.50)
imu_t, imu_r, imu_ok = np.array(imu_t), np.array(imu_r), np.array(imu_ok)


# ---- drawing of the bottom panel
def draw_map(t, size=330):
    img = np.full((size, size, 3), PANEL, np.uint8)
    sx, sy = -py, px                       # screen right = robot's right at start, up = forward at start
    lo, hi = np.array([sx.min(), sy.min()]), np.array([sx.max(), sy.max()])
    sc = (size - 60) / max((hi - lo).max(), 1.0)
    c = (lo + hi) / 2
    P = np.stack([size / 2 + (sx - c[0]) * sc, size / 2 + 10 - (sy - c[1]) * sc], 1).astype(np.int32)
    cv2.polylines(img, [P], False, (70, 70, 70), 2, cv2.LINE_AA)
    k = np.searchsorted(O[:, 0], t)
    for i in range(1, k, 2):
        cv2.line(img, tuple(P[i - 1]), tuple(P[min(i + 1, k - 1)]), COLOURS[surface(O[i, 0])], 4, cv2.LINE_AA)
    if k:
        p, a = P[k - 1], yaw[k - 1]
        tip = (int(p[0] - 14 * np.sin(a)), int(p[1] - 14 * np.cos(a)))
        cv2.circle(img, tuple(p), 7, WHITE, -1); cv2.line(img, tuple(p), tip, WHITE, 3, cv2.LINE_AA)
    bar = int(5 * sc)
    cv2.line(img, (12, size - 14), (12 + bar, size - 14), WHITE, 2)
    cv2.putText(img, "5 m", (16 + bar, size - 9), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
    cv2.putText(img, "path: wheel speed + IMU heading", (8, 20), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
    return img


def draw_charts(t, ut, uv, w, h=330):
    img = np.full((h, w, 3), PANEL, np.uint8)
    T1 = O[-1, 0]; X = lambda tt: int(50 + (tt / T1) * (w - 60))
    rows = [("camera unevenness (mm)", 30, 160, 0.0, 12.0, False), ("IMU fast/slow shaking  (grey: speed outside 0.25-0.50 m/s)", 190, 305, np.log10(0.1), np.log10(5.0), True)]
    for title, y0, y1, vmin, vmax, logy in rows:
        for s, a, b in segs:
            cv2.rectangle(img, (X(a), y0), (X(b), y1), tuple(int(c * 0.45) for c in COLOURS[s]), -1)
        cv2.rectangle(img, (X(0), y0), (X(T1), y1), (80, 80, 80), 1)
        cv2.putText(img, title, (X(0), y0 - 8), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
        Yp = lambda v: int(y1 - (np.clip(v, vmin, vmax) - vmin) / (vmax - vmin) * (y1 - y0))
        ticks = [0.1, 0.3, 1, 3] if logy else [0, 4, 8, 12]
        for tk in ticks:
            v = np.log10(tk) if logy else tk
            cv2.putText(img, f"{tk:g}", (8, Yp(v) + 5), FONT, 0.45, GREY, 1, cv2.LINE_AA)
        if logy:
            m = imu_t <= t
            for tt, rr, ok in zip(imu_t[m], imu_r[m], imu_ok[m]):
                cv2.circle(img, (X(tt), Yp(np.log10(rr))), 2, WHITE if ok else (100, 100, 100), -1)
        elif len(ut):
            pts = np.stack([[X(a) for a in ut], [Yp(b) for b in uv]], 1).astype(np.int32)
            cv2.polylines(img, [pts], False, WHITE, 1, cv2.LINE_AA)
    for s, a, b in segs:
        if cv2.getTextSize(s, FONT, 0.45, 1)[0][0] + 8 < X(b) - X(a):
            cv2.putText(img, s, (X(a) + 4, 322), FONT, 0.45, COLOURS[s], 1, cv2.LINE_AA)
    cv2.line(img, (X(t), 30), (X(t), 305), WHITE, 1)
    return img


def draw_values(t, unev, w, h=330):
    img = np.full((h, w, 3), PANEL, np.uint8)
    s = surface(t)
    cv2.putText(img, "surface (log):", (14, 34), FONT, 0.6, GREY, 1, cv2.LINE_AA)
    cv2.putText(img, s or "-", (170, 34), FONT, 0.8, COLOURS[s], 2, cv2.LINE_AA)
    k = np.searchsorted(imu_t, t) - 1
    w = (O[:, 0] > t - 1.8) & (O[:, 0] <= t)
    v = np.mean(np.abs(O[w, 1])) if w.any() else 0.0
    ratio = f"{imu_r[k]:.2f}" if k >= 0 else "-"
    rows = [("camera unevenness", f"{unev:.1f} mm" if unev == unev else "-", WHITE),
            ("IMU fast/slow", ratio, WHITE if k >= 0 and imu_ok[k] else (110, 110, 110)),
            ("speed", f"{v:.2f} m/s", WHITE)]
    for i, (a, b, c) in enumerate(rows):
        cv2.putText(img, a, (14, 80 + 36 * i), FONT, 0.6, GREY, 1, cv2.LINE_AA)
        cv2.putText(img, b, (230, 80 + 36 * i), FONT, 0.8, c, 2, cv2.LINE_AA)
    cv2.putText(img, "typical values, all 2026-10-08 runs:", (14, 205), FONT, 0.5, GREY, 1, cv2.LINE_AA)
    cv2.putText(img, "camera     IMU", (230, 230), FONT, 0.5, GREY, 1, cv2.LINE_AA)
    for i, (sn, cu, ir) in enumerate(REFERENCE):
        y = 255 + 22 * i
        cv2.putText(img, sn, (14, y), FONT, 0.5, COLOURS[sn], 1, cv2.LINE_AA)
        cv2.putText(img, f"{cu:.1f} mm    {ir:.2f}", (230, y), FONT, 0.5, WHITE, 1, cv2.LINE_AA)
    return img


# ---- pass 2: depth and colour, one video frame per 2nd depth frame
r = open_bag([DEPTH, DINFO, COLOR, CINFO])
Kd = Kc = prev = col = ff = None
rng, n, ut, uv, unev = np.random.default_rng(0), 0, [], [], float("nan")
while r.has_next():
    topic, data, st = r.read_next()
    t = st * 1e-9 - t_start
    if topic == DINFO and Kd is None:
        m = deserialize_message(data, CameraInfo); Kd = (m.k[0], m.k[4], m.k[2], m.k[5])
    if topic == CINFO and Kc is None:
        m = deserialize_message(data, CameraInfo); Kc = (m.k[0], m.k[4], m.k[2], m.k[5])
    if topic == COLOR:
        col = data
    if topic != DEPTH or Kd is None or Kc is None or col is None:
        continue
    n += 1
    if n % 2:
        continue
    m = deserialize_message(data, Image)
    depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
    cloud = gp.deproject(depth, *Kd)
    valid = (depth > 0.30) & (depth < 1.60) & mask_ok
    hz, resid = [], np.zeros_like(depth)
    pts = cloud[valid]
    fit = None
    if len(pts) >= 500:
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = (gp.refine_plane(sample, *prev) if prev else None) or gp.fit_ground_plane(sample, rng=rng)
    if fit is None:
        prev = None; valid[:] = False; unev = float("nan")
    else:
        nrm, d, _ = fit; prev = (nrm, d)
        resid = ((cloud @ nrm) + d).astype(np.float32); resid[~valid] = 0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool); resid[~valid] = 0
        flat = valid & (np.abs(resid) < THRESH); flat[GRIP_ROW:] = False
        unev = float(np.sqrt(np.mean(resid[flat] ** 2)) * 1000) if flat.any() else float("nan")
        hz = [h for h in gp.find_hazards(cloud, valid, resid, nrm, d, THRESH, 25.0, Kd[0], Kd[1]) if h["peak_mm"] <= 300]
        if unev == unev:
            ut.append(t); uv.append(unev)

    hmap = gp.residual_view(resid, valid, SPAN).copy()
    c = deserialize_message(col, Image)
    cimg = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)[:, :, ::-1].copy()
    for h in hz:
        x, y, w, hh = h["bbox"]
        fg = gp.C_BUMP if h["type"] == "bump" else gp.C_HOLE
        cv2.rectangle(hmap, (x, y), (x + w, y + hh), BLACK, 4); cv2.rectangle(hmap, (x, y), (x + w, y + hh), WHITE, 2)
        label(hmap, f"{h['size_mm']:.0f} mm", (x, y - 8 if y > 30 else y + hh + 26), 0.65, WHITE)
        sgn = 1.0 if h["type"] == "bump" else -1.0
        sel = valid[y:y + hh, x:x + w] & (sgn * resid[y:y + hh, x:x + w] > 0.5 * THRESH)
        p = cloud[y:y + hh, x:x + w][sel]
        if len(p) >= 20:
            u = Kc[0] * (p[:, 0] + DEPTH_TO_COLOR_X) / p[:, 2] + Kc[2]; v = Kc[1] * p[:, 1] / p[:, 2] + Kc[3]
            u0, u1 = np.percentile(u, [2, 98]).astype(int); v0, v1 = np.percentile(v, [2, 98]).astype(int)
            cv2.rectangle(cimg, (u0, v0), (u1, v1), BLACK, 4); cv2.rectangle(cimg, (u0, v0), (u1, v1), WHITE, 2)
            label(cimg, f"{h['type']} {h['size_mm']:.0f} mm", (u0, v0 - 8 if v0 > 30 else v1 + 26), 0.6, WHITE)

    width = cimg.shape[1] + hmap.shape[1]
    head = np.full((44, width, 3), PANEL, np.uint8)
    cv2.putText(head, f"Robot camera   t = {t:4.1f} s", (12, 30), FONT, 0.75, WHITE, 2, cv2.LINE_AA)
    cv2.putText(head, name, (390, 30), FONT, 0.55, GREY, 1, cv2.LINE_AA)
    cv2.putText(head, "Height above the fitted ground  (+-40 mm, red up, blue down)",
                (cimg.shape[1] + 12, 30), FONT, 0.75, WHITE, 2, cv2.LINE_AA)
    mp = draw_map(t)
    vals = draw_values(t, unev, 470)
    charts = draw_charts(t, ut, uv, width - mp.shape[1] - vals.shape[1])
    frame = np.vstack([head, np.hstack([cimg, hmap]), np.hstack([mp, charts, vals])])
    if ff is None:
        fh, fw = frame.shape[:2]
        ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{fw}x{fh}",
                               "-r", "15", "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "26",
                               "-pix_fmt", "yuv420p", "-movflags", "+faststart", out + ".mp4"], stdin=subprocess.PIPE)
    ff.stdin.write(frame.tobytes())
ff.stdin.close(); ff.wait()
print(name, n, "depth frames; path length", f"{np.sum(np.abs(O[:, 1]) * dt):.1f} m")
