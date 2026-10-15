#!/usr/bin/env python3
"""Outdoor figures for docs/PROGRESS.md, from garden_surfaces_1618 and _1628.

    outdoor_figures.py OUT_DIR

outdoor_surfaces.jpg      one column per surface: colour image and height map of a typical
                          frame (camera unevenness closest to that stretch's median)
outdoor_run_1628.png      the 1628 run: path coloured by surface, camera unevenness and
                          IMU fast/slow ratio over time, with the median of each stretch
Surface labels are those of render_outdoor.py (docs/DATA.md).
"""
import os, sys
import numpy as np, cv2
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ground_demo"))
import ground_probe as gp
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

BAGS = os.path.expanduser("~/bags")
SURFACES = {
    "garden_surfaces_1618": [("cracked asphalt", 0, 36), ("grass/dirt", 38, 60), ("red rubber path", 62, 75), ("wood chips", 77, 89)],
    "garden_surfaces_1628": [("red rubber path", 0, 21), ("kerb", 22, 26), ("wood chips", 27, 45), ("grass/dirt", 46, 65), ("asphalt", 67, 89)],
}
# which run each column of the surfaces figure comes from; values are the all-run medians of PROGRESS.md
COLUMNS = [("asphalt", "garden_surfaces_1628", 3.9, 2.2), ("cracked asphalt", "garden_surfaces_1618", 3.9, 2.2),
           ("red rubber path", "garden_surfaces_1628", 3.6, 0.51), ("wood chips", "garden_surfaces_1628", 7.5, 0.41),
           ("grass/dirt", "garden_surfaces_1628", 6.8, 0.29)]
RGB = {"asphalt": "#5a96c8", "cracked asphalt": "#96c8e6", "red rubber path": "#d24646", "wood chips": "#b47828",
       "grass/dirt": "#46aa46", "kerb": "#ffdc00"}
THRESH, GRIP_ROW, SPAN = 0.015, 340, 0.04
out_dir = sys.argv[1]
mask_ok = cv2.imread(os.path.join(os.path.dirname(gp.__file__), "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0


def surface(segs, t, margin=0.0):
    return next((s for s, a, b in segs if a + margin <= t <= b - margin), None)


def depth_pass(name, want):
    """Camera unevenness per 2nd depth frame; JPEGs of the frames inside the wanted surfaces."""
    segs = SURFACES[name]
    r = SequentialReader(); r.open(StorageOptions(uri=f"{BAGS}/{name}", storage_id="mcap"), ConverterOptions("", ""))
    t_start = r.get_metadata().starting_time.nanoseconds * 1e-9
    r.set_filter(StorageFilter(topics=["/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info",
                                       "/camera/camera/color/image_raw"]))
    K = prev = col = None; rng, n, U, frames = np.random.default_rng(0), 0, [], {}
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp.endswith("depth/camera_info"):
            if K is None: m = deserialize_message(data, CameraInfo); K = (m.k[0], m.k[4], m.k[2], m.k[5])
            continue
        if tp.endswith("color/image_raw"):
            col = data; continue
        if K is None or col is None: continue
        n += 1
        if n % 2: continue
        m = deserialize_message(data, Image)
        depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
        cloud = gp.deproject(depth, *K); valid = (depth > 0.30) & (depth < 1.60) & mask_ok; pts = cloud[valid]
        if len(pts) < 500: continue
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = (gp.refine_plane(sample, *prev) if prev else None) or gp.fit_ground_plane(sample, rng=rng)
        if fit is None: prev = None; continue
        nrm, d, _ = fit; prev = (nrm, d)
        resid = ((cloud @ nrm) + d).astype(np.float32); resid[~valid] = 0; resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool); resid[~valid] = 0
        flat = valid & (np.abs(resid) < THRESH); flat[GRIP_ROW:] = False
        if not flat.any(): continue
        u = float(np.sqrt(np.mean(resid[flat] ** 2)) * 1000); U.append((t, u))
        s = surface(segs, t, margin=2.0)
        if s in want:
            c = deserialize_message(col, Image)
            cimg = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)[:, :, ::-1]
            enc = lambda im: cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 92])[1]
            frames.setdefault(s, []).append((u, t, enc(cimg), enc(gp.residual_view(resid, valid, SPAN))))
    return np.array(U), frames


def imu_pass(name):
    """Path from wheel speed + Xsens heading, and the IMU fast/slow ratio every 0.25 s."""
    r = SequentialReader(); r.open(StorageOptions(uri=f"{BAGS}/{name}", storage_id="mcap"), ConverterOptions("", ""))
    t_start = r.get_metadata().starting_time.nanoseconds * 1e-9
    r.set_filter(StorageFilter(topics=["/odom", "/imu/data", "/filter/free_acceleration"]))
    O, Y, F = [], [], []
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == "/odom":
            O.append((t, deserialize_message(data, Odometry).twist.twist.linear.x))
        elif tp == "/imu/data":
            q = deserialize_message(data, Imu).orientation
            Y.append((t, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        else:
            F.append((t, deserialize_message(data, Vector3Stamped).vector.z))
    O, Y, F = np.array(O), np.array(Y), np.array(F)
    yaw = np.interp(O[:, 0], Y[:, 0], np.unwrap(Y[:, 1]) - Y[0, 1]); dt = np.diff(O[:, 0], prepend=O[0, 0])
    px, py = np.cumsum(O[:, 1] * dt * np.cos(yaw)), np.cumsum(O[:, 1] * dt * np.sin(yaw))
    N, FS = 180, 100.0; fr = np.fft.rfftfreq(N, 1 / FS); W = np.hanning(N); R = []
    for t in np.arange(2.0, O[-1, 0], 0.25):
        x = F[F[:, 0] <= t, 1][-N:]
        if len(x) < N: continue
        P = np.abs(np.fft.rfft((x - x.mean()) * W)) ** 2
        v = np.mean(np.abs(O[(O[:, 0] > t - 1.8) & (O[:, 0] <= t), 1]))
        R.append((t, P[(fr >= 8) & (fr < 50)].sum() / P[(fr >= 0.5) & (fr < 8)].sum(), 0.25 <= v <= 0.50))
    return O, px, py, np.array(R), float(np.sum(np.abs(O[:, 1]) * dt))


# ---- depth passes
data = {}
for name in SURFACES:
    want = {c[0] for c in COLUMNS if c[1] == name}
    data[name] = depth_pass(name, want)
    print(name, len(data[name][0]), "depth frames")

# ---- figure 1: one column per surface, typical frame
cols = []
for surf, name, cu, ir in COLUMNS:
    C = data[name][1][surf]
    med = float(np.median([c[0] for c in C]))
    u, t, cj, hj = min(C, key=lambda c: abs(c[0] - med))
    cimg = cv2.resize(cv2.imdecode(cj, cv2.IMREAD_COLOR), (320, 240), interpolation=cv2.INTER_AREA)
    hmap = cv2.imdecode(hj, cv2.IMREAD_COLOR)[:GRIP_ROW]          # the gripper rows are masked anyway
    hmap = cv2.resize(hmap, (320, int(320 * hmap.shape[0] / hmap.shape[1])), interpolation=cv2.INTER_AREA)
    text = np.full((96, 320, 3), 255, np.uint8)
    cv2.putText(text, surf, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(text, f"camera unevenness {cu} mm", (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(text, f"IMU fast/slow shaking {ir}", (8, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    gap = np.full((6, 320, 3), 255, np.uint8)
    cols.append(np.vstack([cimg, gap, hmap, text]))
    cols.append(np.full((cols[-1].shape[0], 6, 3), 255, np.uint8))
fig1 = np.hstack(cols[:-1])
key = np.full((40, fig1.shape[1], 3), 255, np.uint8)
ramp = cv2.resize(gp.LUT[np.linspace(0, 255, 256).astype(np.uint8)][None], (260, 14), interpolation=cv2.INTER_NEAREST)
key[12:26, 480:740] = ramp
for txt, x in (("Middle row: height above the fitted ground", 8), ("-40 mm", 410), ("+40 mm", 750), ("black: no data", 840)):
    cv2.putText(key, txt, (x, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
cv2.imwrite(os.path.join(out_dir, "outdoor_surfaces.jpg"), np.vstack([fig1, key]), [cv2.IMWRITE_JPEG_QUALITY, 90])

# ---- figure 2: the 1628 run
name = "garden_surfaces_1628"; segs = SURFACES[name]
U = data[name][0]
O, px, py, R, length = imu_pass(name)
fig = plt.figure(figsize=(14, 5.6))
gs = fig.add_gridspec(2, 2, width_ratios=[1, 2.4], hspace=0.12, wspace=0.18)
ax = fig.add_subplot(gs[:, 0])
for i in range(1, len(O)):
    s_ = surface(segs, O[i, 0])
    ax.plot([-py[i - 1], -py[i]], [px[i - 1], px[i]], color=RGB.get(s_, "#999999"), lw=4 if s_ else 1.5, solid_capstyle="round")
ax.plot(-py[0], px[0], "ko", ms=7); ax.annotate("start", (-py[0], px[0]), xytext=(6, -12), textcoords="offset points")
ax.plot(-py[-1], px[-1], "k^", ms=8); ax.annotate("end", (-py[-1], px[-1]), xytext=(6, 0), textcoords="offset points")
ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("m"); ax.set_ylabel("m")
ax.set_title(f"path from wheel speed + IMU heading\n{length:.1f} m, driven by hand", fontsize=10)
a1, a2 = fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
for a in (a1, a2):
    for s_, ta, tb in segs:
        a.axvspan(ta, tb, color=RGB[s_], alpha=0.18, lw=0)
    a.set_xlim(0, O[-1, 0]); a.grid(alpha=0.3)
a1.plot(U[:, 0], U[:, 1], color="k", lw=0.8)
a1.set_ylabel("camera\nunevenness, mm"); a1.set_ylim(0, 12); a1.tick_params(labelbottom=False)
ok = R[:, 2] > 0
a2.semilogy(R[~ok, 0], R[~ok, 1], ".", color="#bbbbbb", ms=4, label="speed outside 0.25-0.50 m/s")
a2.semilogy(R[ok, 0], R[ok, 1], ".", color="k", ms=5, label="speed 0.25-0.50 m/s")
a2.set_ylabel("IMU fast/slow\nshaking"); a2.set_xlabel("time from start of run, s"); a2.set_ylim(0.08, 8)
a2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, fontsize=8, frameon=False)
for s_, ta, tb in segs:
    mu = U[(U[:, 0] >= ta) & (U[:, 0] <= tb), 1]
    mr = R[(R[:, 0] >= ta) & (R[:, 0] <= tb) & ok, 1]
    if len(mu):
        a1.hlines(np.median(mu), ta, tb, color=RGB[s_], lw=3)
        a1.text((ta + tb) / 2, 11.2, f"{s_}\n{np.median(mu):.1f} mm", ha="center", va="top", fontsize=8)
    if len(mr) >= 3:
        a2.hlines(np.median(mr), ta, tb, color=RGB[s_], lw=3)
        a2.text((ta + tb) / 2, 6.5, f"{np.median(mr):.2f}", ha="center", va="top", fontsize=8)
fig.suptitle(f"{name}: one outdoor run. Thick lines = median of each stretch", fontsize=11)
fig.savefig(os.path.join(out_dir, "outdoor_run_1628.png"), dpi=110, bbox_inches="tight")
print("medians per stretch:", [(s_, round(float(np.median(U[(U[:, 0] >= ta) & (U[:, 0] <= tb), 1])), 1)) for s_, ta, tb in segs])
