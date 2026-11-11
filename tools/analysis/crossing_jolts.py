"""Bump height from the camera against the IMU jolt, per crossing of a tape-measured object.

    python tools/analysis/crossing_jolts.py --bags D:/bags --out docs/results/crossing_jolts

Pure Python (rosbags), no ROS. For the indoor crossing runs of 2026-10-08:

  height    the depth measurement of eval_groundtruth.py / score_groundtruth.py (every 3rd
            depth frame, robot mask, 15 mm threshold, median over the grown outline),
            median over the frames where the object is fully in view. The object and its
            time window come from groundtruth_objects.py
  position  the object's centroid put on the wheel speed + IMU heading path (hazard_map.py)
  crossing  the moments the front and the rear wheels pass over that position
  jolt      the largest |vertical free acceleration - its 0.5 s running median| within
            +-0.15 m of travel of each axle passing, and the shaking 1-5 s around it (hazard_map.jolt_at);
            "felt" uses the governor's rule, calibrated at 0.2 m/s (hazard_map.felt)

crossing_fast_1703 has no camera after 32 s, so its return pass has no heights of its own.
Those crossings are matched to the positions measured on the way out and marked as such.

Writes crossings.csv, crossing_jolts.png and summary.md to --out.
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np
from scipy import ndimage

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ground_demo"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ground_probe as gp  # noqa: E402
from groundtruth_objects import T as TAPE, TR  # noqa: E402

DEPTH, DINFO = "/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"
BAGS = ["crossing_slow_1652", "crossing_fast_1703"]
AXLES, WHEEL_Y = {"front": 0.249, "rear": -0.249}, 0.2915   # scout_v2.xacro: wheels at x = +-0.249, y = +-0.2915
TYRE_HALF = 0.06
DRIVE_V, DRIVE_W = 0.10, 0.30
RIGID = {"wood block", "long plank", "book"}
MASK_OK = cv2.imread(str(ROOT / "ground_demo" / "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0


def tf_matrix(tr):
    q, p = tr.rotation, tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [p.x, p.y, p.z]
    return M


def chain(edges, parent, child):
    """parent <- child through the tree of the first transforms in the bag. The arm does not move during a run."""
    M = np.eye(4)
    while child != parent:
        up, T = edges[child]
        M = T @ M
        child = up
    return M


def read_motion(reader, t0):
    """Transforms, path from wheel speed + IMU heading, and the jolt signal, like hazard_map.motion()."""
    edges, O, Y, F, n_tf = {}, [], [], [], 0
    want = {"/tf", "/tf_static", "/odom", "/imu/data", "/filter/free_acceleration"}
    for c, st, raw in reader.messages(connections=[c for c in reader.connections if c.topic in want]):
        t = st * 1e-9 - t0
        if c.topic in ("/tf", "/tf_static"):
            n_tf += 1
            if n_tf <= 400:
                for tr in reader.deserialize(raw, c.msgtype).transforms:
                    edges.setdefault(tr.child_frame_id, (tr.header.frame_id, tf_matrix(tr.transform)))
            continue
        m = reader.deserialize(raw, c.msgtype)
        if c.topic == "/odom":
            O.append((t, m.twist.twist.linear.x))
        elif c.topic == "/imu/data":
            q = m.orientation
            Y.append((t, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        else:
            F.append((t, m.vector.z))
    O, Y, F = np.array(O), np.array(Y), np.array(F)
    yaw = np.interp(O[:, 0], Y[:, 0], np.unwrap(Y[:, 1]) - Y[0, 1])
    dt = np.diff(O[:, 0], prepend=O[0, 0])
    w = ndimage.uniform_filter1d(np.gradient(yaw, O[:, 0] + 1e-9 * np.arange(len(O))), 15)
    path = dict(t=O[:, 0], v=O[:, 1], w=w, yaw=yaw,
                x=np.cumsum(O[:, 1] * dt * np.cos(yaw)), y=np.cumsum(O[:, 1] * dt * np.sin(yaw)))
    jolt = np.abs(F[:, 1] - ndimage.median_filter(F[:, 1], size=51, mode="nearest"))
    T_bd = chain(edges, "base_footprint", "camera_depth_optical_frame")
    return path, np.stack([F[:, 0], jolt], 1), T_bd


def detections(reader, t0):
    """The bumps of eval_groundtruth.py, every 3rd depth frame, with the keys groundtruth_objects.py selects on."""
    K, prev, rng, n, out = None, None, np.random.default_rng(0), 0, []
    conns = [c for c in reader.connections if c.topic in (DEPTH, DINFO)]
    for c, st, raw in reader.messages(connections=conns):
        if c.topic == DINFO:
            if K is None:
                k = reader.deserialize(raw, c.msgtype).k
                K = (k[0], k[4], k[2], k[5])
            continue
        if K is None:
            continue
        n += 1
        if n % 3:
            continue
        m = reader.deserialize(raw, c.msgtype)
        depth = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
        cloud = gp.deproject(depth, *K)
        valid = (depth > 0.30) & (depth < 1.60) & MASK_OK
        pts = cloud[valid]
        if len(pts) < 500:
            continue
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = (gp.refine_plane(sample, *prev) if prev else None) or gp.fit_ground_plane(sample, rng=rng)
        if fit is None:
            prev = None
            continue
        nrm, d, _ = fit
        prev = (nrm, d)
        resid = ((cloud @ nrm) + d).astype(np.float32)
        resid[~valid] = 0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        resid[~valid] = 0
        for h in gp.find_hazards(cloud, valid, resid, nrm, d, 0.015, 25.0, K[0], K[1]):
            if h["type"] != "bump" or h["peak_mm"] > 300:
                continue
            x, y, w, hh = h["bbox"]
            out.append(dict(t=st * 1e-9 - t0, cx=x + w / 2, y=y, h=hh, L_old=h["length_mm"], W_old=h["width_mm"],
                            old_p90=h["peak_mm"], g_med=h["size_mm"], centroid=np.array(h["centroid"])))
    return out


def at(path, key, t):
    return np.interp(t, path["t"], path[key])


def to_world(path, t, p):
    x, y, a = at(path, "x", t), at(path, "y", t), at(path, "yaw", t)
    return x + np.cos(a) * p[0] - np.sin(a) * p[1], y + np.sin(a) * p[0] + np.cos(a) * p[1]


def robot_frame(path, xy, ts):
    """Where the object is in the robot's frame over time: metres ahead and to the left of base_footprint."""
    dx, dy, a = xy[0] - at(path, "x", ts), xy[1] - at(path, "y", ts), at(path, "yaw", ts)
    return np.cos(a) * dx + np.sin(a) * dy, -np.sin(a) * dx + np.cos(a) * dy


def jolt_at(J, t, speed):
    """hazard_map.jolt_at: the largest jolt near t, and the 75th/95th percentile 1-5 s away.
    The window is +-0.15 m of travel (0.2-0.6 s), so at 0.6-0.9 m/s it does not reach the next object."""
    half = float(np.clip(0.15 / max(speed, 1e-3), 0.2, 0.6))
    w = (J[:, 0] >= t - half) & (J[:, 0] <= t + half)
    around = (np.abs(J[:, 0] - t) > 1.0) & (np.abs(J[:, 0] - t) < 5.0)
    if not w.any() or around.sum() < 50:
        return None, None
    return float(J[w, 1].max()), np.percentile(J[around, 1], [75, 95])


def felt(peak, base):
    """hazard_map.felt, calibrated on corridor_groundtruth_1447 at 0.2 m/s."""
    return peak is not None and peak > max(2.0 * base[0], 1.3 * base[1], 0.75)


def axle_pass(path, J, xy, half_len, axle_x, t_from, t_to):
    """The moment the axle passes the object, which of its wheels run over it, and the jolt then.

    A wheel runs over it when the object reaches across the wheel's track, taking the
    measured length as lying across the path (the strips and the plank do)."""
    ts = path["t"][(path["t"] >= t_from) & (path["t"] <= t_to)]
    if len(ts) < 2:
        return None
    ahead, left = robot_frame(path, xy, ts)
    d = ahead - axle_x
    i = np.flatnonzero((d[:-1] > 0) & (d[1:] <= 0))
    if not len(i):
        return None
    i = i[0]
    tc = float(ts[i] + (ts[i + 1] - ts[i]) * d[i] / (d[i] - d[i + 1]))
    lat = float(np.interp(tc, ts, left))
    wheels = [side for side, wy in (("left", WHEEL_Y), ("right", -WHEEL_Y)) if abs(lat - wy) < half_len + TYRE_HALF]
    speed = float(abs(at(path, "v", tc)))
    driving = speed >= DRIVE_V and abs(at(path, "w", tc)) <= DRIVE_W
    peak, base = jolt_at(J, tc, speed)
    return dict(t=tc, lateral=lat, wheels=wheels, speed=speed, driving=driving, jolt=peak,
                shaking=None if base is None else float(base[1]), felt=driving and felt(peak, base))


def crossing_row(path, J, xy, length_mm, t_from, t_to):
    row = {}
    passes = {axle: axle_pass(path, J, xy, length_mm / 2000.0, x, t_from, t_to) for axle, x in AXLES.items()}
    for axle, a in passes.items():
        a = a or {}
        row[f"{axle}_t"] = a.get("t")
        row[f"{axle}_wheels"] = "+".join(a.get("wheels", [])) or ("none" if a else "")
        row[f"{axle}_jolt"] = a.get("jolt")
        row[f"{axle}_shaking_p95"] = a.get("shaking")
        row[f"{axle}_felt"] = a.get("felt", False)
    done = [a for a in passes.values() if a and a["driving"] and a["jolt"] is not None]
    row["lateral_m"] = next((a["lateral"] for a in passes.values() if a), None)
    row["speed"] = float(np.mean([a["speed"] for a in done])) if done else None
    row["on_wheel"] = any(a["wheels"] for a in done)
    # the jolt of the axle that ran over it; with no wheel on it, the jolt as the robot straddled it
    on = [a["jolt"] for a in done if a["wheels"]] or [a["jolt"] for a in done]
    row["jolt"] = max(on) if on else None
    row["felt"] = any(a["felt"] for a in done if a["wheels"])
    return row


def analyse(bag_dir):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    name = Path(bag_dir).name
    with AnyReader([Path(bag_dir)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as r:
        t0 = r.start_time * 1e-9
        path, J, T_bd = read_motion(r, t0)
        dets = detections(r, t0)
    t_cam_end = max(d["t"] for d in dets) if dets else 0.0

    rows, seen = [], []
    for obj, ta, tb, sel in TR[name]:
        G = [d for d in dets if ta <= d["t"] <= tb and sel(d)]
        if not G:
            print(f"{name}: {obj} at {ta}-{tb} s not found")
            continue
        full = [d for d in G if d["y"] > 3 and d["y"] + d["h"] < 340] or G
        height = float(np.median([d["g_med"] for d in full]))
        length = float(np.median([d["L_old"] for d in full]))
        pos = np.median([to_world(path, d["t"], (T_bd @ np.r_[d["centroid"], 1.0])[:2]) for d in full], axis=0)
        # the rear axle passes a few seconds after the camera saw it
        rows.append(dict(bag=name, object=obj, tape_mm=TAPE[obj][2], camera_mm=height, length_mm=length,
                         frames=len(full), seen_t=ta, height_from="this pass",
                         **crossing_row(path, J, pos, length, ta, ta + 12.0)))
        seen.append((obj, ta, pos, length, height))

    # crossing_fast_1703: no camera on the way back, so use the positions measured on the way out
    turned = np.flatnonzero(np.abs(path["yaw"] - path["yaw"][0]) > np.radians(150))
    if t_cam_end < path["t"][-1] - 5 and len(turned):
        t_back = path["t"][turned[0]]
        for obj, ta, pos, length, height in seen:
            if ta > t_back:
                continue
            c = crossing_row(path, J, pos, length, t_back, path["t"][-1])
            if c["front_t"] is None and c["rear_t"] is None:
                continue
            rows.append(dict(bag=name, object=obj, tape_mm=TAPE[obj][2], camera_mm=height, length_mm=length, frames=0,
                             seen_t=ta, height_from="outward pass", **c))
    return rows


def plot(rows, path):
    fig, ax = plt.subplots(figsize=(7.6, 4.8), dpi=130)
    colours = {"crossing_slow_1652": "#2a78d6", "crossing_fast_1703": "#eb6834"}
    for r in rows:
        if r["jolt"] is None or r["height_from"] != "this pass":   # the return of 1703 is placed too loosely
            continue
        c = colours[r["bag"]]
        if not r["on_wheel"]:      # straddled: no wheel touched it
            ax.scatter(r["camera_mm"], r["jolt"], s=40, marker="x", color="#9a9a96", linewidth=1.3, zorder=2)
            continue
        rigid = r["object"] in RIGID
        ax.scatter(r["camera_mm"], r["jolt"], s=48, marker="o" if rigid else "s",
                   facecolor=c if rigid else "none", edgecolor=c, linewidth=1.5, zorder=3)
    ax.axhline(0.75, color="#8a8a86", lw=1, ls="--", zorder=1)
    ax.text(51.5, 0.7, "the felt test needs at least 0.75 m/s²", color="#6b6b67", fontsize=8.5, va="top", ha="right")
    ax.set_xlim(12, 52)
    ax.set_ylim(0, None)
    ax.set_xlabel("Bump height measured by the camera (mm)")
    ax.set_ylabel("IMU jolt at the crossing (m/s²)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color="#e6e6e3", lw=0.8)
    speed = {b: np.median([r["speed"] for r in rows if r["bag"] == b and r["speed"] is not None
                            and r["height_from"] == "this pass"]) for b in colours}
    handles = [Line2D([], [], ls="", marker="o", color=colours[b], label=f"{b}, about {speed[b]:.1f} m/s") for b in colours]
    handles += [Line2D([], [], ls="", marker="o", color="#444", label="wood block, plank, book"),
                Line2D([], [], ls="", marker="s", mfc="none", color="#444", label="foam"),
                Line2D([], [], ls="", marker="x", color="#9a9a96", label="between the wheels, not touched")]
    ax.legend(handles=handles, fontsize=8.5, frameon=False, loc="upper right")
    ax.set_title("Bump height against IMU jolt, one marker per pass", loc="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fmt(v, f="{:.2f}"):
    return "" if v is None else f.format(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", required=True, help="folder with the bag folders")
    ap.add_argument("--out", default="docs/results/crossing_jolts")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for b in BAGS:
        rows += analyse(Path(args.bags) / b)

    with open(out / "crossings.csv", "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    plot(rows, out / "crossing_jolts.png")

    lines = ["# Crossings: camera height against IMU jolt", "",
             "Side offset: object centre, metres left of the robot centre line (wheels at +-0.29 m). "
             "Wheels: which wheels ran over it, front / rear. Jolt in m/s^2.", "",
             "| Run | Object | Tape | Camera | Side offset | Wheels | Speed | Jolt | Felt |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        src = "" if r["height_from"] == "this pass" else " (back, no camera)"
        lines.append(f"| {r['bag'][-4:]}{src} | {r['object']} | {r['tape_mm']} mm | {r['camera_mm']:.0f} mm | "
                     f"{fmt(r['lateral_m'], '{:+.2f}')} m | {r['front_wheels'] or '-'} / {r['rear_wheels'] or '-'} | "
                     f"{fmt(r['speed'])} m/s | {fmt(r['jolt'])} | {'yes' if r['felt'] else 'no'} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
