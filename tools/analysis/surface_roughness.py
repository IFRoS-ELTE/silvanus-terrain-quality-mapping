#!/usr/bin/env python3
"""Roughness per surface from the bags: IMU node replay, 0.25 m records, figures.

    python tools/analysis/surface_roughness.py --bags D:/bags --out docs/results/surface_roughness

No ROS needed (reads the .mcap bags with `rosbags`); run it in the
vision_training environment.

What it does, per bag:
1. Replays /filter/free_acceleration and /odom through terrain_imu.roughness,
   the class the IMU roughness node uses, sampled at the node's 4 Hz. That is
   the bag-replay check of the node: the numbers here are what it publishes.
2. Computes the camera unevenness of every second depth frame exactly as
   tools/analysis/outdoor_figures.py does (RANSAC plane, RMS of the flat
   points, gripper masked).
3. Cuts the driven path into 0.25 m pieces (distance from wheel odometry) and
   writes one record per piece: IMU roughness when the wheels leave it,
   median camera unevenness of the frames that looked at it, and the surface
   from vision_training/configs/terrain_labels.yaml (the checked labels).

The camera looks about 0.85 m ahead of the robot (it sees an object about 4 s
before the wheels reach it at 0.21 m/s, docs/PROGRESS.md), so a depth frame
taken when the robot is at distance s describes the ground at s + 0.85 m.

Outputs: records.csv, node_replay.csv, summary.md, roughness_per_surface.png,
unevenness_vs_imu.png.
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ground_demo"))
sys.path.insert(0, str(ROOT / "src" / "terrain_imu"))
import ground_probe as gp  # noqa: E402
from terrain_imu.roughness import SAMPLES, SPEED_RANGE, RoughnessWindow, band_ratio  # noqa: E402

COLOUR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEPTH_INFO = "/camera/camera/depth/camera_info"
ACCEL_TOPIC = "/filter/free_acceleration"

# Bags with the 100 Hz IMU and labelled surfaces. 1338 and 1434 only have the
# old 10 Hz IMU, too slow for the 8-50 Hz band; 1703 was driven at 0.6 m/s.
BAGS = ["garden_surfaces_1548", "garden_surfaces_1607", "garden_surfaces_1618", "garden_surfaces_1628",
        "corridor_groundtruth_1447", "crossing_slow_1652"]

# Camera unevenness settings of outdoor_figures.py, unchanged.
THRESH, GRIP_ROW = 0.015, 340
STEP_M, LOOKAHEAD_M = 0.25, 0.85

ORDER = ["tile", "asphalt", "red_path", "wood_chips", "grass_dirt"]
NAMES = {"tile": "tile (indoor)", "asphalt": "asphalt", "red_path": "red path",
         "wood_chips": "wood chips", "grass_dirt": "grass/dirt"}
# Same colour per surface as the terrain classifier figures.
COLOURS = {"asphalt": "#2a78d6", "red_path": "#eb6834", "wood_chips": "#1baf7a",
           "grass_dirt": "#eda100", "tile": "#e87ba4"}
# Medians from docs/PROGRESS.md section 3, for the comparison table.
PROGRESS_MEDIANS = {"asphalt": (3.9, 2.2), "red_path": (3.6, 0.51), "wood_chips": (7.5, 0.41),
                    "grass_dirt": (6.8, 0.29)}


class Labels:
    def __init__(self, path):
        cfg = yaml.safe_load(Path(path).read_text())
        self.margin = float(cfg.get("margin_s", 2.0))
        self.bags = cfg["bags"]

    def at(self, bag, t):
        for a, b, label in self.bags.get(bag, []):
            if a + self.margin <= t <= b - self.margin:
                return label
        return None


def read_bag(bag_dir):
    """All we need from one bag, in log-time order, times in seconds from the first colour frame
    (the clock of the surface labels)."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    accel, odom, depth = [], [], []
    K, mask = None, None
    with AnyReader([Path(bag_dir)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        conns = [c for c in reader.connections if c.topic in (COLOUR_TOPIC, DEPTH_TOPIC, DEPTH_INFO, ACCEL_TOPIC, "/odom")]
        t0, n_depth = None, 0
        for conn, stamp, raw in reader.messages(connections=conns):
            if conn.topic == COLOUR_TOPIC:
                t0 = stamp if t0 is None else t0
                continue
            if conn.topic == DEPTH_INFO:
                if K is None:
                    m = reader.deserialize(raw, conn.msgtype)
                    K = (m.k[0], m.k[4], m.k[2], m.k[5])
                continue
            if conn.topic == DEPTH_TOPIC:
                n_depth += 1
                if n_depth % 2:          # every second frame, like outdoor_figures.py
                    continue
                m = reader.deserialize(raw, conn.msgtype)
                d = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width)
                depth.append((stamp, d.astype(np.float32) * 0.001))
                continue
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == ACCEL_TOPIC:
                accel.append((stamp, m.vector.z))
            else:
                h = m.header.stamp
                odom.append((stamp, h.sec + h.nanosec * 1e-9, m.twist.twist.linear.x))
    if t0 is None:
        raise ValueError(f"{bag_dir}: no colour frames, cannot align with the surface labels")
    rel = lambda ns: (ns - t0) * 1e-9  # noqa: E731
    accel = np.array([(rel(s), z) for s, z in accel])
    odom = np.array([(rel(s), hs, v) for s, hs, v in odom])
    depth = [(rel(s), d) for s, d in depth]
    return accel, odom, depth, K


def node_replay(accel, odom, rate_hz=4.0):
    """Feed messages to RoughnessWindow in bag order and sample it at the node's timer rate."""
    events = [(t, 0, z) for t, z in accel] + [(t, 1, (hs, v)) for t, hs, v in odom]
    events.sort(key=lambda e: e[0])
    w, out, next_tick = RoughnessWindow(), [], events[0][0]
    for t, kind, value in events:
        while t >= next_tick:
            out.append((next_tick, w.roughness(), w.ratio(), w.mean_speed()))
            next_tick += 1.0 / rate_hz
        if kind == 0:
            w.add_accel(value)
        else:
            w.add_speed(*value)
    return np.array(out)


def camera_unevenness(depth, K, mask_ok):
    rng, prev, out = np.random.default_rng(0), None, []
    for t, d in depth:
        cloud = gp.deproject(d, *K)
        valid = (d > 0.30) & (d < 1.60) & mask_ok
        pts = cloud[valid]
        if len(pts) < 500:
            continue
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = (gp.refine_plane(sample, *prev) if prev else None) or gp.fit_ground_plane(sample, rng=rng)
        if fit is None:
            prev = None
            continue
        nrm, dd, _ = fit
        prev = (nrm, dd)
        resid = ((cloud @ nrm) + dd).astype(np.float32)
        resid[~valid] = 0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        resid[~valid] = 0
        flat = valid & (np.abs(resid) < THRESH)
        flat[GRIP_ROW:] = False
        if flat.any():
            out.append((t, float(np.sqrt(np.mean(resid[flat] ** 2)) * 1000)))
    return np.array(out)


def records_for(bag, accel, odom, unev, labels):
    t_odom, v = odom[:, 0], np.abs(odom[:, 2])
    s = np.concatenate([[0.0], np.cumsum(v[1:] * np.diff(t_odom))])   # distance driven, m
    rows = []
    for k in range(int(s[-1] // STEP_M)):
        a, b = k * STEP_M, (k + 1) * STEP_M
        t_end = float(np.interp(b, s, t_odom))
        t_mid = float(np.interp((a + b) / 2, s, t_odom))
        idx = np.searchsorted(accel[:, 0], t_end, side="right")
        ratio = band_ratio(accel[idx - SAMPLES:idx, 1]) if idx >= SAMPLES else float("nan")
        speed = float(np.mean(v[(t_odom > t_end - 1.8) & (t_odom <= t_end)]))
        cam = []
        if len(unev):
            ground = np.interp(unev[:, 0], t_odom, s) + LOOKAHEAD_M   # where each depth frame looked
            cam = unev[(ground >= a) & (ground < b), 1]
        rows.append({
            "bag": bag, "s_m": round(a, 2), "t": round(t_mid, 2), "speed": round(speed, 3),
            "imu_ratio": round(ratio, 4) if np.isfinite(ratio) else "",
            "imu_valid": int(SPEED_RANGE[0] <= speed <= SPEED_RANGE[1] and np.isfinite(ratio)),
            "camera_unevenness_mm": round(float(np.median(cam)), 2) if len(cam) else "",
            "surface": labels.at(bag, t_mid) or "",
        })
    return rows


def plot_boxes(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    panels = [("camera_unevenness_mm", "camera unevenness, mm (RMS height around the ground plane)", False, False),
              ("imu_ratio", "IMU roughness: power 8-50 Hz / 0.5-8 Hz", True, True)]
    for ax, (key, title, log, need_speed) in zip(axes, panels):
        data, names = [], []
        for surf in ORDER:
            vals = [float(r[key]) for r in rows if r["surface"] == surf and r[key] != ""
                    and (not need_speed or r["imu_valid"])]
            data.append(vals)
            names.append(f"{NAMES[surf]}\nn={len(vals)}")
        bp = ax.boxplot(data, widths=0.55, patch_artist=True, showfliers=True,
                        medianprops={"color": "#1f1f1f", "linewidth": 2},
                        flierprops={"marker": "o", "markersize": 3, "markeredgecolor": "#8a8a8a"},
                        whiskerprops={"color": "#8a8a8a"}, capprops={"color": "#8a8a8a"})
        for patch, surf in zip(bp["boxes"], ORDER):
            patch.set_facecolor(COLOURS[surf])
            patch.set_alpha(0.75)
            patch.set_edgecolor(COLOURS[surf])
        ax.set_xticks(range(1, len(ORDER) + 1), names, fontsize=8.5)
        ax.set_title(title, fontsize=10, loc="left")
        if log:
            ax.set_yscale("log")
        ax.grid(axis="y", color="#e6e6e6", linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    fig.text(0.01, 0.01, "One value per 0.25 m of driven path. IMU only at 0.25-0.50 m/s. "
             "Tile from the indoor runs 1447 and 1652, outdoor surfaces from 1548, 1607, 1618, 1628.",
             fontsize=8, color="#555555")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_scatter(rows, path, rho):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.2, 5))
    for surf in ORDER:
        pts = np.array([(float(r["camera_unevenness_mm"]), float(r["imu_ratio"])) for r in rows
                        if r["surface"] == surf and r["imu_valid"] and r["camera_unevenness_mm"] != ""])
        if not len(pts):
            continue
        ax.scatter(pts[:, 0], pts[:, 1], s=14, color=COLOURS[surf], alpha=0.55, linewidths=0,
                   label=f"{NAMES[surf]} (n={len(pts)})")
        mx, my = np.median(pts[:, 0]), np.median(pts[:, 1])
        ax.scatter([mx], [my], s=90, color=COLOURS[surf], edgecolors="white", linewidths=2, zorder=3)
    ax.set_yscale("log")
    ax.set_xlabel("camera unevenness, mm (predicted from depth, ground ahead)")
    ax.set_ylabel("IMU roughness, 8-50 Hz / 0.5-8 Hz (felt by the wheels)")
    ax.set_title(f"Each dot is 0.25 m of path; large dots are the median per surface. Spearman rho = {rho:.2f}",
                 fontsize=9.5, loc="left")
    ax.grid(color="#eeeeee", linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=8.5, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", required=True, help="folder with the bag folders")
    ap.add_argument("--labels", default=str(ROOT / "vision_training" / "configs" / "terrain_labels.yaml"))
    ap.add_argument("--out", default=str(ROOT / "docs" / "results" / "surface_roughness"))
    args = ap.parse_args()

    labels = Labels(args.labels)
    mask_ok = cv2.imread(str(ROOT / "ground_demo" / "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows, replay_rows = [], []
    for bag in BAGS:
        bag_dir = Path(args.bags) / bag
        if not bag_dir.exists():
            print(f"{bag}: not found, skipped")
            continue
        accel, odom, depth, K = read_bag(bag_dir)
        rep = node_replay(accel, odom)
        for t, rough, ratio, speed in rep:
            replay_rows.append({"bag": bag, "t": round(t, 2),
                                "roughness": round(rough, 4) if np.isfinite(rough) else "",
                                "ratio": round(ratio, 4) if np.isfinite(ratio) else "",
                                "speed": round(speed, 3) if np.isfinite(speed) else "",
                                "surface": labels.at(bag, t) or ""})
        unev = camera_unevenness(depth, K, mask_ok)
        bag_rows = records_for(bag, accel, odom, unev, labels)
        rows += bag_rows
        print(f"{bag}: {len(accel)} accel, {len(depth)} depth frames used, {len(bag_rows)} records of 0.25 m, "
              f"{sum(r['imu_valid'] for r in bag_rows)} at 0.25-0.50 m/s")

    for name, data in (("records.csv", rows), ("node_replay.csv", replay_rows)):
        with open(out / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(data[0]))
            w.writeheader()
            w.writerows(data)

    from scipy.stats import spearmanr
    both = [r for r in rows if r["surface"] and r["imu_valid"] and r["camera_unevenness_mm"] != ""]
    rho, p = spearmanr([float(r["camera_unevenness_mm"]) for r in both], [float(r["imu_ratio"]) for r in both])
    plot_boxes([r for r in rows if r["surface"]], out / "roughness_per_surface.png")
    plot_scatter(both, out / "unevenness_vs_imu.png", rho)

    def med(vals):
        return f"{np.median(vals):.2f}" if vals else "n/a"

    lines = ["# Roughness per surface", "",
             "One record per 0.25 m of driven path. IMU values only at 0.25-0.50 m/s. "
             "Node replay = the 4 Hz values the IMU roughness node publishes on these bags.", "",
             "| Surface | Records (0.25 m) | Camera unevenness, median mm | IMU roughness, median | "
             "Node replay, median | PROGRESS.md medians (camera, IMU) |", "|---|---|---|---|---|---|"]
    for surf in ORDER:
        cam = [float(r["camera_unevenness_mm"]) for r in rows if r["surface"] == surf and r["camera_unevenness_mm"] != ""]
        imu = [float(r["imu_ratio"]) for r in rows if r["surface"] == surf and r["imu_valid"]]
        node = [float(r["roughness"]) for r in replay_rows if r["surface"] == surf and r["roughness"] != ""]
        ref = PROGRESS_MEDIANS.get(surf)
        lines.append(f"| {NAMES[surf]} | {sum(r['surface'] == surf for r in rows)} | {med(cam)} | {med(imu)} | "
                     f"{med(node)} | {f'{ref[0]} mm, {ref[1]}' if ref else 'n/a'} |")
    lines += ["", f"Camera unevenness against IMU roughness, {len(both)} records with both: "
              f"Spearman rho = {rho:.2f} (p = {p:.1g})."]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
