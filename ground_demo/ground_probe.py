#!/usr/bin/env python3
"""
ground_probe: terrain geometry from a single RGB-D camera.

Fits the ground plane with RANSAC, measures the signed distance of every point
to that plane, and reports holes and bumps with their real size in millimetres.
Also reports the camera height and tilt it infers from the plane, which is what
we need to choose the arm pose on the robot.

Sources:
  --live                 Intel RealSense (needs pyrealsense2)
  --bag FILE.bag         a .bag recorded with realsense-viewer
  --synthetic            a simulated ground plane with a known box on it,
                         used to verify the maths without a camera

Keys:
  q  quit          s  toggle logging        p  save a screenshot
  1..9  set the surface label for the log   [ / ]  bump threshold down / up

Outputs (in out/):
  frames.csv       one row per frame: height, tilt, unevenness, range, label
  detections.csv   one row per detected hazard
  shot_*.png       screenshots
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# Diverging blue <-> gray <-> red. Hole is blue, flat is gray, bump is red.
C_HOLE = (214, 120, 42)    # BGR  #2a78d6
C_FLAT = (236, 239, 240)   # BGR  #f0efec
C_BUMP = (72, 73, 227)     # BGR  #e34948
C_VOID = (25, 26, 26)      # BGR  no data
C_INK = (245, 245, 245)
C_DIM = (183, 194, 195)
C_PANEL = (25, 26, 26)

SURFACE_LABELS = {
    ord("1"): "flat_calib", ord("2"): "tile", ord("3"): "concrete",
    ord("4"): "carpet", ord("5"): "asphalt", ord("6"): "grass",
    ord("7"): "gravel", ord("8"): "dirt", ord("9"): "other",
}


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------

def deproject(depth_m, fx, fy, ppx, ppy):
    """Depth image (metres, 0 = invalid) -> HxWx3 point cloud in camera frame.

    Camera optical convention: x right, y down, z forward.
    """
    h, w = depth_m.shape
    u = np.arange(w, dtype=np.float32)[None, :]
    v = np.arange(h, dtype=np.float32)[:, None]
    z = depth_m
    x = (u - ppx) / fx * z
    y = (v - ppy) / fy * z
    return np.dstack((x, y, z))


def fit_ground_plane(pts, iters=250, tol=0.012, rng=None):
    """RANSAC plane fit, constrained to look like a ground plane.

    Returns (normal, d, inlier_fraction) with the normal pointing 'up'
    (negative y in the optical frame) and the plane defined by n . p + d = 0.
    Returns None if no plausible ground plane is found.
    """
    rng = rng or np.random.default_rng(0)
    n_pts = len(pts)
    if n_pts < 200:
        return None

    best_n, best_d, best_count = None, None, 0
    idx = rng.integers(0, n_pts, size=(iters, 3))
    for tri in idx:
        p0, p1, p2 = pts[tri[0]], pts[tri[1]], pts[tri[2]]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nrm)
        if norm < 1e-6:
            continue
        nrm = nrm / norm
        d = -float(nrm @ p0)
        # Orient the normal toward the camera. The camera sits at the origin,
        # so d is its signed distance to the plane; flipping until d > 0 makes
        # the normal point back at the camera whatever the mounting angle.
        # (An earlier version keyed off the y component, which silently
        # rejected every plane when the camera looks straight down.)
        if d < 0:
            nrm, d = -nrm, -d
        # the surface must be within a plausible working range
        if not (0.10 < d < 3.0):
            continue
        count = int(np.count_nonzero(np.abs(pts @ nrm + d) < tol))
        if count > best_count:
            best_n, best_d, best_count = nrm, d, count

    if best_n is None or best_count < 0.15 * n_pts:
        return None

    # refine on the inliers with a least-squares fit
    res = pts @ best_n + best_d
    inl = pts[np.abs(res) < tol]
    centroid = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    nrm = vt[2]
    d = -float(nrm @ centroid)
    if d < 0:
        nrm, d = -nrm, -d
    frac = float(np.count_nonzero(np.abs(pts @ nrm + d) < tol)) / n_pts
    return nrm, d, frac


def refine_plane(pts, nrm, d, tol=0.012):
    """Re-fit last frame's plane to this frame. Much steadier than re-running
    RANSAC from scratch, which re-guesses and makes every number wobble."""
    inl = pts[np.abs(pts @ nrm + d) < tol]
    if len(inl) < 0.5 * len(pts):
        return None
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    n1 = vt[2]
    d1 = -float(n1 @ c)
    if d1 < 0:
        n1, d1 = -n1, -d1
    frac = float(np.count_nonzero(np.abs(pts @ n1 + d1) < tol)) / len(pts)
    return n1, d1, frac


def plane_pose(nrm, d):
    """Camera height above the plane, and tilt below horizontal, in degrees."""
    height = abs(d)
    tilt = np.degrees(np.arccos(np.clip(nrm[2], -1.0, 1.0))) - 90.0
    return height, abs(tilt)


def plane_basis(nrm):
    """An in-plane frame: forward (optical axis projected onto the plane) and right."""
    axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    fwd = axis - (axis @ nrm) * nrm
    n_f = np.linalg.norm(fwd)
    if n_f < 1e-6:
        fwd = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    else:
        fwd = fwd / n_f
    right = np.cross(nrm, fwd)
    return fwd, right


def find_hazards(cloud, valid, resid, nrm, d, thresh, min_area_cm2, fx, fy,
                 grow_frac=0.5):
    """Cluster the points that stand off the plane into holes and bumps.

    A hazard is detected where the residual passes `thresh`, but its height is
    measured over the outline grown down to `grow_frac * thresh`. Measuring only
    the pixels above the threshold keeps the top of the noise and reads thin
    objects high: a 15 mm board came out at 22 mm, a 20 mm strip at 24 mm.
    Over the grown outline the median is within a few mm on every tape-measured
    object (docs/DATA.md, BRANCH_NOTES.md).
    """
    fwd, right = plane_basis(nrm)
    origin_on_plane = -d * nrm          # the camera's own footprint on the plane
    out = []

    for kind, sgn in (("bump", 1.0), ("hole", -1.0)):
        sel = (sgn * resid > thresh) & valid
        m = sel.astype(np.uint8)
        if not m.any():
            continue
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        low = ((sgn * resid > grow_frac * thresh) & valid).astype(np.uint8)
        _, lab_low = cv2.connectedComponents(low, connectivity=8)
        for i in range(1, n_lab):
            if stats[i, cv2.CC_STAT_AREA] < 40:
                continue
            blob = lab == i
            pts = cloud[blob]
            r = resid[blob]
            pts = pts[np.isfinite(pts).all(axis=1)]
            if len(pts) < 40:
                continue

            # true area: project onto the plane and take the convex hull
            loc = np.stack(((pts - origin_on_plane) @ right,
                            (pts - origin_on_plane) @ fwd), axis=1)
            hull = cv2.convexHull(loc.astype(np.float32))
            area_m2 = float(cv2.contourArea(hull))
            if area_m2 * 1e4 < min_area_cm2:
                continue

            # minimum-area rotated rectangle: the object's own axes, not the
            # camera's, so a box sitting at an angle is not overestimated
            (_, _), (side_a, side_b), _ = cv2.minAreaRect(hull)
            length, width = max(side_a, side_b), min(side_a, side_b)
            centroid = pts.mean(axis=0)
            ahead = float((centroid - origin_on_plane) @ fwd)

            ids = np.unique(lab_low[blob])
            grown = np.isin(lab_low, ids[ids > 0])

            out.append({
                "type": kind,
                "centroid": tuple(float(c) for c in centroid),
                "size_mm": float(np.median(np.abs(resid[grown])) * 1000.0),
                "peak_mm": float(np.percentile(np.abs(r), 90) * 1000.0),
                "median_mm": float(np.median(np.abs(r)) * 1000.0),
                "area_cm2": area_m2 * 1e4,
                "length_mm": length * 1000.0,
                "width_mm": width * 1000.0,
                "ahead_m": ahead,
                "range_m": float(centroid[2]),
                "bbox": tuple(int(x) for x in (
                    stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                    stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])),
            })

    out.sort(key=lambda h: -h["size_mm"])
    return out


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------

def diverging_lut():
    lut = np.zeros((256, 3), np.uint8)
    for i in range(256):
        t = i / 255.0
        if t < 0.5:
            a, b, f = C_HOLE, C_FLAT, t / 0.5
        else:
            a, b, f = C_FLAT, C_BUMP, (t - 0.5) / 0.5
        lut[i] = [int(round(a[c] + (b[c] - a[c]) * f)) for c in range(3)]
    return lut


LUT = diverging_lut()


def residual_view(resid, valid, span):
    idx = np.clip((resid / span) * 127.5 + 127.5, 0, 255).astype(np.uint8)
    img = LUT[idx]
    img[~valid] = C_VOID
    return img


def draw_hazards(img, hazards, limit=6):
    for h in hazards[:limit]:
        x, y, w, hh = h["bbox"]
        col = C_BUMP if h["type"] == "bump" else C_HOLE
        cv2.rectangle(img, (x, y), (x + w, y + hh), col, 2)
        tag = (f'{h["type"]}  {h["length_mm"]:.0f} x {h["width_mm"]:.0f} x '
               f'{h["size_mm"]:.0f} mm  {h["range_m"]:.2f}m away')
        ty = y - 6 if y > 18 else y + hh + 16
        cv2.putText(img, tag, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(img, tag, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    return img


def stats_panel(size, rows):
    w, h = size
    img = np.full((h, w, 3), C_PANEL, np.uint8)
    y = 30
    for text, emphasis in rows:
        if text == "":
            y += 10
            continue
        scale, col, thick = (0.62, C_INK, 2) if emphasis else (0.52, C_DIM, 1)
        cv2.putText(img, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick,
                    cv2.LINE_AA)
        y += 26 if emphasis else 22
        if y > h - 10:
            break
    return img


# --------------------------------------------------------------------------
# sources
# --------------------------------------------------------------------------

class SyntheticSource:
    """A known ground plane with a known box, for verifying the maths."""

    def __init__(self, height=0.75, tilt_deg=40.0, box_mm=25.0, noise=True):
        self.w, self.h = 848, 480
        self.fx = self.fy = 425.0
        self.ppx, self.ppy = 424.0, 240.0
        self.truth = {"height": height, "tilt": tilt_deg, "box_mm": box_mm}
        t = np.radians(tilt_deg)
        self.n = np.array([0.0, -np.cos(t), -np.sin(t)], np.float32)
        self.d = height
        self.box_mm = box_mm
        self.noise = noise
        self.rng = np.random.default_rng(1)
        self.k = 0

    def intrinsics(self):
        return self.fx, self.fy, self.ppx, self.ppy

    def read(self):
        self.k += 1
        u = np.arange(self.w, dtype=np.float32)[None, :]
        v = np.arange(self.h, dtype=np.float32)[:, None]
        rx = (u - self.ppx) / self.fx
        ry = (v - self.ppy) / self.fy
        rays = np.dstack((np.broadcast_to(rx, (self.h, self.w)),
                          np.broadcast_to(ry, (self.h, self.w)),
                          np.ones((self.h, self.w), np.float32)))
        denom = rays @ self.n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(denom < -1e-6, -self.d / denom, 0.0)
        pts = rays * t[..., None]

        # a box on the ground, drifting forward so it sweeps through the frame
        fwd, right = plane_basis(self.n)
        org = -self.d * self.n
        loc_f = (pts - org) @ fwd
        loc_r = (pts - org) @ right
        cx = 1.15 - 0.012 * (self.k % 70)
        box = (np.abs(loc_r) < 0.11) & (np.abs(loc_f - cx) < 0.09) & (t > 0)
        t_box = np.where(denom < -1e-6, -(self.d - self.box_mm / 1000.0) / denom, 0.0)
        t = np.where(box, t_box, t)

        z = (rays[..., 2] * t).astype(np.float32)
        z[t <= 0] = 0.0
        z[z > 4.0] = 0.0
        if self.noise:
            sigma = 0.0015 + 0.004 * z ** 2          # D435-like, grows with range
            z = np.where(z > 0, z + self.rng.normal(0, 1, z.shape) * sigma, 0)
        colour = cv2.applyColorMap(
            np.clip(z / 3.0 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_BONE)
        return z.astype(np.float32), colour

    def close(self):
        pass


class RealSenseSource:
    def __init__(self, bag=None, width=848, height=480, fps=30):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if bag:
            cfg.enable_device_from_file(bag, repeat_playback=True)
        else:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        dev = self.profile.get_device()
        self.scale = dev.first_depth_sensor().get_depth_scale()
        if not bag:
            # fixed exposure keeps the colour stream consistent between runs
            try:
                cs = [s for s in dev.query_sensors() if s.get_info(rs.camera_info.name)
                      == "RGB Camera"][0]
                cs.set_option(rs.option.enable_auto_exposure, 0)
                cs.set_option(rs.option.exposure, 120)
            except Exception:
                pass
        self._intr = None

    def intrinsics(self):
        return self._intr

    def read(self):
        frames = self.align.process(self.pipe.wait_for_frames())
        df = frames.get_depth_frame()
        cf = frames.get_color_frame()
        if not df or not cf:
            return None, None
        i = df.profile.as_video_stream_profile().intrinsics
        self._intr = (i.fx, i.fy, i.ppx, i.ppy)
        z = np.asanyarray(df.get_data()).astype(np.float32) * self.scale
        return z, np.asanyarray(cf.get_data())

    def close(self):
        self.pipe.stop()


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--live", action="store_true", help="use an attached RealSense")
    src.add_argument("--bag", help="replay a .bag file")
    src.add_argument("--synthetic", action="store_true",
                     help="simulated ground with a known box (self-test)")
    ap.add_argument("--thresh-mm", type=float, default=15.0,
                    help="stand-off from the plane that counts as a hazard")
    ap.add_argument("--min-area-cm2", type=float, default=25.0)
    ap.add_argument("--zmin", type=float, default=0.30, help="near clip, metres")
    ap.add_argument("--zmax", type=float, default=1.60,
                    help="far clip, metres; beyond this D435 noise swamps a 20mm step")
    ap.add_argument("--span-mm", type=float, default=40.0,
                    help="full scale of the residual colour map")
    ap.add_argument("--smooth", type=int, default=9,
                    help="median-smooth the displayed size over N frames")
    ap.add_argument("--hold", type=int, default=10,
                    help="frames to keep showing a hazard after it drops out; "
                         "stops the box flickering. 0 disables.")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="window size; 1.0 is full size, 0.6 is small")
    ap.add_argument("--headless", type=int, default=0,
                    help="run N frames with no window and print a summary")
    args = ap.parse_args()

    if args.synthetic:
        source = SyntheticSource()
    elif args.bag or args.live:
        try:
            source = RealSenseSource(bag=args.bag)
        except ImportError:
            sys.exit("pyrealsense2 is not installed.  pip install pyrealsense2\n"
                     "Or try --synthetic to check the pipeline without a camera.")
    else:
        sys.exit("pick a source: --live, --bag FILE, or --synthetic")

    os.makedirs(OUT, exist_ok=True)
    f_frames = open(os.path.join(OUT, "frames.csv"), "a", newline="")
    f_det = open(os.path.join(OUT, "detections.csv"), "a", newline="")
    w_frames = csv.writer(f_frames)
    w_det = csv.writer(f_det)
    if f_frames.tell() == 0:
        w_frames.writerow(["t", "label", "height_m", "tilt_deg", "unevenness_mm",
                           "inlier_frac", "mean_range_m", "n_hazards"])
    if f_det.tell() == 0:
        w_det.writerow(["t", "label", "type", "size_mm", "median_mm", "area_cm2", "length_mm",
                        "width_mm", "ahead_m", "range_m"])

    thresh = args.thresh_mm / 1000.0
    label, logging_on, shots = "unlabelled", False, 0
    fps, last = 0.0, time.time()
    rng = np.random.default_rng(7)
    headless_rows = []

    frame_no = 0
    held, hold_left = [], 0
    prev_plane = None
    smooth = {"length_mm": [], "width_mm": [], "size_mm": []}
    while True:
        depth, colour = source.read()
        if depth is None:
            break
        frame_no += 1
        intr = source.intrinsics()
        if intr is None:
            continue
        fx, fy, ppx, ppy = intr

        cloud = deproject(depth, fx, fy, ppx, ppy)
        valid = (depth > args.zmin) & (depth < args.zmax)

        rows = []
        hazards = []
        resid = np.zeros_like(depth)
        height = tilt = unevenness = frac = mean_range = float("nan")

        sub = cloud[valid]
        if len(sub) > 500:
            sample = sub[rng.integers(0, len(sub), size=min(6000, len(sub)))]
            fit = refine_plane(sample, *prev_plane) if prev_plane else None
            if fit is None:
                fit = fit_ground_plane(sample, rng=rng)
            if fit is not None:
                prev_plane = (fit[0], fit[1])
                nrm, d, frac = fit
                resid = (cloud @ nrm + d).astype(np.float32)
                resid[~valid] = 0.0
                # depth noise is independent per pixel and grows with range;
                # a 5x5 median removes most of it without rounding real steps.
                # erode the valid mask so the filter's edge effects are dropped.
                resid = cv2.medianBlur(resid, 5)
                valid = cv2.erode(valid.astype(np.uint8),
                                  np.ones((5, 5), np.uint8)).astype(bool)
                resid[~valid] = 0.0
                height, tilt = plane_pose(nrm, d)
                flat = valid & (np.abs(resid) < thresh)
                unevenness = float(np.sqrt(np.mean(resid[flat] ** 2)) * 1000.0) \
                    if flat.any() else float("nan")
                mean_range = float(depth[valid].mean())
                hazards = find_hazards(cloud, valid, resid, nrm, d, thresh,
                                       args.min_area_cm2, fx, fy)

        if hazards:
            big = hazards[0]
            for k, buf in smooth.items():
                buf.append(big[k])
                del buf[:-args.smooth]
                big[k] = float(np.median(buf))
            held, hold_left = hazards, args.hold
        elif hold_left > 0:
            hold_left -= 1
        shown = hazards if hazards else (held if hold_left > 0 else [])

        now = time.time()
        dt = now - last
        last = now
        fps = 0.9 * fps + 0.1 / dt if dt > 0 else fps

        if logging_on and np.isfinite(height):
            w_frames.writerow([f"{now:.3f}", label, f"{height:.4f}", f"{tilt:.2f}",
                               f"{unevenness:.3f}", f"{frac:.3f}",
                               f"{mean_range:.3f}", len(hazards)])
            for h in hazards:
                w_det.writerow([f"{now:.3f}", label, h["type"], f'{h["size_mm"]:.1f}',
                                f'{h["median_mm"]:.1f}',
                                f'{h["area_cm2"]:.1f}', f'{h["length_mm"]:.1f}',
                                f'{h["width_mm"]:.1f}', f'{h["ahead_m"]:.3f}',
                                f'{h["range_m"]:.3f}'])

        if args.headless:
            headless_rows.append((height, tilt, unevenness, hazards))
            if frame_no >= args.headless:
                break
            continue

        # ---- display -------------------------------------------------
        h_img, w_img = depth.shape
        left = draw_hazards(colour.copy(), shown)
        depth_vis = cv2.applyColorMap(
            np.clip(depth / args.zmax * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_VIRIDIS)
        depth_vis[depth == 0] = C_VOID
        res_vis = residual_view(resid, valid, args.span_mm / 1000.0)

        rows = [
            ("GROUND PROBE", True),
            ("", False),
            (f"camera height   {height:6.3f} m" if np.isfinite(height)
             else "camera height      --", True),
            (f"camera tilt     {tilt:6.1f} deg" if np.isfinite(tilt)
             else "camera tilt        --", True),
            (f"plane inliers   {frac*100:5.1f} %" if np.isfinite(frac)
             else "plane inliers      --", False),
            ("", False),
            (f"unevenness RMS  {unevenness:6.2f} mm" if np.isfinite(unevenness)
             else "unevenness RMS     --", True),
            (f"mean range      {mean_range:6.3f} m" if np.isfinite(mean_range)
             else "mean range         --", False),
            ("", False),
            (f"hazards         {len(shown)}", True),
        ]
        for h in shown[:3]:
            depth_word = "height" if h["type"] == "bump" else "depth "
            rows.append((f'  {h["type"].upper()}', True))
            rows.append((f'    {h["range_m"]:6.2f} m from camera', False))
            rows.append((f'    {h["ahead_m"]:6.2f} m ahead on floor', False))
            rows.append((f'    length {h["length_mm"]:6.0f} mm', False))
            rows.append((f'    width  {h["width_mm"]:6.0f} mm', False))
            rows.append((f'    {depth_word} {h["size_mm"]:6.0f} mm', False))
            rows.append((f'    area   {h["area_cm2"]:6.0f} cm2', False))
        rows += [
            ("", False),
            (f"threshold       {thresh*1000:.0f} mm", False),
            (f"band            {args.zmin:.2f} - {args.zmax:.2f} m", False),
            (f"label           {label}", False),
            (f"logging         {'ON' if logging_on else 'off'}",  logging_on),
            (f"fps             {fps:.1f}", False),
        ]
        right = stats_panel((w_img, h_img), rows)

        top = np.hstack((left, depth_vis))
        bot = np.hstack((res_vis, right))
        panel = np.vstack((top, bot))
        show = panel if abs(args.scale - 1.0) < 1e-3 else cv2.resize(
            panel, None, fx=args.scale, fy=args.scale,
            interpolation=cv2.INTER_AREA if args.scale < 1 else cv2.INTER_LINEAR)
        cv2.imshow("ground_probe  [q]uit  [s]ave-log  [p]hoto  1-9 label", show)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("s"):
            logging_on = not logging_on
        elif k == ord("p"):
            shots += 1
            p = os.path.join(OUT, f"shot_{label}_{shots:02d}.png")
            cv2.imwrite(p, panel)
            print("saved", p)
        elif k == ord("["):
            thresh = max(0.003, thresh - 0.002)
        elif k == ord("]"):
            thresh = min(0.100, thresh + 0.002)
        elif k in SURFACE_LABELS:
            label = SURFACE_LABELS[k]

    source.close()
    f_frames.close()
    f_det.close()
    if not args.headless:
        cv2.destroyAllWindows()

    if args.headless and headless_rows:
        hs = np.array([r[0] for r in headless_rows], float)
        ts = np.array([r[1] for r in headless_rows], float)
        us = np.array([r[2] for r in headless_rows], float)
        big = [max((h["size_mm"] for h in r[3]), default=0.0) for r in headless_rows]
        print(f"frames            {len(headless_rows)}")
        print(f"height     mean   {np.nanmean(hs):.4f} m   sd {np.nanstd(hs)*1000:.1f} mm")
        print(f"tilt       mean   {np.nanmean(ts):.2f} deg  sd {np.nanstd(ts):.2f} deg")
        print(f"unevenness mean   {np.nanmean(us):.2f} mm")
        print(f"largest hazard    {np.nanmean([b for b in big if b > 0] or [0]):.1f} mm "
              f"(detected in {sum(1 for b in big if b > 0)}/{len(big)} frames)")
        if isinstance(source, SyntheticSource):
            t = source.truth
            print("\nground truth      "
                  f'height {t["height"]:.3f} m, tilt {t["tilt"]:.1f} deg, '
                  f'box {t["box_mm"]:.1f} mm')


if __name__ == "__main__":
    main()
