#!/usr/bin/env python3
"""Offline hazard map from one bag: bumps, holes, cracks, roughness and the A/B/C classes.

    hazard_map.py BAG OUT_DIR [--no-cracks]

Everything the robot side and the crack model produce, put on one map:

  path        wheel speed + Xsens heading (docs/PROGRESS.md section 4)
  bumps/holes ground_probe.find_hazards on every 3rd depth frame, robot mask on.
              Skipped when under half of the points lie on the plane (wood chips)
  cracks      the crack U-Net (crack_unet16_v1.onnx, in src/crack_detection/models or
              vision_training/models) with the crack node's colour robot mask, on the
              colour image 3 times a second. Crack pixels go to 3D through the
              depth image. Dropped: long straight lines (tile grout, paint) and
              pixels on a height step (kerb and door edges)
  roughness   camera unevenness per frame, IMU fast/slow ratio over 1.8 s
  classes     a hazard is felt when a wheel crosses it and the IMU jolt at that
              moment stands out from the shaking around it.
              A seen not felt, B felt not seen, C seen and felt

Writes OUT_DIR/records.jsonl and hazards.json (docs/schema.md) and map.png.
Surface labels for the record file come from docs/DATA.md (surface_log).
"""
import argparse, json, os, sys
import numpy as np, cv2
from scipy import ndimage
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "ground_demo"))
sys.path.insert(0, os.path.join(REPO, "vision_training", "deploy"))
import ground_probe as gp
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from tf2_msgs.msg import TFMessage
import tf2_ros, rclpy.time, rclpy.duration

DEPTH, DINFO = "/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"
COLOR, CINFO = "/camera/camera/color/image_raw", "/camera/camera/color/camera_info"
THRESH, MIN_AREA, MAX_HEIGHT, MIN_INLIERS, GRIP_ROW = 0.015, 25.0, 0.30, 0.5, 340
K_ROUGH = float(os.environ.get("K_ROUGH", 3.0))   # threshold = max(15 mm, K_ROUGH x the frame's unevenness)
WHEELS = [(0.249, 0.2915), (0.249, -0.2915), (-0.249, 0.2915), (-0.249, -0.2915)]   # scout_v2.xacro
TYRE_HALF = 0.06          # half the tyre contact width, m
MERGE_M = 0.30            # docs/schema.md: detections closer than this are one hazard
MIN_SEEN = 3              # a bump/hole seen in fewer frames is dropped as noise
CELL = 0.05               # crack grid, m
LARGE_MM, ROUGH_MM = 30, 5.0   # on ground rougher than 5 mm, bumps and holes under 30 mm are drawn faint: tufts, chips
SURFACES = {              # docs/DATA.md, seconds from the start of the bag
    "garden_surfaces_1536": [("tile", 0, 5), ("door threshold", 30, 36), ("metal drain grate", 43, 48), ("smooth concrete", 48, 60)],
    "garden_surfaces_1548": [("asphalt", 0, 10), ("grass/dirt", 12, 55), ("cracked asphalt", 57, 72), ("concrete", 72, 75), ("asphalt", 76, 87)],
    "garden_surfaces_1618": [("cracked asphalt", 0, 36), ("grass/dirt", 38, 60), ("red rubber path", 62, 75), ("wood chips", 77, 89)],
    "garden_surfaces_1628": [("red rubber path", 0, 21), ("kerb", 22, 26), ("wood chips", 27, 45), ("grass/dirt", 46, 65), ("asphalt", 67, 89)],
    "corridor_groundtruth_1447": [("tile", 0, 60)], "crossing_slow_1652": [("tile", 0, 90)], "crossing_fast_1703": [("tile", 0, 90)],
}
C_BUMP, C_HOLE, C_CRACK, INK, MUTED = "#e34948", "#2a78d6", "#c98c00", "#2b2b29", "#8a8a86"


def open_bag(bag, topics):
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    if topics:
        r.set_filter(StorageFilter(topics=topics))
    return r


def tf_matrix(t):
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [p.x, p.y, p.z]
    return M


def camera_transforms(bag):
    """base_footprint <- depth and colour optical frames. The arm does not move during a run."""
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    r = open_bag(bag, ["/tf", "/tf_static"]); n = 0
    while r.has_next() and n < 400:
        tp, data, _ = r.read_next(); n += 1
        for tr in deserialize_message(data, TFMessage).transforms:
            (buf.set_transform_static if tp == "/tf_static" else buf.set_transform)(tr, "bag")
    get = lambda f: tf_matrix(buf.lookup_transform("base_footprint", f, rclpy.time.Time()).transform)
    return get("camera_depth_optical_frame"), get("camera_color_optical_frame")


def motion(bag, t_start, poses=None):
    """Path from wheel speed + IMU heading, IMU fast/slow ratio, and the vertical jolt signal.
    With poses (a run_slam.sh / pose_logger.py CSV with map_yaw) the position and heading come
    from SLAM instead, in its map frame; the wheels still give speed and distance driven."""
    O, Y, F = [], [], []
    r = open_bag(bag, ["/odom", "/imu/data", "/filter/free_acceleration"])
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == "/odom":
            O.append((t, deserialize_message(data, Odometry).twist.twist.linear.x))
        elif tp == "/imu/data":
            q = deserialize_message(data, Imu).orientation
            Y.append((t, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        else:
            F.append((t, deserialize_message(data, Vector3Stamped).vector.z))
    if not O or not Y:
        raise SystemExit(f"{os.path.basename(bag)}: no /odom or /imu/data in this bag, it cannot be put on a map")
    O, Y, F = np.array(O), np.array(Y), np.array(F)
    yaw = np.interp(O[:, 0], Y[:, 0], np.unwrap(Y[:, 1]) - Y[0, 1])
    dt = np.diff(O[:, 0], prepend=O[0, 0])
    px, py = np.cumsum(O[:, 1] * dt * np.cos(yaw)), np.cumsum(O[:, 1] * dt * np.sin(yaw))
    if poses is not None:
        P = np.genfromtxt(poses, delimiter=",", names=True)
        tp = P["t"] - t_start
        px, py = np.interp(O[:, 0], tp, P["map_x"]), np.interp(O[:, 0], tp, P["map_y"])
        yaw = np.interp(O[:, 0], tp, np.unwrap(P["map_yaw"]))
    path = dict(t=O[:, 0], v=O[:, 1], x=px, y=py, yaw=yaw, s=np.cumsum(np.abs(O[:, 1]) * dt))

    N, FS = 180, 100.0
    fr = np.fft.rfftfreq(N, 1 / FS); W = np.hanning(N); R = []
    for t in np.arange(2.0, O[-1, 0], 0.25):
        x = F[F[:, 0] <= t, 1][-N:]
        if len(x) < N:
            continue
        P = np.abs(np.fft.rfft((x - x.mean()) * W)) ** 2
        v = np.mean(np.abs(O[(O[:, 0] > t - 1.8) & (O[:, 0] <= t), 1]))
        R.append((t, P[(fr >= 8) & (fr < 50)].sum() / P[(fr >= 0.5) & (fr < 8)].sum(), 0.25 <= v <= 0.50))
    # jolt: vertical free acceleration minus its 0.5 s running median
    jolt = np.abs(F[:, 1] - ndimage.median_filter(F[:, 1], size=51, mode="nearest"))
    return path, np.array(R), np.stack([F[:, 0], jolt], 1)


def pose_at(path, t):
    return (np.interp(t, path["t"], path["x"]), np.interp(t, path["t"], path["y"]), np.interp(t, path["t"], path["yaw"]))


def to_world(path, t, p_base):
    x, y, a = pose_at(path, t)
    return x + np.cos(a) * p_base[..., 0] - np.sin(a) * p_base[..., 1], y + np.sin(a) * p_base[..., 0] + np.cos(a) * p_base[..., 1]


def straight_lines(mask):
    """Long straight runs in a crack mask: tile grout and painted lines, not cracks."""
    # 100 px: tile keeps 3-5 % of its "cracks", the asphalt cracks of 1548/1618/1628 keep 97-100 %
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=60, minLineLength=100, maxLineGap=8)
    out = np.zeros_like(mask)
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)), 255, 11)
    return out > 0


def camera_pass(bag, t_start, path, T_bd, T_bc, seg):
    mask_ok = cv2.imread(os.path.join(os.path.dirname(gp.__file__), "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0
    T_cd = np.linalg.inv(T_bc) @ T_bd           # colour <- depth
    r = open_bag(bag, [DEPTH, DINFO, COLOR, CINFO])
    Kd = Kc = prev = col = None
    rng, n, last_crack = np.random.default_rng(0), 0, -9.0
    dets, unev, cracks, planes = [], [], {}, []
    stats = dict(frames=0, skipped=0, crack_frames=0, crack_px=0, crack_kept=0)
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == DINFO and Kd is None:
            m = deserialize_message(data, CameraInfo); Kd = (m.k[0], m.k[4], m.k[2], m.k[5])
        if tp == CINFO and Kc is None:
            m = deserialize_message(data, CameraInfo); Kc = (m.k[0], m.k[4], m.k[2], m.k[5])
        if tp == COLOR:
            col = data
        if tp != DEPTH or Kd is None:
            continue
        n += 1
        if n % 3:
            continue
        m = deserialize_message(data, Image)
        depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
        cloud = gp.deproject(depth, *Kd)
        valid = (depth > 0.30) & (depth < 1.60) & mask_ok
        pts = cloud[valid]
        if len(pts) < 500:
            continue
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = (gp.refine_plane(sample, *prev) if prev else None) or gp.fit_ground_plane(sample, rng=rng)
        if fit is None:
            prev = None; continue
        nrm, d, inl = fit; prev = (nrm, d); stats["frames"] += 1
        planes.append((t, *nrm, d, inl))
        resid = ((cloud @ nrm) + d).astype(np.float32); resid[~valid] = 0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool); resid[~valid] = 0
        flat = valid & (np.abs(resid) < THRESH); flat[GRIP_ROW:] = False
        u_mm = float(np.sqrt(np.mean(resid[flat] ** 2)) * 1000) if flat.any() else 0.0
        if flat.any():
            unev.append((t, u_mm))
        if inl < MIN_INLIERS:
            stats["skipped"] += 1
            continue
        # on rough ground the fixed 15 mm boxes tufts, chips and leaves: follow the surface
        thresh = max(THRESH, K_ROUGH * u_mm / 1000.0)
        for h in gp.find_hazards(cloud, valid, resid, nrm, d, thresh, MIN_AREA, Kd[0], Kd[1], max_height=MAX_HEIGHT):
            if h["peak_mm"] > MAX_HEIGHT * 1000:
                continue
            pb = (T_bd @ np.r_[h["centroid"], 1.0])[:3]
            wx, wy = to_world(path, t, pb)
            dets.append(dict(t=t, type=h["type"], x=float(wx), y=float(wy), size_mm=h["size_mm"], length_mm=h["length_mm"],
                             width_mm=h["width_mm"], range_m=h["range_m"], centroid=h["centroid"]))

        # cracks, 3 times a second, on the latest colour frame
        if seg is None or col is None or Kc is None or t - last_crack < 0.33:
            continue
        last_crack = t
        c = deserialize_message(col, Image)
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)
        _, cmask = seg.predict(img, bgr=(c.encoding == "bgr8"), ground_mask=seg.ground)
        cmask[straight_lines(cmask)] = 0
        stats["crack_frames"] += 1
        # depth pixels on flat ground, into the colour image
        gy, gx = np.nonzero(flat[::2, ::2]); gy, gx = gy * 2, gx * 2
        P = cloud[gy, gx]
        Pc = P @ T_cd[:3, :3].T + T_cd[:3, 3]
        u = np.round(Kc[0] * Pc[:, 0] / Pc[:, 2] + Kc[2]).astype(int); v = np.round(Kc[1] * Pc[:, 1] / Pc[:, 2] + Kc[3]).astype(int)
        inside = (u >= 0) & (u < c.width) & (v >= 0) & (v < c.height)
        hit = np.zeros(len(P), bool); hit[inside] = cmask[v[inside], u[inside]] > 0
        # pixels on a height step (kerb, door edge) are edges, not cracks
        gx_, gy_ = cv2.Sobel(resid, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(resid, cv2.CV_32F, 0, 1, ksize=3)
        step = np.hypot(gx_, gy_)[gy, gx] > 0.02
        stats["crack_px"] += int(hit.sum()); hit &= ~step; stats["crack_kept"] += int(hit.sum())
        if hit.any():
            pb = P[hit] @ T_bd[:3, :3].T + T_bd[:3, 3]
            wx, wy = to_world(path, t, pb)
            for key in set(zip(np.floor(wx / CELL).astype(int), np.floor(wy / CELL).astype(int))):
                cracks.setdefault(key, set()).add(n)
    return dets, np.array(unev), cracks, stats, np.array(planes)


def merge(dets):
    """docs/schema.md: same type closer than 0.3 m is one hazard; position averaged, size from the closest sighting."""
    out = []
    for d in sorted(dets, key=lambda d: d["t"]):
        for h in out:
            if h["type"] == d["type"] and np.hypot(h["x"] - d["x"], h["y"] - d["y"]) < MERGE_M:
                h["xs"].append(d["x"]); h["ys"].append(d["y"]); h["x"], h["y"] = np.mean(h["xs"]), np.mean(h["ys"])
                h["seen"] += 1; h["dets"].append(d)
                if d["range_m"] < h["best"]["range_m"]:
                    h["best"] = d
                break
        else:
            out.append(dict(type=d["type"], x=d["x"], y=d["y"], xs=[d["x"]], ys=[d["y"]], seen=1, best=d, dets=[d], t_first=d["t"]))
    return [h for h in out if h["seen"] >= MIN_SEEN]


def crack_hazards(cracks, min_frames=3, min_cells=4):
    keep = {k: v for k, v in cracks.items() if len(v) >= min_frames}
    if not keep:
        return []
    ks = np.array(list(keep)); x0, y0 = ks.min(0)
    grid = np.zeros(tuple(ks.max(0) - ks.min(0) + 1), np.uint8)
    grid[ks[:, 0] - x0, ks[:, 1] - y0] = 1
    lab, n = ndimage.label(grid, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        cells = np.argwhere(lab == i)
        if len(cells) < min_cells:
            continue
        xy = (cells + [x0, y0] + 0.5) * CELL
        frames = set().union(*(keep[(a + x0, b + y0)] for a, b in cells))
        ev = np.sort(np.linalg.eigvalsh(np.cov(xy.T))) if len(xy) > 2 else np.array([0, 0])
        out.append(dict(type="crack", x=float(xy[:, 0].mean()), y=float(xy[:, 1].mean()), seen=len(frames),
                        length_mm=float(4 * np.sqrt(max(ev[-1], 0)) * 1000 + CELL * 1000),
                        width_mm=float(4 * np.sqrt(max(ev[0], 0)) * 1000 + CELL * 1000), cells=xy))
    return out


def wheel_crossings(path, h):
    """Times at which each wheel passes over the hazard (a disc of half its length, at least 10 cm)."""
    r = min(max(h.get("best", h).get("length_mm", 100) / 2000.0, 0.10), 0.6) + TYRE_HALF
    times = []
    for wx, wy in WHEELS:
        X, Y = to_world(path, path["t"], np.array([[wx, wy]] * len(path["t"])))
        dist = np.hypot(X - h["x"], Y - h["y"])
        i = int(np.argmin(dist))
        if dist[i] < r:
            times.append(float(path["t"][i]))
    return sorted(times)


def jolt_at(J, t, half=0.6):
    """Largest jolt within +-half s of t (the position is only good to a few cm), and the
    shaking 1-5 s away (75th and 95th percentile), which leaves the hazard's own jolts out."""
    w = (J[:, 0] >= t - half) & (J[:, 0] <= t + half)
    around = (np.abs(J[:, 0] - t) > 1.0) & (np.abs(J[:, 0] - t) < 5.0)
    if not w.any() or around.sum() < 50:
        return None, None, None
    i = np.flatnonzero(w)[np.argmax(J[w, 1])]
    return float(J[i, 1]), np.percentile(J[around, 1], [75, 95]), float(J[i, 0])


def felt(peak, base, k=1.0):
    """The jolt stands out from the shaking around it, both its middle and its high end.
    Calibrated on corridor_groundtruth_1447 at 0.2 m/s: plain tile up to 0.65 m/s^2, 20 mm foam
    0.63-0.69 (never felt, foam squashes), 15 mm wood 0.81, 50 mm block 1.1-1.9."""
    return peak is not None and peak > k * max(2.0 * base[0], 1.3 * base[1], 0.75)


COLOR_MASK = os.path.join(REPO, "src", "crack_detection", "models", "robot_mask_color.png")   # the crack node's


def crack_segmenter():
    """The crack U-Net and the crack node's colour robot mask (white = ground)."""
    from crack_onnx import CrackSegmenter, load_ground_mask
    for d in (("src", "crack_detection", "models"), ("vision_training", "models")):
        path = os.path.join(REPO, *d, "crack_unet16_v1.onnx")
        if os.path.exists(path):
            seg = CrackSegmenter(path, meta_path=os.path.join(REPO, *d, "crack_unet16_v1.json"), num_threads=4)
            seg.ground = load_ground_mask(COLOR_MASK, 640, 480)
            return seg
    raise SystemExit("crack_unet16_v1.onnx not found: download it from the crack-model-v1 release (docs/crack_model.md)")


def build(bag, use_cracks=True, poses=None):
    """Everything up to the classified hazards and the 0.25 m records, for this script and inspection_map.py."""
    bag = bag.rstrip("/"); name = os.path.basename(bag)
    t_start = open_bag(bag, []).get_metadata().starting_time.nanoseconds * 1e-9
    T_bd, T_bc = camera_transforms(bag)
    path, R, J = motion(bag, t_start, poses)
    seg = crack_segmenter() if use_cracks else None
    dets, U, cracks, stats, planes = camera_pass(bag, t_start, path, T_bd, T_bc, seg)

    hazards = merge(dets) + crack_hazards(cracks)
    explained = []
    for h in hazards:
        if h["type"] == "crack":      # a few mm wide: camera only, the wheels cannot feel it
            h["crossing_t"], h["imu_peak"], h["crossing_speed"], h["class"] = None, None, None, "A"
            continue
        crossings = wheel_crossings(path, h)
        h["crossing_t"], h["imu_peak"], h["jolt_t"] = (crossings[0] if crossings else None), None, None
        for tc in crossings:
            peak, base, tj = jolt_at(J, tc)
            if felt(peak, base) and (h["imu_peak"] is None or peak > h["imu_peak"]):
                h["imu_peak"], h["crossing_t"], h["jolt_t"] = peak, tc, tj
        h["crossing_speed"] = float(abs(np.interp(h["crossing_t"], path["t"], path["v"]))) if crossings else None
    # one jolt is one hazard: when a wheel crosses several (tufts), the tallest one takes it
    felt_h = sorted((h for h in hazards if h.get("jolt_t") is not None), key=lambda h: -h["best"]["size_mm"])
    for h in hazards:
        h.setdefault("class", "A")
    for h in felt_h:
        if any(abs(h["jolt_t"] - e) < 0.3 for e in explained):
            h["imu_peak"] = None
            continue
        h["class"] = "C"; explained.append(h["jolt_t"])
    # B: jolts that stand out while no seen hazard is under a wheel
    moving = np.interp(J[:, 0], path["t"], np.abs(path["v"])) > 0.05
    cand = (J[:, 1] > 0.5) & moving & (J[:, 1] == ndimage.maximum_filter1d(J[:, 1], 41))
    for t in J[cand, 0]:
        if any(abs(t - e) < 0.8 for e in explained):
            continue
        peak, base, _ = jolt_at(J, t, half=0.05)
        if not felt(peak, base, k=1.25):
            continue
        # a wheel on a seen bump or hole at that moment: that hazard was felt, a little off the predicted time
        wheels = [to_world(path, t, np.array(w)) for w in WHEELS]
        near = [h for h in hazards if h["type"] != "crack" and h["class"] != "B" and any(
            np.hypot(wx - h["x"], wy - h["y"]) < min(max(h["best"]["length_mm"] / 2000.0, 0.10), 0.6) + TYRE_HALF + 0.15
            for wx, wy in wheels)]
        if near:      # the other axle on a hazard already felt, or a crossing a little off the predicted time
            h = max(near, key=lambda h: h["best"]["size_mm"])
            if h["imu_peak"] is None or peak > h["imu_peak"]:
                h["imu_peak"], h["crossing_t"], h["class"] = peak, float(t), "C"
                h["crossing_speed"] = float(abs(np.interp(t, path["t"], path["v"])))
            explained.append(t)
            continue
        fx, fy = to_world(path, t, np.array([0.249, 0.0]))
        if any(h["class"] == "B" and np.hypot(h["x"] - fx, h["y"] - fy) < MERGE_M for h in hazards):
            continue
        hazards.append(dict(type="bump", x=float(fx), y=float(fy), seen=0, t_first=float(t), crossing_t=float(t), imu_peak=peak,
                            crossing_speed=float(abs(np.interp(t, path["t"], path["v"]))), **{"class": "B"}))
        explained.append(t)

    hazards.sort(key=lambda h: h.get("t_first", h.get("crossing_t") or 0) if h["type"] != "crack" else 1e9 + h["x"])
    for i, h in enumerate(hazards, 1):
        h["id"] = f"H{i}"
    recs = records(name, t_start, path, R, U, hazards)
    return dict(bag=bag, name=name, t_start=t_start, T_bd=T_bd, T_bc=T_bc, path=path, R=R, J=J, U=U,
                hazards=hazards, cracks=cracks, planes=planes, stats=stats, recs=recs, seg=seg)


def hazard_json(h):
    b = h.get("best", {})
    return {"id": h["id"], "type": h["type"], "x": round(h["x"], 2), "y": round(h["y"], 2),
            "length_mm": round(b.get("length_mm", h.get("length_mm", 0))) if h["class"] != "B" else None,
            "width_mm": round(b.get("width_mm", h.get("width_mm", 0))) if h["class"] != "B" else None,
            "height_mm": round(b["size_mm"]) if "size_mm" in b else None,
            "imu_peak": round(h["imu_peak"], 2) if h["imu_peak"] is not None else None,
            "crossing_speed": round(h["crossing_speed"], 2) if h["crossing_speed"] is not None else None,
            "class": h["class"], "seen": h["seen"], "crop": h.get("crop")}


def records(name, t_start, path, R, U, hazards):
    """One record per 0.25 m driven."""
    segs = SURFACES.get(name, [])
    recs, marks = [], np.arange(0.25, path["s"][-1], 0.25)
    prev_t = 0.0
    for s in marks:
        t = float(np.interp(s, path["s"], path["t"]))
        x, y, _ = pose_at(path, t)
        w = (U[:, 0] > prev_t) & (U[:, 0] <= t) if len(U) else np.zeros(0, bool)
        k = np.searchsorted(R[:, 0], t) - 1
        recs.append({"t": round(t_start + t, 2), "x": round(float(x), 2), "y": round(float(y), 2),
                     "speed": round(float(abs(np.interp(t, path["t"], path["v"]))), 2),
                     "imu_roughness": round(float(R[k, 1]), 3) if k >= 0 and R[k, 2] else None,
                     "camera_unevenness_mm": round(float(np.median(U[w, 1])), 2) if w.any() else None,
                     "terrain_camera": None, "terrain_imu": None,
                     "surface_log": next((sname for sname, s0, s1 in segs if s0 <= t <= s1), None),
                     "hazards": [h["id"] for h in hazards if prev_t < (h.get("t_first") or -1) <= t]})
        prev_t = t
    return recs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("out"); ap.add_argument("--no-cracks", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ctx = build(a.bag, not a.no_cracks)
    name, path, hazards, recs, stats, seg = ctx["name"], ctx["path"], ctx["hazards"], ctx["recs"], ctx["stats"], ctx["seg"]
    out = [hazard_json(h) for h in hazards]
    json.dump(out, open(os.path.join(a.out, "hazards.json"), "w"), indent=1)

    with open(os.path.join(a.out, "records.jsonl"), "w") as f:
        f.writelines(json.dumps(r) + "\n" for r in recs)

    draw_map(name, path, recs, hazards, os.path.join(a.out, "map.png"))
    cnt = lambda ty, cl: sum(h["type"] == ty and h["class"] == cl for h in hazards)
    print(f"{name}: {path['s'][-1]:.1f} m driven, {stats['frames']} depth frames ({stats['skipped']} skipped, ground not one plane), "
          f"{len(recs)} records")
    for ty in ("bump", "hole", "crack"):
        print(f"  {ty:5s}  A {cnt(ty, 'A'):3d}  B {cnt(ty, 'B'):3d}  C {cnt(ty, 'C'):3d}")
    if seg is not None:
        print(f"  crack pixels on flat ground {stats['crack_px']}, kept after the height-step filter {stats['crack_kept']} "
              f"({stats['crack_frames']} colour frames)")


def draw_map(name, path, recs, hazards, out_png):
    span = np.ptp(path["x"]) + 1.0, np.ptp(path["y"]) + 1.0
    fig, ax = plt.subplots(figsize=(10, float(np.clip(7.0 * span[1] / span[0], 3.2, 8.0))))
    ax.set_facecolor("#fcfcfb"); fig.patch.set_facecolor("#fcfcfb")
    # path, coloured by camera unevenness (one neutral hue, light = smooth)
    cmap = LinearSegmentedColormap.from_list("unev", ["#d6d6d2", "#2b2b29"])
    pts = np.array([[r["x"], r["y"]] for r in recs]); val = np.array([r["camera_unevenness_mm"] or np.nan for r in recs])
    seg = np.stack([pts[:-1], pts[1:]], 1)
    lc = LineCollection(seg, cmap=cmap, norm=plt.Normalize(3, 9), linewidths=7, capstyle="round", zorder=1)
    lc.set_array(np.nan_to_num(val[1:], nan=3.0)); ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, orientation="horizontal", fraction=0.05, pad=0.14, aspect=40, shrink=0.6)
    cb.set_label("camera unevenness along the path, mm (light = smooth)", color=INK)
    for h in hazards:
        if h["type"] == "crack":
            ax.scatter(h["cells"][:, 0], h["cells"][:, 1], s=7, marker="s", color=C_CRACK, linewidths=0, zorder=2)
    def rough_at(h):
        i = int(np.argmin(np.hypot(pts[:, 0] - h["x"], pts[:, 1] - h["y"])))
        return np.nan_to_num(val[i]) > ROUGH_MM
    big = lambda h: h["class"] != "B" and (h["best"]["size_mm"] >= LARGE_MM or not rough_at(h))
    for h in hazards:
        if h["type"] == "crack":
            continue
        colr = C_BUMP if h["type"] == "bump" else C_HOLE
        mk = "^" if h["type"] == "bump" else "v"
        if h["class"] == "B":
            ax.scatter(h["x"], h["y"], s=90, marker="s", facecolor="#fcfcfb", edgecolor=INK, linewidths=2, zorder=4)
        elif big(h):
            ax.scatter(h["x"], h["y"], s=130, marker=mk, color=colr if h["class"] == "C" else "#fcfcfb",
                       edgecolor=colr, linewidths=2, zorder=5)
            if h["class"] == "C" or h["best"]["size_mm"] >= 25:
                ax.annotate(f'{h["id"]} {h["best"]["size_mm"]:.0f} mm', (h["x"], h["y"]), xytext=(8, 4),
                            textcoords="offset points", fontsize=7.5, color=INK, zorder=6)
        else:     # under 30 mm on rough ground: mostly tufts, chips and leaves
            ax.scatter(h["x"], h["y"], s=28, marker=mk, color=colr, alpha=0.35, linewidths=0, zorder=3)
    ax.plot(path["x"][0], path["y"][0], "o", color=INK, ms=7, zorder=5)
    ax.annotate("start", (path["x"][0], path["y"][0]), xytext=(8, -12), textcoords="offset points", fontsize=9, color=INK)
    ax.set_aspect("equal"); ax.autoscale_view(); ax.margins(0.08)
    ax.grid(color="#e4e4e0", lw=0.8); ax.set_axisbelow(True)
    for sp in ax.spines.values():
        sp.set_color("#c9c9c4")
    ax.tick_params(colors=MUTED); ax.set_xlabel("m", color=MUTED); ax.set_ylabel("m", color=MUTED)
    n = lambda ty, cl: sum(h["type"] == ty and h["class"] == cl and big(h) for h in hazards)
    small = sum(h["type"] in ("bump", "hole") and h["class"] != "B" and not big(h) for h in hazards)
    handles = [
        plt.Line2D([], [], ls="", marker="^", ms=10, color=C_BUMP, label=f"bump, seen and felt (C): {n('bump', 'C')}"),
        plt.Line2D([], [], ls="", marker="^", ms=10, mfc="#fcfcfb", mec=C_BUMP, mew=2, label=f"bump, seen only (A): {n('bump', 'A')}"),
        plt.Line2D([], [], ls="", marker="v", ms=10, color=C_HOLE, label=f"hole, seen and felt (C): {n('hole', 'C')}"),
        plt.Line2D([], [], ls="", marker="v", ms=10, mfc="#fcfcfb", mec=C_HOLE, mew=2, label=f"hole, seen only (A): {n('hole', 'A')}"),
        plt.Line2D([], [], ls="", marker="s", ms=9, mfc="#fcfcfb", mec=INK, mew=2, label=f"felt only (B): {sum(h['class'] == 'B' for h in hazards)}"),
        plt.Line2D([], [], ls="", marker="s", ms=6, color=C_CRACK, label=f"crack (camera): {sum(h['type'] == 'crack' for h in hazards)}"),
        plt.Line2D([], [], ls="", marker="^", ms=6, color=C_BUMP, alpha=0.35,
                   label=f"under {LARGE_MM} mm on rough ground: {small}"),
    ]
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), frameon=False, fontsize=9, labelcolor=INK)
    ax.set_title(f"{name}: hazard map from one run (wheel speed + IMU heading)", color=INK, fontsize=11, loc="left")
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
