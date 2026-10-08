#!/usr/bin/env python3
"""End-to-end check of the repository's find_hazards (new height) with the robot mask, on the ground-truth bags."""
import sys, os, json, importlib, numpy as np, cv2
sys.path.insert(0, os.path.expanduser("~/workspaces/silvanus-terrain-quality-mapping/ground_demo"))
import ground_probe as gp
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import Image, CameraInfo
MASK_OK = cv2.imread(os.path.expanduser("~/workspaces/silvanus-terrain-quality-mapping/ground_demo/robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0
for bag in sys.argv[1:]:
    r = SequentialReader(); r.open(StorageOptions(uri=f"/home/praveen/bags/{bag}", storage_id="mcap"), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=["/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"]))
    K, prev, rng, n, t0, dets = None, None, np.random.default_rng(0), 0, None, []
    while r.has_next():
        topic, data, t = r.read_next(); t0 = t0 or t * 1e-9
        if topic.endswith("camera_info"):
            if K is None: m = deserialize_message(data, CameraInfo); K = (m.k[0], m.k[4], m.k[2], m.k[5])
            continue
        if K is None: continue
        n += 1
        if n % 3: continue
        m = deserialize_message(data, Image)
        depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
        cloud = gp.deproject(depth, *K); valid = (depth > 0.30) & (depth < 1.60) & MASK_OK; pts = cloud[valid]
        if len(pts) < 500: continue
        sample = pts[rng.integers(0, len(pts), size=min(6000, len(pts)))]
        fit = gp.refine_plane(sample, *prev) if prev else None
        if fit is None: fit = gp.fit_ground_plane(sample, rng=rng)
        if fit is None: prev = None; continue
        nrm, d, _ = fit; prev = (nrm, d)
        resid = ((cloud @ nrm) + d).astype(np.float32); resid[~valid] = 0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool); resid[~valid] = 0
        for h in gp.find_hazards(cloud, valid, resid, nrm, d, 0.015, 25.0, K[0], K[1]):
            if h["peak_mm"] > 300: continue
            x, y, w, hh = h["bbox"]
            dets.append(dict(t=t * 1e-9 - t0, type=h["type"], cx=x + w/2, y=y, h=hh, L_old=h["length_mm"], W_old=h["width_mm"],
                             old_p90=h["peak_mm"], g_med=h["size_mm"]))
    json.dump({"mask": dets}, open(f"{bag}_repo.json", "w")); print(bag, len(dets))
