#!/usr/bin/env python3
"""Validate the measurement pipeline against simulated ground truth.

This proves the geometry is correct: that the recovered camera height, camera
tilt and object size mean what they claim to mean. It does NOT tell you the
real noise limits, because the simulated noise is independent per pixel while
a real D435's is spatially correlated. Measure those with the camera.

Run:  python3 selftest.py
"""
import cv2
import numpy as np

from ground_probe import (SyntheticSource, deproject, find_hazards,
                          fit_ground_plane, plane_pose)

ZMIN, NFRAMES = 0.30, 12


def measure(height, tilt, box_mm, thresh_mm=15.0, zmax=1.60, n=NFRAMES):
    src = SyntheticSource(height=height, tilt_deg=tilt, box_mm=abs(box_mm))
    fx, fy, ppx, ppy = src.intrinsics()
    rng = np.random.default_rng(3)
    hs, ts, bs, hits, fps = [], [], [], 0, 0
    for _ in range(n):
        depth, _ = src.read()
        cloud = deproject(depth, fx, fy, ppx, ppy)
        valid = (depth > ZMIN) & (depth < zmax)
        sub = cloud[valid]
        if len(sub) < 500:
            continue
        fit = fit_ground_plane(sub[rng.integers(0, len(sub), 6000)], rng=rng)
        if fit is None:
            continue
        nrm, d, _ = fit
        resid = (cloud @ nrm + d).astype(np.float32)
        resid[~valid] = 0.0
        resid = cv2.medianBlur(resid, 5)
        valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        resid[~valid] = 0.0
        h_est, t_est = plane_pose(nrm, d)
        hs.append(h_est)
        ts.append(t_est)
        hz = find_hazards(cloud, valid, resid, nrm, d, thresh_mm / 1000.0, 25.0, fx, fy)
        real = [x for x in hz if abs(x["size_mm"] - abs(box_mm)) < 0.5 * abs(box_mm) + 8]
        fps += len(hz) - len(real)
        if real:
            hits += 1
            bs.append(max(x["size_mm"] for x in real))
    return (np.mean(hs) if hs else np.nan, np.mean(ts) if ts else np.nan,
            np.mean(bs) if bs else np.nan, hits / n, fps / n)


def pose_sweep():
    print("\nGEOMETRY: does the recovered pose and size match the truth?\n")
    print(f"{'truth h':>8}{'tilt':>6}{'object':>8} | {'h err':>8}{'tilt err':>10}"
          f"{'size err':>10} | {'found':>7}")
    print("-" * 66)
    for h, t, b in [(0.75, 40, 25), (0.75, 40, 50), (0.75, 35, 25), (0.75, 50, 25),
                    (0.60, 40, 25), (0.90, 40, 25), (0.75, 40, -30)]:
        he, te, be, hit, _ = measure(h, t, b)
        print(f"{h:8.2f}{t:6.0f}{abs(b):6.0f}mm{'' if b > 0 else ' '} | "
              f"{(he - h) * 1000:6.1f}mm{te - t:9.2f}d{be - abs(b):9.1f}mm | {hit * 100:6.0f}%")


def floor_sweep():
    print("\nDETECTION FLOOR: smallest object found, per threshold\n")
    sizes = (8, 10, 15, 20, 25, 40)
    print(f"{'threshold':>10} | " + "".join(f"{s:>6}mm" for s in sizes))
    print("-" * 60)
    for th in (6, 8, 10, 15, 20):
        cells = []
        for s in sizes:
            _, _, _, hit, _ = measure(0.75, 40, s, thresh_mm=th)
            cells.append(f"{hit * 100:5.0f}%")
        print(f"{th:8.0f}mm | " + "".join(f"{c:>8}" for c in cells))
    print("\nAn object is found when it stands off the plane by more than the")
    print("threshold. Nothing subtler than the threshold is visible, by design.")


def noise_floor():
    print("\nNOISE FLOOR: false hazards per frame on perfectly flat ground\n")
    print(f"{'threshold':>10} | {'0.3-1.6 m band':>16} | {'0.3-1.2 m band':>16}")
    print("-" * 50)
    for th in (5, 8, 12, 15, 20):
        _, _, _, _, a = measure(0.75, 40, 0.0, thresh_mm=th, zmax=1.60)
        _, _, _, _, b = measure(0.75, 40, 0.0, thresh_mm=th, zmax=1.20)
        print(f"{th:8.0f}mm | {a:15.2f}  | {b:15.2f}")
    print("\nZero throughout, because the 25 cm2 minimum blob area rejects")
    print("independent pixel noise. Real depth noise is spatially correlated,")
    print("so treat this as a lower bound and measure the real rate on the camera.")


if __name__ == "__main__":
    pose_sweep()
    floor_sweep()
    noise_floor()
