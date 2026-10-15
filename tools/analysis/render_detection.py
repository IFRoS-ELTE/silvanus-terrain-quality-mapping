#!/usr/bin/env python3
"""Render the depth hazard detection from a bag as a full-resolution video and a still per object.

    render_detection.py BAG T_FROM T_TO OUT_PREFIX

Runs ground_probe.find_hazards (robot mask on) on every 2nd depth frame between
T_FROM and T_TO seconds. Each detected bump is boxed on the colour image and on
the height map. A bump that matches a tape-measured object (groundtruth_objects.py)
is labelled with its name, measured height and tape height.

Writes OUT_PREFIX.mp4 (real time) and OUT_PREFIX_objects.png: each object's median height
over the frames where it is fully in view, shown on a typical frame.
"""
import os, sys, subprocess
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ground_demo"))
import ground_probe as gp
from groundtruth_objects import T, TR
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import Image, CameraInfo

DEPTH, DINFO = "/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"
COLOR, CINFO = "/camera/camera/color/image_raw", "/camera/camera/color/camera_info"
DEPTH_TO_COLOR_X = 0.015   # colour optical frame origin is 15 mm to the depth frame's left (tf_static)
SPAN = 0.04                # height map colour range, +-40 mm
THRESH = 0.015
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE, BLACK, GREEN = (255, 255, 255), (0, 0, 0), (60, 200, 60)

bag, t_from, t_to, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
name = os.path.basename(bag.rstrip("/"))
tracks = TR.get(name, [])
mask_ok = cv2.imread(os.path.join(os.path.dirname(gp.__file__), "robot_mask_d435_gripper.png"), cv2.IMREAD_GRAYSCALE) > 0


def label(img, text, org, scale, col, bg=BLACK, thick=2):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thick)
    x, y = max(0, min(org[0], img.shape[1] - w - 4)), max(h + 4, min(org[1], img.shape[0] - base - 2))
    cv2.rectangle(img, (x - 3, y - h - 4), (x + w + 3, y + base + 2), bg, -1)
    cv2.putText(img, text, (x, y), FONT, scale, col, thick, cv2.LINE_AA)


def match(h, t):
    x, y, w, hh = h["bbox"]
    d = dict(cx=x + w / 2, L_old=h["length_mm"], W_old=h["width_mm"], old_p90=h["peak_mm"])
    for obj, ta, tb, sel in tracks:
        if ta <= t <= tb and sel(d):
            return obj
    return None


def color_box(cloud, valid, resid, h, Kc):
    """Bounding box of the hazard's points projected into the colour image."""
    x, y, w, hh = h["bbox"]
    sgn = 1.0 if h["type"] == "bump" else -1.0
    sel = valid[y:y + hh, x:x + w] & (sgn * resid[y:y + hh, x:x + w] > 0.5 * THRESH)
    p = cloud[y:y + hh, x:x + w][sel]
    if len(p) < 20:
        return None
    X, Y, Z = p[:, 0] + DEPTH_TO_COLOR_X, p[:, 1], p[:, 2]
    u, v = Kc[0] * X / Z + Kc[2], Kc[1] * Y / Z + Kc[3]
    u0, u1 = np.percentile(u, [2, 98]).astype(int)
    v0, v1 = np.percentile(v, [2, 98]).astype(int)
    return u0, v0, u1 - u0, v1 - v0


def legend_bar(width):
    bar = np.full((70, width, 3), 25, np.uint8)
    ramp = gp.LUT[np.linspace(0, 255, 256).astype(np.uint8)]
    ramp = cv2.resize(ramp[None, :, :], (300, 16), interpolation=cv2.INTER_NEAREST)
    x0 = width - 300 - 90
    bar[40:56, x0:x0 + 300] = ramp
    for txt, xx, c in (("-40 mm", x0 - 75, WHITE), ("0", x0 + 145, BLACK), ("+40 mm", x0 + 306, WHITE)):
        cv2.putText(bar, txt, (xx, 54), FONT, 0.55, c, 1, cv2.LINE_AA)
    tape = "Tape-measured heights:  " + "   ".join(f"{k} {v[2]} mm" for k, v in T.items())
    cv2.putText(bar, tape, (12, 25), FONT, 0.6, WHITE, 1, cv2.LINE_AA)
    cv2.putText(bar, "Green = known object: measured height (tape height).   White = other detection.",
                (12, 54), FONT, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return bar


r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
r.set_filter(StorageFilter(topics=[DEPTH, DINFO, COLOR, CINFO]))
Kd = Kc = prev = col = t0 = ff = None
rng, n, cands = np.random.default_rng(0), 0, {}
while r.has_next():
    topic, data, stamp = r.read_next()
    if topic in (DEPTH, DINFO) and t0 is None:
        t0 = stamp * 1e-9
    if topic == DINFO and Kd is None:
        m = deserialize_message(data, CameraInfo); Kd = (m.k[0], m.k[4], m.k[2], m.k[5])
    if topic == CINFO and Kc is None:
        m = deserialize_message(data, CameraInfo); Kc = (m.k[0], m.k[4], m.k[2], m.k[5])
    if topic == COLOR:
        col = data
    if topic != DEPTH or Kd is None or Kc is None or col is None:
        continue
    t = stamp * 1e-9 - t0
    if t > t_to:
        break
    n += 1
    if n % 2 or t < t_from:
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
    nrm, d, _ = fit; prev = (nrm, d)
    resid = ((cloud @ nrm) + d).astype(np.float32); resid[~valid] = 0
    resid = cv2.medianBlur(resid, 5)
    valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool); resid[~valid] = 0
    hz = [h for h in gp.find_hazards(cloud, valid, resid, nrm, d, THRESH, 25.0, Kd[0], Kd[1]) if h["peak_mm"] <= 300]

    hmap = gp.residual_view(resid, valid, SPAN).copy()
    c = deserialize_message(col, Image)
    cimg = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)[:, :, ::-1].copy()
    clean = cimg.copy()
    for h in hz:
        obj = match(h, t) if h["type"] == "bump" else None
        x, y, w, hh = h["bbox"]
        txt = f"{obj}: {h['size_mm']:.0f} mm (tape {T[obj][2]})" if obj else f"{h['type']} {h['size_mm']:.0f} mm"
        fg = GREEN if obj else WHITE
        cv2.rectangle(hmap, (x, y), (x + w, y + hh), BLACK, 5)
        cv2.rectangle(hmap, (x, y), (x + w, y + hh), fg, 2)
        label(hmap, f"{h['size_mm']:.0f} mm", (x, y - 8 if y > 30 else y + hh + 28), 0.8, fg)
        cb = color_box(cloud, valid, resid, h, Kc)
        if cb:
            u, v, w2, h2 = cb
            cv2.rectangle(cimg, (u, v), (u + w2, v + h2), BLACK, 5)
            cv2.rectangle(cimg, (u, v), (u + w2, v + h2), fg, 2)
            label(cimg, txt, (u, v - 8 if v > 30 else v + h2 + 28), 0.7 if obj else 0.6, fg)
        # candidate stills: object fully in view top to bottom, in both images
        if obj and cb and y > 3 and y + hh < 340 and cb[1] >= 0 and cb[1] + cb[3] <= cimg.shape[0]:
            jpg = lambda im: cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 92])[1]
            cands.setdefault(obj, []).append((h["size_mm"], abs(cb[1] + cb[3] / 2 - cimg.shape[0] / 2), t, h["range_m"],
                                              jpg(clean), cb, jpg(gp.residual_view(resid, valid, SPAN)), h["bbox"]))

    head = np.full((44, cimg.shape[1] + hmap.shape[1], 3), 25, np.uint8)
    cv2.putText(head, f"Robot camera   ({name}, t = {t:4.1f} s)", (12, 30), FONT, 0.75, WHITE, 2, cv2.LINE_AA)
    cv2.putText(head, "Height above the fitted floor  (red up, blue down, black no data)",
                (cimg.shape[1] + 12, 30), FONT, 0.75, WHITE, 2, cv2.LINE_AA)
    frame = np.vstack([head, np.hstack([cimg, hmap]), legend_bar(cimg.shape[1] + hmap.shape[1])])
    if ff is None:
        fh, fw = frame.shape[:2]
        ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{fw}x{fh}",
                               "-r", "15", "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "18",
                               "-pix_fmt", "yuv420p", "-movflags", "+faststart", out + ".mp4"], stdin=subprocess.PIPE)
    ff.stdin.write(frame.tobytes())
ff.stdin.close(); ff.wait()

# one row per object: the frame whose height is closest to the median over all its frames,
# and of those the one where the object is most central in the colour image
rows, summary = [], []
for obj in [k for k in T if k in cands]:
    C = cands[obj]
    med = float(np.median([c[0] for c in C]))
    size, _, t, rng_m, cj, (u, v, w2, h2), hj, (x, y, w, hh) = min(C, key=lambda c: (round(abs(c[0] - med)), c[1]))
    cimg, hmap = cv2.imdecode(cj, cv2.IMREAD_COLOR), cv2.imdecode(hj, cv2.IMREAD_COLOR)
    for im, (a, b, ww, hh2) in ((cimg, (u, v, w2, h2)), (hmap, (x, y, w, hh))):
        cv2.rectangle(im, (a, b), (a + ww, b + hh2), BLACK, 6)
        cv2.rectangle(im, (a, b), (a + ww, b + hh2), GREEN, 3)
    row = np.hstack([cv2.resize(cimg, (480, 360), interpolation=cv2.INTER_AREA),
                     np.full((360, 8, 3), 25, np.uint8), cv2.resize(hmap, (636, 360), interpolation=cv2.INTER_AREA)])
    bar = np.full((78, row.shape[1], 3), 25, np.uint8)
    cv2.putText(bar, f"{obj}:  tape {T[obj][2]} mm,  measured {med:.0f} mm", (12, 34), FONT, 0.9, WHITE, 2, cv2.LINE_AA)
    cv2.putText(bar, f"median over {len(C)} frames (range {min(c[0] for c in C):.0f}-{max(c[0] for c in C):.0f} mm);"
                f"  this frame t = {t:.1f} s, {rng_m:.2f} m away", (12, 64), FONT, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    rows.append(np.vstack([bar, row]))
    summary.append(f"{obj} {med:.0f} mm ({len(C)} frames)")
if rows:
    cv2.imwrite(out + "_objects.png", np.vstack(rows))
print(n, "depth frames;", ", ".join(summary))
