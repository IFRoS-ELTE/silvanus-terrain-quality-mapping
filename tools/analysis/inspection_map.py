#!/usr/bin/env python3
"""Inspection map of one run: a photo of the driven ground with every hazard and crack on it.

    inspection_map.py BAG OUT_DIR

The camera looks almost straight down (about 10 deg off), so each colour frame can be
laid on the ground plane fitted in the depth image. The frames are stitched in the map
frame of hazard_map.py (wheel speed + IMU heading) at 5 mm per pixel; where frames
overlap, the pixel nearest the centre of its own image wins. On top: the crack pixels
of the U-Net, with the filters of hazard_map.py, and every hazard of hazard_map.py
with a photo of it.

Writes to OUT_DIR:
  ground.jpg        the stitched ground
  cover.png         where ground.jpg shows real ground (alpha), for a viewer's background
  cracks.png        the cracks, same size, transparent elsewhere
  crops/H*.jpg      one photo per hazard: the camera frame of the closest sighting for
                    bumps and holes, a cut-out of the stitched ground for cracks
  inspection.json   hazards with position along the route, side, size, class,
                    priority and photo; the path; the map's pixel scale
"""
import json, os, sys
import numpy as np, cv2
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hazard_map as hm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo

RES = 0.005          # m per pixel of the stitched ground
COLOR_EVERY = 5      # colour at 30 Hz: every 5th frame, 6 a second
V_MAX = 405          # colour rows below this show the gripper
ROUGH_MM = 5.0       # same as the hazard map: small hazards on rougher ground are tufts and chips
CRACK_RGB = (255, 150, 20)


def plane_in_colour(row, T_dc):
    """Ground plane n.p + d = 0 from the depth optical frame into the colour optical frame."""
    n, d = np.array(row[1:4]), row[4]
    R, t = T_dc[:3, :3], T_dc[:3, 3]
    return R.T @ n, float(n @ t + d)


def doubtful(h, offset):
    """Tall, right on the wheel track and still not felt: the chassis clears about 13 cm, so a real
    object like that would have been hit. On wood chips this is a depth error, not an obstacle."""
    return h["type"] == "bump" and h["class"] == "A" and h["best"]["size_mm"] >= 100 and abs(offset) < 0.35


def priority(h, rough, offset=1.0):
    if h["type"] == "crack":
        L = h["length_mm"] / 1000.0
        return "high" if L >= 1.0 else "medium" if L >= 0.4 else "low"
    if h["class"] == "B" or doubtful(h, offset):
        return "check"
    height, felt = h["best"]["size_mm"], h["class"] == "C"
    if height >= 50 or (felt and height >= 30):
        return "high"
    if (height >= 30 or felt) and not (rough and height < 30):
        return "medium"
    return "low"


def main():
    bag, out = sys.argv[1].rstrip("/"), sys.argv[2]
    os.makedirs(os.path.join(out, "crops"), exist_ok=True)
    ctx = hm.build(bag)
    path, hazards, recs, seg = ctx["path"], ctx["hazards"], ctx["recs"], ctx["seg"]
    planes, T_bd, T_bc, t_start = ctx["planes"], ctx["T_bd"], ctx["T_bc"], ctx["t_start"]
    T_cd = np.linalg.inv(T_bc) @ T_bd; T_dc = np.linalg.inv(T_cd)

    # extent: the ground the camera can see, 0.93 m ahead of the robot and about 0.5 m to each side
    cx, cy = hm.to_world(path, path["t"], np.array([[T_bc[0, 3], 0.0]] * len(path["t"])))
    x0, x1 = cx.min() - 0.9, cx.max() + 0.9
    y0, y1 = cy.min() - 0.9, cy.max() + 0.9
    W, H = int(np.ceil((x1 - x0) / RES)), int(np.ceil((y1 - y0) / RES))
    ground = np.full((H, W, 3), 238, np.uint8)
    best = np.full((H, W), np.inf, np.float32)
    crack_n = np.zeros((H, W), np.uint16); cover_n = np.zeros((H, W), np.uint16)
    to_px = lambda x, y: (((x - x0) / RES).astype(int), ((y1 - y) / RES).astype(int))

    # crack cells that passed the height-step filter in hazard_map.py, 5 cm squares grown by 5 cm
    allowed = np.zeros((H, W), np.uint8)
    for h in hazards:
        if h["type"] == "crack":
            px, py = to_px(h["cells"][:, 0], h["cells"][:, 1])
            for a, b in zip(px, py):
                r = int(0.05 / RES)
                allowed[max(b - r, 0):b + r, max(a - r, 0):a + r] = 1

    r = hm.open_bag(bag, [hm.COLOR, hm.CINFO])
    Kc = rays = wgt = uu = vv = None
    n = 0; frames = {}
    want = {}       # filled once the colour intrinsics are known
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == hm.CINFO:
            if Kc is None:
                m = deserialize_message(data, CameraInfo); Kc = (m.k[0], m.k[4], m.k[2], m.k[5])
                vv, uu = np.mgrid[0:V_MAX:2, 0:m.width:2]; vv, uu = vv.ravel(), uu.ravel()
                rays = np.stack([(uu - Kc[2]) / Kc[0], (vv - Kc[3]) / Kc[1], np.ones(len(uu))], 1)
                wgt = ((uu - m.width / 2) ** 2 + (vv - m.height / 2) ** 2).astype(np.float32)
                # photo: the sighting where the hazard sits deepest inside the colour image (the depth
                # camera sees wider), not simply the closest one
                for h in hazards:
                    if h["type"] == "crack" or h["class"] == "B":
                        continue
                    def margin(d):
                        P = T_cd[:3, :3] @ np.array(d["centroid"]) + T_cd[:3, 3]
                        u, v = Kc[0] * P[0] / P[2] + Kc[2], Kc[1] * P[1] / P[2] + Kc[3]
                        return min(u, m.width - u, v, V_MAX - v)
                    h["photo"] = max(h["dets"], key=margin)
                    if margin(h["photo"]) >= 20:
                        want[h["id"]] = h["photo"]["t"]
                    else:       # only the wider depth camera ever saw it
                        h["no_photo"] = "only seen at the side, where the colour camera does not reach"
            continue
        if Kc is None:
            continue
        n += 1
        # keep the frame closest to each hazard's best sighting, for its photo
        for hid, tw in want.items():
            if abs(t - tw) < 0.05 and (hid not in frames or abs(t - tw) < frames[hid][0]):
                frames[hid] = (abs(t - tw), data)
        if n % COLOR_EVERY:
            continue
        # the nearest good plane: on wood chips the fit is poor, but the camera's height and tilt
        # over the ground hardly change, so the last good one is close enough to lay the photo down
        good = planes[planes[:, 5] >= hm.MIN_INLIERS]
        i = int(np.argmin(np.abs(good[:, 0] - t)))
        if abs(good[i, 0] - t) > 3.0:
            continue
        nc, dc = plane_in_colour(good[i], T_dc)
        s = -dc / (rays @ nc)
        ok = (s > 0.2) & (s < 2.0)
        Pc = rays[ok] * s[ok, None]
        Pb = Pc @ T_bc[:3, :3].T + T_bc[:3, 3]
        wx, wy = hm.to_world(path, t, Pb)
        px, py = to_px(wx, wy)
        inb = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        px, py, u, v, w = px[inb], py[inb], uu[ok][inb], vv[ok][inb], wgt[ok][inb]
        c = deserialize_message(data, Image)
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)
        rgb = img if c.encoding == "rgb8" else img[:, :, ::-1]
        better = w < best[py, px]
        ground[py[better], px[better]] = rgb[v[better], u[better]]
        best[py[better], px[better]] = w[better]
        cells = np.unique(py * W + px)
        cover_n.flat[cells] += 1
        if seg is not None:
            _, cm = seg.predict(img, bgr=(c.encoding == "bgr8"))
            cm[hm.straight_lines(cm)] = 0
            hit = cm[v, u] > 0
            crack_n.flat[np.unique(py[hit] * W + px[hit])] += 1

    # close the small gaps between stitched pixels
    empty = ~np.isfinite(best)
    gaps = (empty & (cv2.dilate((~empty).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0)).astype(np.uint8)
    ground = cv2.inpaint(ground, gaps, 2, cv2.INPAINT_TELEA)
    cover = ~empty | (gaps > 0)
    cv2.imwrite(os.path.join(out, "ground.jpg"), ground[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 88])
    mask = np.zeros((H, W, 4), np.uint8); mask[cover, 3] = 255      # where the ground photo is real, for viewers
    cv2.imwrite(os.path.join(out, "cover.png"), mask)

    crack = (crack_n >= 2) & (crack_n >= 0.3 * np.maximum(cover_n, 1)) & (allowed > 0)
    crack = cv2.dilate(crack.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    rgba = np.zeros((H, W, 4), np.uint8); rgba[crack] = (*CRACK_RGB, 235)
    cv2.imwrite(os.path.join(out, "cracks.png"), rgba[:, :, [2, 1, 0, 3]])

    # photos: camera frame of the closest sighting for bumps and holes, the stitched ground for the rest
    for h in hazards:
        fn = f"crops/{h['id']}.jpg"
        crop = np.zeros((0, 0, 3), np.uint8)
        if h.get("no_photo"):
            h["crop"] = None
            continue
        if h["id"] in frames:
            c = deserialize_message(frames[h["id"]][1], Image)
            img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)
            bgr = img[:, :, ::-1].copy() if c.encoding == "rgb8" else img.copy()
            P = T_cd[:3, :3] @ np.array(h["photo"]["centroid"]) + T_cd[:3, 3]
            u, v = Kc[0] * P[0] / P[2] + Kc[2], Kc[1] * P[1] / P[2] + Kc[3]
            half = (max(h["photo"]["length_mm"], 150) / 2 + 120) / 1000 * Kc[0] / P[2]
            a0, a1 = int(max(u - half, 0)), int(min(u + half, c.width)); b0, b1 = int(max(v - half, 0)), int(min(v + half, c.height))
            crop = bgr[b0:b1, a0:a1]
        if min(crop.shape[:2]) < 40:    # outside the colour camera's narrower view: cut it from the stitched ground
            half = int((max(h.get("length_mm", 300) / 2000.0, 0.15) + 0.12) / RES)
            px, py = to_px(np.array([h["x"]]), np.array([h["y"]])); px, py = int(px[0]), int(py[0])
            a0, a1, b0, b1 = max(px - half, 0), min(px + half, W), max(py - half, 0), min(py + half, H)
            crop = ground[b0:b1, a0:a1, ::-1].copy()
            if h["type"] == "crack":
                m = crack[b0:b1, a0:a1]
                crop[m] = (0.25 * crop[m] + 0.75 * np.array(CRACK_RGB[::-1])).astype(np.uint8)
        if crop.size == 0:
            continue
        crop = cv2.resize(crop, (360, int(360 * crop.shape[0] / max(crop.shape[1], 1))), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(out, fn), crop, [cv2.IMWRITE_JPEG_QUALITY, 88])
        h["crop"] = fn

    # position along the route: nearest point of the path, and the side of it
    ds = np.hypot(np.diff(path["x"]), np.diff(path["y"])); s_geo = np.r_[0, np.cumsum(ds)]
    rec_xy = np.array([[r_["x"], r_["y"]] for r_ in recs]); rec_u = np.array([r_["camera_unevenness_mm"] or np.nan for r_ in recs])
    rec_surf = [r_["surface_log"] for r_ in recs]
    items = []
    for h in hazards:
        i = int(np.argmin(np.hypot(path["x"] - h["x"], path["y"] - h["y"])))
        a = path["yaw"][i]
        off = -np.sin(a) * (h["x"] - path["x"][i]) + np.cos(a) * (h["y"] - path["y"][i])
        k = int(np.argmin(np.hypot(rec_xy[:, 0] - h["x"], rec_xy[:, 1] - h["y"])))
        rough = bool(np.nan_to_num(rec_u[k]) > ROUGH_MM)
        j = hm.hazard_json(h)
        px, py = to_px(np.array([h["x"]]), np.array([h["y"]]))
        j.update(px=int(px[0]), py=int(py[0]), route_m=round(float(s_geo[i]), 1), offset_m=round(float(off), 2),
                 side="left" if off > 0.15 else "right" if off < -0.15 else "on the path",
                 surface=rec_surf[k], rough_ground=rough, priority=priority(h, rough, off),
                 note="tall and on the wheel track but not felt: probably a depth error" if doubtful(h, off) else h.get("no_photo"))
        if h["type"] == "crack":
            cx_, cy_ = to_px(h["cells"][:, 0], h["cells"][:, 1])
            j["box_px"] = [int(cx_.min()), int(cy_.min()), int(cx_.max()), int(cy_.max())]
        items.append(j)
    pxp, pyp = to_px(rec_xy[:, 0], rec_xy[:, 1])
    data = {
        "run": ctx["name"], "length_m": round(float(path["s"][-1]), 1), "m_per_px": RES,
        "width_px": W, "height_px": H, "origin": {"x0": round(x0, 3), "y1": round(y1, 3)},
        "path": [[int(a), int(b), None if np.isnan(u) else round(float(u), 1), s_]
                 for a, b, u, s_ in zip(pxp, pyp, rec_u, rec_surf)],
        "crack_length_m": round(sum(i["length_mm"] for i in items if i["type"] == "crack") / 1000.0, 1),
        "hazards": items,
        "rules": {
            "class": "A seen by the camera only, B felt by the IMU only, C seen and felt",
            "priority": "bumps and holes: high from 50 mm, or felt and from 30 mm; medium from 30 mm or felt; "
                        "cracks: high from 1 m long, medium from 0.4 m; check on site: felt only, or 100 mm and more "
                        "on the wheel track without being felt",
            "rough_ground": f"camera unevenness above {ROUGH_MM} mm: small bumps there are mostly tufts and chips"},
    }
    json.dump(data, open(os.path.join(out, "inspection.json"), "w"), indent=1)
    cnt = {p: sum(i["priority"] == p for i in items) for p in ("high", "medium", "low", "check")}
    print(f"{ctx['name']}: ground {W}x{H} px at {RES*1000:.0f} mm, {int(cover.sum() * RES * RES)} m2 covered, "
          f"{len(items)} hazards {cnt}, cracks {data['crack_length_m']} m, {sum('crop' in h and h['crop'] is not None for h in hazards)} photos")


if __name__ == "__main__":
    main()
