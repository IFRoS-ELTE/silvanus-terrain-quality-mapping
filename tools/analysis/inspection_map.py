#!/usr/bin/env python3
"""Inspection map of one run: a photo of the driven ground with every hazard and crack on it.

    inspection_map.py BAG OUT_DIR [--poses SLAM_POSES.csv --map SLAM_MAP.yaml]

The camera looks almost straight down (about 10 deg off), so each colour frame can be
laid on the ground plane fitted in the depth image. The frames are stitched in the map
frame of hazard_map.py (wheel speed + IMU heading, or SLAM with --poses) at 5 mm per
pixel; where frames overlap, the pixel nearest the centre of its own image wins. On top:
the crack pixels of the U-Net, with the filters of hazard_map.py, and every hazard of
hazard_map.py with a photo of it. With --map the lidar map is drawn under it, 2.5 m around
the route; a bag recorded through the speed governor also marks where it slowed or stopped.

Writes to OUT_DIR:
  ground.jpg        the stitched ground
  cover.png         where ground.jpg shows real ground (alpha), for a viewer's background
  cracks.png        the cracks, same size, transparent elsewhere
  crops/H*.jpg      one photo per hazard: the camera frame of the closest sighting for
                    bumps and holes, a cut-out of the stitched ground for cracks
  map.png           with --map: walls dark, free floor light, unknown transparent, same size
  inspection.json   hazards with position along the route, side, size, class,
                    priority and photo; the path, with the governor's mode per 0.25 m; the
                    map's pixel scale
"""
import argparse, json, math, os, sys
import numpy as np, cv2, yaml
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hazard_map as hm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

RES = 0.005          # m per pixel of the stitched ground
COLOR_EVERY = 5      # colour at 30 Hz: every 5th frame, 6 a second
V_MAX = 480          # rows of the colour image; the crack node's robot mask decides which are ground
ROUGH_MM = 5.0       # same as the hazard map: small hazards on rougher ground are tufts and chips
CRACK_RGB = (255, 150, 20)
MAP_MARGIN = 2.5     # m of lidar map shown around the route
STRAIGHT_W = 0.15    # rad/s: frames are laid down only while the robot turns slower than this
MIN_STEP = 0.03      # m the robot must move between two frames laid down
MIN_V = 0.05         # m/s forward: creeping and hovering at a goal give frames at every angle
MIN_RUN = 0.5        # m: frames only from straight stretches at least this long
MIN_RADIUS = 1.0     # m: curves gentler than this are fine while driving; turning on the spot is not
GRID_SHARE = 0.5     # a floor has a grid (tiles, paving) if this share of frames shows lines in two perpendicular directions
WALL_M = 0.10        # with --map: a bump this close to a mapped wall is the foot of the wall
SHIFT_PX = 12        # how far a frame may be moved to line up with the page, px (6 cm)
TURN_DEG, TURN_STEP = 2.5, 0.5   # how far a frame may be turned to line up, and in what steps, deg
MAX_TURN_DEG = 6.0   # the running correction never goes beyond this
MAX_SNAP = 0.18      # m: a frame is moved onto the tile grid only if it needs less than this
MAX_OFFSET = 0.15    # m: the correction carried from frame to frame never grows past this


def main_direction(x, y):
    """Direction (rad) of the longest spread of the driven points: the lanes of a sweep."""
    P = np.stack([x - x.mean(), y - y.mean()])
    w, V = np.linalg.eigh(P @ P.T)
    return math.atan2(V[1, -1], V[0, -1])


def edges(gray):
    g = cv2.GaussianBlur(gray, (3, 3), 0)
    return cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1))


def place(pxf, pyf, centre, turn, offset):
    """Frame pixels turned by `turn` (deg) about the robot centre, then moved by offset: integer page pixels."""
    c, s_ = math.cos(math.radians(turn)), math.sin(math.radians(turn))
    dx, dy = pxf - centre[0], pyf - centre[1]
    return (np.rint(centre[0] + c * dx - s_ * dy).astype(int) + offset[0],
            np.rint(centre[1] + s_ * dx + c * dy).astype(int) + offset[1])


def register(ground, best, pxf, pyf, centre, turn, offset, rgb, turn_deg=None):
    """(turn, offset) that lines a frame up with the page laid so far, searched around the current ones,
    or None when they overlap too little or there is nothing to line up on.

    SLAM's heading is good to 1-2 deg (the tile grid of single frames, 2026-10-27: 44.0-46.5 deg in the
    map), which bends every grout line at the join of two frames, and its position to a few cm, which
    put one line down two or three times. So each frame is tried turned by up to TURN_DEG about the
    robot centre and moved by up to SHIFT_PX, matched on edges (grout lines, crack edges, texture).
    The tile grid repeats every 60 px, so the search cannot jump to the next tile."""
    H, W = best.shape
    gray = rgb.mean(axis=1).astype(np.float32)
    best_fit, best_score, here = None, -1.0, None
    span = TURN_DEG if turn_deg is None else turn_deg
    for a in turn + np.arange(-span, span + 1e-9, TURN_STEP):
        px, py = place(pxf, pyf, centre, a, offset)
        a0, a1, b0, b1 = px.min(), px.max() + 1, py.min(), py.max() + 1
        if a0 - SHIFT_PX < 0 or b0 - SHIFT_PX < 0 or a1 + SHIFT_PX > W or b1 + SHIFT_PX > H:
            return None
        patch = np.zeros((b1 - b0, a1 - a0), np.float32); have = np.zeros_like(patch, np.uint8)
        patch[py - b0, px - a0] = gray; have[py - b0, px - a0] = 1
        have = cv2.erode(cv2.morphologyEx(have, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)), np.ones((5, 5), np.uint8))
        region = (slice(b0 - SHIFT_PX, b1 + SHIFT_PX), slice(a0 - SHIFT_PX, a1 + SHIFT_PX))
        laid = np.isfinite(best[region])
        if laid.mean() < 0.25 or have.mean() < 0.3:
            return None
        page = edges(ground[region].mean(axis=2).astype(np.float32)) * cv2.erode(laid.astype(np.uint8), np.ones((5, 5), np.uint8))
        tmpl = edges(cv2.inpaint(patch.astype(np.uint8), 1 - have, 2, cv2.INPAINT_TELEA).astype(np.float32)) * have
        if tmpl.std() < 1.0 or page.std() < 1.0:
            return None
        score = cv2.matchTemplate(page, tmpl, cv2.TM_CCORR_NORMED)
        by, bx = np.unravel_index(np.argmax(score), score.shape)
        if abs(a - turn) < 1e-9:
            here = score[SHIFT_PX, SHIFT_PX]
        if score[by, bx] > best_score:
            best_score, best_fit = score[by, bx], (float(a), offset + np.array([bx - SHIFT_PX, by - SHIFT_PX]))
    if best_fit is None or best_score < 0.3 or best_score < 1.05 * here or abs(best_fit[0]) > MAX_TURN_DEG:
        return turn, offset
    return best_fit


def grid_lines(Pb, gray):
    """The straight lines under the camera, in the robot frame (x ahead, y left): (direction of the
    strongest family, deg 0-90, [(direction deg, midpoint xy, length m), ...]) or None.
    Laid flat, the tiles of one frame are square to 0.5 deg (2026-10-27), so on a tile or paving floor
    these lines tell each frame's true heading and, against the grid's spacing, its true position."""
    R = 0.005
    X, Y = Pb[:, 0], Pb[:, 1]
    cols, rows = ((X - X.min()) / R).astype(int), ((Y.max() - Y) / R).astype(int)
    img = np.zeros((rows.max() + 1, cols.max() + 1), np.uint8); img[rows, cols] = gray.astype(np.uint8)
    have = np.zeros_like(img); have[rows, cols] = 1
    img = cv2.inpaint(img, 1 - cv2.morphologyEx(have, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)), 2, cv2.INPAINT_TELEA)
    e = cv2.Canny(cv2.GaussianBlur(img, (5, 5), 0), 15, 45) * cv2.erode(have, np.ones((9, 9), np.uint8))
    L = cv2.HoughLinesP(e, 1, np.pi / 720, 40, minLineLength=int(0.25 / R), maxLineGap=6)
    if L is None or len(L) < 3:
        return None
    lines = []
    for x1, y1, x2, y2 in L[:, 0]:              # back to the robot frame: rows point down, so y flips
        p1 = np.array([X.min() + x1 * R, Y.max() - y1 * R]); p2 = np.array([X.min() + x2 * R, Y.max() - y2 * R])
        lines.append((math.degrees(math.atan2(*(p2 - p1)[::-1])) % 180, (p1 + p2) / 2, float(np.hypot(*(p2 - p1)))))
    a = np.array([l[0] % 90 for l in lines]); n = np.array([l[2] for l in lines])
    ref = a[np.argmax(n)]
    d = (a - ref + 45) % 90 - 45
    agree = np.abs(d) < 2.0
    if n[agree].sum() < 1.0:                   # at least 1 m of lines agreeing to 2 deg
        return None
    return float((ref + np.average(d[agree], weights=n[agree])) % 90), [l for l, k in zip(lines, agree) if k]


def lattice(lines, x, y, yaw_deg, G):
    """Offsets of a frame's lines across the two directions of the grid (G and G + 90 deg, map frame):
    {0: [rho, ...], 1: [...]} with rho = distance of the line from the map origin along that direction's
    normal, for lines turned into the map with the frame's (corrected) heading yaw_deg."""
    out = {0: [], 1: []}
    c, s_ = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    for direction, mid, _ in lines:
        dw = (direction + yaw_deg) % 180
        k = 0 if min(abs(dw - G) % 180, 180 - abs(dw - G) % 180) < 10 else 1
        beta = math.radians(G + 90 * k)
        nrm = np.array([-math.sin(beta), math.cos(beta)])
        pw = np.array([x + c * mid[0] - s_ * mid[1], y + s_ * mid[0] + c * mid[1]])
        out[k].append(float(nrm @ pw))
    return out


def two_ways(gl):
    """Lines in both directions of a grid, at right angles: tiles or paving, not one kerb or path edge."""
    ref, lines = gl
    d = [((l[0] - ref) % 180) for l in lines]
    return any(min(x, 180 - x) < 2 for x in d) and any(abs(x - 90) < 2 for x in d)


def grid_direction(bag, path, planes, T_bc, T_dc, t_start):
    """First pass: the direction (deg, 0-90, map frame) of the floor's straight lines, the median over
    every frame that is laid down, or None on a floor without them. Decided before laying anything, so
    the first frames are corrected like all the others."""
    r = hm.open_bag(bag, [hm.COLOR, hm.CINFO])
    K = rays = None; n = 0; last = None; seen = []; frames = 0
    good = planes[planes[:, 5] >= hm.MIN_INLIERS]
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == hm.CINFO:
            if K is None:
                m = deserialize_message(data, CameraInfo); K = (m.k[0], m.k[4], m.k[2], m.k[5])
                vv, uu = np.mgrid[0:400:2, 0:m.width:2]; vv, uu = vv.ravel(), uu.ravel()
                rays = np.stack([(uu - K[2]) / K[0], (vv - K[3]) / K[1], np.ones(len(uu))], 1)
            continue
        n += 1
        if K is None or n % COLOR_EVERY or not path["straight"][min(np.searchsorted(path["t"], t), len(path["t"]) - 1)]:
            continue
        here = np.array(hm.pose_at(path, t)[:2])
        if last is not None and np.hypot(*(here - last)) < MIN_STEP:
            continue
        last = here
        i = int(np.argmin(np.abs(good[:, 0] - t)))
        if abs(good[i, 0] - t) > 3.0:
            continue
        nc, dc = plane_in_colour(good[i], T_dc); s_ = -dc / (rays @ nc); ok = (s_ > 0.2) & (s_ < 2.0)
        Pb = (rays[ok] * s_[ok, None]) @ T_bc[:3, :3].T + T_bc[:3, 3]
        c = deserialize_message(data, Image); img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)
        gl = grid_lines(Pb, img[vv[ok], uu[ok]].mean(axis=1))
        frames += 1
        if gl is not None and two_ways(gl):
            seen.append((gl[0] + math.degrees(hm.pose_at(path, t)[2])) % 90)
    if len(seen) < 10 or len(seen) < GRID_SHARE * frames:      # kerbs and path edges are lines, not a grid
        return None
    ref = seen[0]
    return float((ref + np.median([(g - ref + 45) % 90 - 45 for g in seen])) % 90)


def page_lines(ground, cover, grid_deg, min_len=0.4):
    """Pixels on long straight lines of the finished page that run along the floor's grid (grid_deg,
    page frame, and at right angles to it): grout and paving joints, not cracks."""
    gray = cv2.cvtColor(ground, cv2.COLOR_RGB2GRAY)
    e = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 20, 60) * cover.astype(np.uint8)
    lines = cv2.HoughLinesP(e, 1, np.pi / 360, threshold=60, minLineLength=int(min_len / RES), maxLineGap=12)
    out = np.zeros(gray.shape, np.uint8)
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            a = (-math.degrees(math.atan2(y2 - y1, x2 - x1)) - grid_deg) % 90       # page rows point down
            if min(a, 90 - a) < 3:
                cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)), 1, int(0.03 / RES))
    return out > 0


def wall_distance(map_yaml):
    """Distance (m) from a map point to the nearest occupied cell of the lidar map."""
    meta = yaml.safe_load(open(map_yaml))
    occ = cv2.imread(os.path.join(os.path.dirname(map_yaml), meta["image"]), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    d = ndimage.distance_transform_edt(occ >= 50) * res
    def at(x, y):
        c, r = int((x - ox) / res), occ.shape[0] - 1 - int((y - oy) / res)
        return float(d[r, c]) if 0 <= r < d.shape[0] and 0 <= c < d.shape[1] else math.inf
    return at


def map_layer(map_yaml, x0, y1, W, H, theta=0.0):
    """The occupancy map resampled onto the ground photo's pixels (page turned by theta), as RGBA."""
    meta = yaml.safe_load(open(map_yaml))
    occ = cv2.imread(os.path.join(os.path.dirname(map_yaml), meta["image"]), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    pu, pv = np.meshgrid(x0 + (np.arange(W) + 0.5) * RES, y1 - (np.arange(H) + 0.5) * RES)
    wx, wy = pu * math.cos(theta) - pv * math.sin(theta), pu * math.sin(theta) + pv * math.cos(theta)
    c = ((wx - ox) / res).astype(int)
    r = occ.shape[0] - 1 - ((wy - oy) / res).astype(int)
    inside = (r >= 0) & (r < occ.shape[0]) & (c >= 0) & (c < occ.shape[1])
    cell = np.full((H, W), 205, np.uint8)
    cell[inside] = occ[r[inside], c[inside]]
    rgba = np.zeros((H, W, 4), np.uint8)
    rgba[cell > 250] = (246, 244, 240, 255)          # free floor
    rgba[cell < 50] = (52, 52, 56, 255)              # wall or obstacle
    return rgba


def governor_modes(bag, times):
    """The speed governor's mode (free / slow / stop) for each interval between record times, if recorded."""
    r = hm.open_bag(bag, ["/governor/state"])
    S = []
    while r.has_next():
        _, data, st = r.read_next()
        S.append((st * 1e-9, json.loads(deserialize_message(data, String).data)["mode"]))
    if not S:
        return [None] * len(times)
    t = np.array([s[0] for s in S]); m = np.array([s[1] for s in S])
    out, prev = [], times[0] - 0.5
    for tr in times:
        w = m[(t > prev) & (t <= tr)]
        out.append("stop" if "stop" in w else "slow" if "slow" in w else "free" if len(w) else None)
        prev = tr
    return out


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
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--poses", help="SLAM poses CSV (tools/analysis/run_slam.sh): position from SLAM, not wheels + IMU")
    ap.add_argument("--map", help="occupancy map YAML in the same frame as --poses, drawn under the ground photo")
    opt = ap.parse_args()
    bag, out = opt.bag.rstrip("/"), opt.out
    os.makedirs(os.path.join(out, "crops"), exist_ok=True)
    ctx = hm.build(bag, poses=opt.poses)
    path, hazards, recs, seg = ctx["path"], ctx["hazards"], ctx["recs"], ctx["seg"]
    planes, T_bd, T_bc, t_start = ctx["planes"], ctx["T_bd"], ctx["T_bc"], ctx["t_start"]
    T_cd = np.linalg.inv(T_bc) @ T_bd; T_dc = np.linalg.inv(T_cd)

    # the page runs along the main driving direction, so a sweep comes out as a rectangle
    straight = (np.abs(path["w"]) <= STRAIGHT_W + np.maximum(path["v"], 0) / MIN_RADIUS) & (path["v"] >= MIN_V)
    lab, nlab = ndimage.label(straight)                 # only stretches of at least MIN_RUN m: creeping and
    for k in range(1, nlab + 1):                        # hovering at a goal give short bits at odd angles
        idx = np.flatnonzero(lab == k)
        if path["s"][idx[-1]] - path["s"][idx[0]] < MIN_RUN:
            straight[idx] = False
    path["straight"] = straight
    theta = main_direction(path["x"][straight], path["y"][straight]) if straight.sum() > 10 else 0.0
    theta = (theta + math.pi / 2) % math.pi - math.pi / 2               # never turn the page upside down
    ct, st_ = math.cos(theta), math.sin(theta)
    along = lambda x, y: (x * ct + y * st_, -x * st_ + y * ct)          # map frame -> page frame

    # extent: the ground the camera can see, 0.93 m ahead of the robot and about 0.5 m to each side
    cx, cy = along(*hm.to_world(path, path["t"][straight], np.array([[T_bc[0, 3], 0.0]] * straight.sum())))
    margin = MAP_MARGIN if opt.map else 0.9
    x0, x1 = cx.min() - margin, cx.max() + margin
    y0, y1 = cy.min() - margin, cy.max() + margin
    W, H = int(np.ceil((x1 - x0) / RES)), int(np.ceil((y1 - y0) / RES))
    ground = np.full((H, W, 3), 238, np.uint8)
    best = np.full((H, W), np.inf, np.float32)
    crack_n = np.zeros((H, W), np.uint16); cover_n = np.zeros((H, W), np.uint16)

    def to_px(x, y, exact=False):
        u, v = along(np.asarray(x, float), np.asarray(y, float))
        px, py = (u - x0) / RES, (y1 - v) / RES
        return (px, py) if exact else (px.astype(int), py.astype(int))

    # crack cells that passed the height-step filter in hazard_map.py, 5 cm squares grown by 5 cm
    allowed = np.zeros((H, W), np.uint8)
    for h in hazards:
        if h["type"] == "crack":
            px, py = to_px(h["cells"][:, 0], h["cells"][:, 1])
            for a, b in zip(px, py):
                r = int(0.05 / RES)
                allowed[max(b - r, 0):b + r, max(a - r, 0):a + r] = 1

    G = grid_direction(bag, path, planes, T_bc, T_dc, t_start)
    r = hm.open_bag(bag, [hm.COLOR, hm.CINFO])
    Kc = rays = wgt = uu = vv = None
    n = 0; frames = {}
    want = {}       # filled once the colour intrinsics are known
    offset = np.zeros(2, int); turn = 0.0; last_xy = None; used = shifted = 0
    grid_seen, grid_fixed, grid_snapped, laid_lines = [], 0, 0, {0: [], 1: []}
    while r.has_next():
        tp, data, st = r.read_next(); t = st * 1e-9 - t_start
        if tp == hm.CINFO:
            if Kc is None:
                m = deserialize_message(data, CameraInfo); Kc = (m.k[0], m.k[4], m.k[2], m.k[5])
                vv, uu = np.mgrid[0:m.height:2, 0:m.width:2]; vv, uu = vv.ravel(), uu.ravel()
                keep = seg.ground[vv, uu] > 0 if seg is not None else vv < 400
                vv, uu = vv[keep], uu[keep]
                V_MAX = int(vv.max()) + 1
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
        # only while driving straight, and only after moving: frames taken while the robot turned on
        # the spot land on the page at every angle (the swirls of 2026-10-27), and a robot standing
        # still pastes the same patch over and over
        if not path["straight"][min(np.searchsorted(path["t"], t), len(path["t"]) - 1)]:
            continue
        here = np.array(hm.pose_at(path, t)[:2])
        if last_xy is not None and np.hypot(*(here - last_xy)) < MIN_STEP:
            continue
        last_xy = here
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
        pxf, pyf = to_px(wx, wy, exact=True)
        centre = np.array(to_px(*hm.pose_at(path, t)[:2], exact=True), float)      # the robot, turning point
        c = deserialize_message(data, Image)
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, 3)
        rgb = img if c.encoding == "rgb8" else img[:, :, ::-1]
        # line the frame up with the page so far: SLAM is good to a few cm, which put one grout
        # line down two or three times. The correction carries over to the next frame.
        u_all, v_all = uu[ok], vv[ok]
        span, snapped = None, False
        gl = grid_lines(Pb, rgb[v_all, u_all].mean(axis=1))
        rx, ry, ryaw = hm.pose_at(path, t)
        if gl is not None and G is not None and two_ways(gl):
            g_world = (gl[0] + math.degrees(ryaw)) % 90
            fix = (G - g_world + 45) % 90 - 45              # heading error of this frame, deg (map frame)
            if abs(fix) <= 10.0:
                turn, span = -fix, 0.0                      # page rows point down: a map turn is the opposite page turn
                grid_fixed += 1
                # position: put this frame's grout lines on the same lines already laid (nearest within
                # 14 cm, under half a 30 cm tile, so no line can be taken for its neighbour), if they agree
                ou, ov = offset[0] * RES, -offset[1] * RES        # the correction carried over, as a map vector
                ox_, oy_ = ou * ct - ov * st_, ou * st_ + ov * ct
                rho = lattice(gl[1], rx + ox_, ry + oy_, math.degrees(ryaw) + fix, G)
                delta = np.zeros(2); matched = 0; dks = {0: 0.0, 1: 0.0}
                for k in (0, 1):
                    refs = np.array(laid_lines[k])
                    if len(refs) and rho[k]:
                        res_ = np.array([refs[np.argmin(np.abs(refs - r_))] - r_ for r_ in rho[k]])
                        res_ = res_[np.abs(res_) < 0.14]
                        if len(res_) >= 2 and np.median(np.abs(res_ - np.median(res_))) < 0.02:
                            dks[k] = float(np.median(res_)); matched += 1
                            beta = math.radians(G + 90 * k)
                            delta += dks[k] * np.array([-math.sin(beta), math.cos(beta)])
                if matched and np.hypot(*delta) <= MAX_SNAP:
                    du, dv = along(*delta)                      # map vector -> page vector
                    offset = offset + np.rint([du / RES, -dv / RES]).astype(int)
                    snapped = True; grid_snapped += 1
                new_area = all(not len(laid_lines[k]) or min(abs(np.array(laid_lines[k]) - r_).min() for r_ in rho[k]) > 0.14
                               for k in (0, 1) if rho[k])
                if snapped or new_area:
                    for k in (0, 1):                            # this frame's lines, as laid, extend the reference
                        refs = np.array(laid_lines[k])
                        for r_ in rho[k]:
                            if not len(refs) or np.min(np.abs(refs - (r_ + dks[k]))) > 0.03:
                                laid_lines[k].append(r_ + dks[k])
        fit = None if snapped else register(ground, best, pxf, pyf, centre, turn, offset, rgb[v_all, u_all], turn_deg=span)
        if fit is not None and (fit[0] != turn or (fit[1] != offset).any()):
            shifted += 1
            turn, offset = fit
        elif not snapped:
            # nothing better to line up on: let the correction fade, so that over a long route the
            # page stays on the map positions (hazards, cracks) instead of drifting off with it
            turn, offset = 0.9 * turn, np.rint(0.9 * offset).astype(int)
        if np.hypot(*offset) * RES > MAX_OFFSET:
            offset = np.rint(offset * MAX_OFFSET / (np.hypot(*offset) * RES)).astype(int)
        turn = float(np.clip(turn, -MAX_TURN_DEG, MAX_TURN_DEG))
        px, py = place(pxf, pyf, centre, turn, offset)
        used += 1
        inb = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        px, py, u, v, w = px[inb], py[inb], u_all[inb], v_all[inb], wgt[ok][inb]
        better = w < best[py, px]
        ground[py[better], px[better]] = rgb[v[better], u[better]]
        best[py[better], px[better]] = w[better]
        cells = np.unique(py * W + px)
        cover_n.flat[cells] += 1
        if seg is not None:
            _, cm = seg.predict(img, bgr=(c.encoding == "bgr8"), ground_mask=seg.ground)
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
    if opt.map:
        cv2.imwrite(os.path.join(out, "map.png"), cv2.cvtColor(map_layer(opt.map, x0, y1, W, H, theta), cv2.COLOR_RGBA2BGRA))

    # cracks: seen in at least 3 frames and in 40 % of the frames that saw the spot, and not on a
    # long straight line of the finished page. The per-frame filter only drops straight runs it sees
    # whole; on the page the tile grid is whole, and anything lying on it is grout (2026-10-27: every
    # "crack" of the corridor sweep was a piece of grout)
    if G is not None:
        crack = (crack_n >= 3) & (crack_n >= 0.4 * np.maximum(cover_n, 1)) & (allowed > 0)
        crack &= ~page_lines(ground, cover, G - math.degrees(theta))
    else:                                   # no grid: asphalt cracks can be long and straight too
        crack = (crack_n >= 2) & (crack_n >= 0.3 * np.maximum(cover_n, 1)) & (allowed > 0)
    crack = cv2.dilate(crack.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    # a crack item keeps its place in the work list only if its pixels survived
    kept = []
    for h in hazards:
        if h["type"] == "crack":
            hx, hy = to_px(h["cells"][:, 0], h["cells"][:, 1])
            r_ = int(0.05 / RES)
            on = [crack[max(b - r_, 0):b + r_, max(a_ - r_, 0):a_ + r_].any() for a_, b in zip(hx, hy)]
            if np.mean(on) < 0.3:
                continue
        kept.append(h)
    dropped = len(hazards) - len(kept); hazards = kept
    if opt.map:
        wall = wall_distance(opt.map)
        on_wall = [h for h in hazards if h["type"] in ("bump", "hole") and h["class"] != "C" and wall(h["x"], h["y"]) < WALL_M]
        hazards = [h for h in hazards if h not in on_wall]
        dropped += len(on_wall)
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
    modes = governor_modes(bag, [r_["t"] for r_ in recs])
    data = {
        "run": ctx["name"], "length_m": round(float(path["s"][-1]), 1), "m_per_px": RES,
        "width_px": W, "height_px": H, "origin": {"x0": round(x0, 3), "y1": round(y1, 3)},
        "rotation_deg": round(math.degrees(theta), 2),     # page x axis from the map x axis (along the lanes)
        "map": "map.png" if opt.map else None,
        "positions": "SLAM (lidar)" if opt.poses else "wheels + IMU heading",
        "path": [[int(px_), int(py_), None if np.isnan(u) else round(float(u), 1), s_, g]
                 for px_, py_, u, s_, g in zip(pxp, pyp, rec_u, rec_surf, modes)],
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
    print(f"{ctx['name']}: {used} frames laid while driving straight, {grid_fixed} turned and {grid_snapped} moved onto the floor's grid, "
          f"{shifted} moved to line up, "
          f"{dropped} items dropped: cracks on the page's straight lines, bumps on walls")
    print(f"{ctx['name']}: ground {W}x{H} px at {RES*1000:.0f} mm, {int(cover.sum() * RES * RES)} m2 covered, "
          f"{len(items)} hazards {cnt}, cracks {data['crack_length_m']} m, {sum('crop' in h and h['crop'] is not None for h in hazards)} photos")


if __name__ == "__main__":
    main()
