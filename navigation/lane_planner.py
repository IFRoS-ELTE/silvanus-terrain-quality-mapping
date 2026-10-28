"""Lanes for a coverage sweep, fitted into the free floor of an occupancy map. No ROS here.

The robot stands where the sweep should start, facing the area. The area is a rectangle of up to
LENGTH m ahead and WIDTH m across (centred on the robot unless told otherwise). Only floor that the
map knows is free and that is at least CLEARANCE from any wall or unknown cell counts, so walls,
furniture and unmapped corners are left out by themselves. Each lane is cut to its longest free
stretch; lanes shorter than MIN_LANE are dropped. Lanes alternate direction, and every corner faces
the next leg (see coverage_sweep.lanes for why).

The arm counts too. In Pose B the camera sits 0.94 m ahead of the robot centre, so turning on the
spot sweeps a circle of about 1 m. Along a lane the arm points ahead and the body needs CLEARANCE to
the side; at every corner, where the robot turns, TURN_CLEARANCE must be free all round, so lane ends
are pulled back until it is.

Without a length the area is the whole room: all the open floor the robot can reach from where it
stands without passing anywhere narrower than twice CLEARANCE, which a door is, so the sweep stays in
the room even when the lidar sees out through the door. Lanes then also lie behind the robot, and a
lane line that furniture cuts in two keeps both stretches. Floor the camera has already seen can be
left out, so a second pass covers only what the map has gained.
"""

import math

import cv2              # not scipy: the robot's container has OpenCV but no scipy
import numpy as np

SPACING = 0.8        # m between lanes: the depth camera sees about 1.6 m across, so neighbours overlap
CLEARANCE = 0.6      # m from the robot centre to any wall or unknown cell (half width 0.35 + margin)
TURN_CLEARANCE = 1.25  # m free all round where the robot turns and where a lane ends: camera housing 1.05 m
                      # ahead, and the governor stops 0.15 m before it (1.2 m), so lane ends never trigger that
MIN_LANE = 1.0       # m: shorter free stretches are not worth a lane
STEP = 0.05          # m between samples along a lane
CAMERA_AHEAD = 0.94  # m from the robot centre to the camera in Pose B
SEE_RADIUS = 0.4     # m of floor around the camera's point that counts as seen (colour view 0.84 m across)
SEEN_SHARE = 0.5     # a stretch whose strip of floor is more than this much seen already is skipped


def clearance_map(grid, resolution):
    """Distance (m) from every cell to the nearest occupied or unknown cell. grid: OccupancyGrid data
    as rows (row 0 = lowest y), -1 unknown, 0..100 occupied probability."""
    free = (grid >= 0) & (grid < 50)
    return edt(free) * resolution


def plan(grid, resolution, origin, pose, length=5.0, width=5.0, start=0.3, side=0.0,
         spacing=SPACING, clearance=CLEARANCE, min_lane=MIN_LANE, turn_clearance=TURN_CLEARANCE, seen=None):
    """Waypoints [(x, y, yaw)] and the lanes [((x0, y0), (x1, y1))], in the order driven, in the map frame.

    pose: robot (x, y, yaw); lanes run along its heading, from start m ahead to start + length.
    side: shift of the area to the left (+) or right (-) of the robot, m.
    length None: the whole room (module docstring); width, start and side are then not used.
    seen: bool array like grid, floor the camera has already covered (seen_mask)."""
    dist = clearance_map(grid, resolution)
    ox, oy = origin
    x, y, yaw = pose
    f = np.array([math.cos(yaw), math.sin(yaw)])
    l = np.array([-f[1], f[0]])
    whole = length is None
    if whole:
        # the open floor connected to the robot; a door is narrower than 2 x clearance and cuts it off
        _, lab = cv2.connectedComponents((dist >= clearance).astype(np.uint8), connectivity=4)
        r0, c0 = int((y - oy) / resolution), int((x - ox) / resolution)
        k = lab[r0, c0] if 0 <= r0 < lab.shape[0] and 0 <= c0 < lab.shape[1] else 0
        if k == 0:                     # the robot stands closer to a wall than that: the nearest open floor
            cells = np.argwhere(lab > 0)
            if not len(cells):
                return [], []
            k = lab[tuple(cells[np.argmin(np.hypot(cells[:, 0] - r0, cells[:, 1] - c0))])]
        dist = np.where(lab == k, dist, 0.0)
        rr, cc = np.nonzero(lab == k)
        px, py = ox + (cc + 0.5) * resolution - x, oy + (rr + 0.5) * resolution - y
        along, across = px * f[0] + py * f[1], px * l[0] + py * l[1]
        start, length = float(along.min()), float(along.max() - along.min())
        side, width = float(across.max() + across.min()) / 2, float(across.max() - across.min())

    def lookup(a, p):
        """a at each point; 0 outside the map."""
        c, r = np.floor((p[:, 0] - ox) / resolution).astype(int), np.floor((p[:, 1] - oy) / resolution).astype(int)
        inside = (r >= 0) & (r < a.shape[0]) & (c >= 0) & (c < a.shape[1])
        out = np.zeros(len(p), a.dtype)
        out[inside] = a[r[inside], c[inside]]
        return out

    n = max(1, int(round(width / spacing)) + 1)
    base = side + (np.linspace(-width / 2, width / 2, n) if n > 1 else np.zeros(1))
    u = np.arange(start, start + length + 1e-9, STEP)

    def fit(offsets):
        lanes = []
        for v in offsets:
            pts = np.array([x, y]) + np.outer(u, f) + v * l
            d = lookup(dist, pts)
            free = d >= clearance
            runs, i = [], 0
            while i < len(u):
                if not free[i]:
                    i += 1
                    continue
                j = i
                while j + 1 < len(u) and free[j + 1]:
                    j += 1
                runs.append((i, j))
                i = j + 1
            if not whole and runs:         # a rectangle keeps the longest stretch of each lane
                runs = [max(runs, key=lambda r: r[1] - r[0])]
            for i, j in runs:
                # pull the ends in until the robot can turn there with the arm out
                turn = np.flatnonzero(d[i:j + 1] >= turn_clearance)
                if not len(turn) or u[i + turn[-1]] - u[i + turn[0]] < min_lane:
                    continue
                i, j = i + turn[0], i + turn[-1]
                # the strip of floor the camera would see from this stretch, already mostly seen: skip it
                if seen is not None and np.mean([lookup(seen, pts[i:j + 1] + o * l).mean()
                                                 for o in np.linspace(-SEE_RADIUS, SEE_RADIUS, 5)]) > SEEN_SHARE:
                    continue
                lanes.append((v, u[i], tuple(pts[i]), tuple(pts[j])))
        return lanes

    # shift the whole pattern sideways by up to half a spacing: in a narrow space the one lane that
    # has room to turn may sit between the offsets centred on the robot
    shifts = np.linspace(-spacing / 2, spacing / 2, 5) if n > 1 or width > 0 else [0.0]
    lanes = max((fit(base + s_) for s_ in shifts), key=lambda ls: sum(math.dist(a, b) for _, _, a, b in ls))
    if not lanes:
        return [], []
    # one line after the other, each stretch of a line in turn, back and forth
    lines = [sorted((ln for ln in lanes if ln[0] == v), key=lambda ln: ln[1]) for v in sorted({ln[0] for ln in lanes})]
    # nearest side first, so the robot does not cross the whole area to begin
    if abs(lines[-1][0][0]) < abs(lines[0][0][0]):
        lines.reverse()

    def route(flip):
        xy, order = [], []
        for k, line in enumerate(lines):
            back = (k + flip) % 2 == 1
            for _, _, a, b in (line[::-1] if back else line):
                xy += [b, a] if back else [a, b]
                order.append((b, a) if back else (a, b))
        return xy, order
    # the whole room may lie behind the robot as well: begin at the nearer end of the first line
    xy, order = min((route(fl) for fl in ((0, 1) if whole else (0,))), key=lambda r: math.dist(r[0][0], (x, y)))
    yaws = [math.atan2(q[1] - p[1], q[0] - p[0]) for p, q in zip(xy, xy[1:])]
    waypoints = [(px, py, yw) for (px, py), yw in zip(xy, yaws + yaws[-1:])]
    return waypoints, order


def seen_mask(shape, resolution, origin, points, radius=SEE_RADIUS):
    """Cells within radius of any of the camera's points [(x, y)] on the floor: what it has covered."""
    mark = np.zeros(shape, bool)
    if len(points):
        p = np.asarray(points, float)
        c, r = np.floor((p[:, 0] - origin[0]) / resolution).astype(int), np.floor((p[:, 1] - origin[1]) / resolution).astype(int)
        ok = (r >= 0) & (r < shape[0]) & (c >= 0) & (c < shape[1])
        mark[r[ok], c[ok]] = True
    if not mark.any():
        return mark
    return edt(~mark) * resolution <= radius


def edt(mask):
    """Distance in cells from every True cell to the nearest False one (exact Euclidean)."""
    return cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE).astype(float)


def nearest_obstacle(grid, resolution, origin, xy):
    """Distance (m) from xy to the nearest occupied cell; unknown cells do not count, since before the
    first turn most of the map is still unknown."""
    occ = np.argwhere(grid >= 50)
    if not len(occ):
        return math.inf
    wx = origin[0] + (occ[:, 1] + 0.5) * resolution
    wy = origin[1] + (occ[:, 0] + 0.5) * resolution
    return float(np.min(np.hypot(wx - xy[0], wy - xy[1])))


def covered_area(lanes, see=1.6):
    """Floor the camera covers, m^2: each lane's length times what the camera sees across."""
    return sum(math.dist(a, b) for a, b in lanes) * see
