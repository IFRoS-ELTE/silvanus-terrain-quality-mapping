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
"""

import math

import numpy as np
from scipy import ndimage

SPACING = 0.8        # m between lanes: the depth camera sees about 1.6 m across, so neighbours overlap
CLEARANCE = 0.6      # m from the robot centre to any wall or unknown cell (half width 0.35 + margin)
TURN_CLEARANCE = 1.25  # m free all round where the robot turns and where a lane ends: camera housing 1.05 m
                      # ahead, and the governor stops 0.15 m before it (1.2 m), so lane ends never trigger that
MIN_LANE = 1.0       # m: shorter free stretches are not worth a lane
STEP = 0.05          # m between samples along a lane


def clearance_map(grid, resolution):
    """Distance (m) from every cell to the nearest occupied or unknown cell. grid: OccupancyGrid data
    as rows (row 0 = lowest y), -1 unknown, 0..100 occupied probability."""
    free = (grid >= 0) & (grid < 50)
    return ndimage.distance_transform_edt(free) * resolution


def plan(grid, resolution, origin, pose, length=5.0, width=5.0, start=0.3, side=0.0,
         spacing=SPACING, clearance=CLEARANCE, min_lane=MIN_LANE, turn_clearance=TURN_CLEARANCE):
    """Waypoints [(x, y, yaw)] and the lanes [((x0, y0), (x1, y1))] in the map frame.

    pose: robot (x, y, yaw); lanes run along its heading, from start m ahead to start + length.
    side: shift of the area to the left (+) or right (-) of the robot, m."""
    dist = clearance_map(grid, resolution)
    ox, oy = origin
    x, y, yaw = pose
    f = np.array([math.cos(yaw), math.sin(yaw)])
    l = np.array([-f[1], f[0]])

    def room(p):
        """Clearance (m) at each point; 0 outside the map."""
        c, r = np.floor((p[:, 0] - ox) / resolution).astype(int), np.floor((p[:, 1] - oy) / resolution).astype(int)
        inside = (r >= 0) & (r < dist.shape[0]) & (c >= 0) & (c < dist.shape[1])
        out = np.zeros(len(p))
        out[inside] = dist[r[inside], c[inside]]
        return out

    n = max(1, int(round(width / spacing)) + 1)
    base = side + (np.linspace(-width / 2, width / 2, n) if n > 1 else np.zeros(1))
    u = np.arange(start, start + length + 1e-9, STEP)

    def fit(offsets):
        lanes = []
        for v in offsets:
            pts = np.array([x, y]) + np.outer(u, f) + v * l
            d = room(pts)
            free = d >= clearance
            # longest run of free samples
            best, i = None, 0
            while i < len(u):
                if not free[i]:
                    i += 1
                    continue
                j = i
                while j + 1 < len(u) and free[j + 1]:
                    j += 1
                if best is None or j - i > best[1] - best[0]:
                    best = (i, j)
                i = j + 1
            if best:
                # pull the ends in until the robot can turn there with the arm out
                i, j = best
                turn = np.flatnonzero(d[i:j + 1] >= turn_clearance)
                best = (u[i + turn[0]], u[i + turn[-1]]) if len(turn) else None
            if best and best[1] - best[0] >= min_lane:
                a = np.array([x, y]) + best[0] * f + v * l
                b = np.array([x, y]) + best[1] * f + v * l
                lanes.append((v, (tuple(a), tuple(b))))
        return lanes

    # shift the whole pattern sideways by up to half a spacing: in a narrow space the one lane that
    # has room to turn may sit between the offsets centred on the robot
    shifts = np.linspace(-spacing / 2, spacing / 2, 5) if n > 1 or width > 0 else [0.0]
    lanes = max((fit(base + s_) for s_ in shifts), key=lambda ls: sum(math.dist(*ab) for _, ab in ls))
    # nearest side first, so the robot does not cross the whole area to begin
    if lanes and abs(lanes[-1][0]) < abs(lanes[0][0]):
        lanes.reverse()
    xy = []
    for k, (_, (a, b)) in enumerate(lanes):
        xy += [a, b] if k % 2 == 0 else [b, a]
    yaws = [math.atan2(q[1] - p[1], q[0] - p[0]) for p, q in zip(xy, xy[1:])]
    waypoints = [(px, py, yw) for (px, py), yw in zip(xy, yaws + yaws[-1:])]
    return waypoints, [ab for _, ab in lanes]


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
