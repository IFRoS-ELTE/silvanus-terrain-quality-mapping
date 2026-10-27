"""Lanes for a coverage sweep, fitted into the free floor of an occupancy map. No ROS here.

The robot stands where the sweep should start, facing the area. The area is a rectangle of up to
LENGTH m ahead and WIDTH m across (centred on the robot unless told otherwise). Only floor that the
map knows is free and that is at least CLEARANCE from any wall or unknown cell counts, so walls,
furniture and unmapped corners are left out by themselves. Each lane is cut to its longest free
stretch; lanes shorter than MIN_LANE are dropped. Lanes alternate direction, and every corner faces
the next leg (see coverage_sweep.lanes for why).
"""

import math

import numpy as np
from scipy import ndimage

SPACING = 0.8        # m between lanes: the depth camera sees about 1.6 m across, so neighbours overlap
CLEARANCE = 0.6      # m from the robot centre to any wall or unknown cell (half width 0.35 + margin)
MIN_LANE = 1.0       # m: shorter free stretches are not worth a lane
STEP = 0.05          # m between samples along a lane


def clearance_map(grid, resolution):
    """Distance (m) from every cell to the nearest occupied or unknown cell. grid: OccupancyGrid data
    as rows (row 0 = lowest y), -1 unknown, 0..100 occupied probability."""
    free = (grid >= 0) & (grid < 50)
    return ndimage.distance_transform_edt(free) * resolution


def plan(grid, resolution, origin, pose, length=5.0, width=5.0, start=0.3, side=0.0,
         spacing=SPACING, clearance=CLEARANCE, min_lane=MIN_LANE):
    """Waypoints [(x, y, yaw)] and the lanes [((x0, y0), (x1, y1))] in the map frame.

    pose: robot (x, y, yaw); lanes run along its heading, from start m ahead to start + length.
    side: shift of the area to the left (+) or right (-) of the robot, m."""
    dist = clearance_map(grid, resolution)
    ox, oy = origin
    x, y, yaw = pose
    f = np.array([math.cos(yaw), math.sin(yaw)])
    l = np.array([-f[1], f[0]])

    def ok(p):
        c, r = np.floor((p[:, 0] - ox) / resolution).astype(int), np.floor((p[:, 1] - oy) / resolution).astype(int)
        inside = (r >= 0) & (r < dist.shape[0]) & (c >= 0) & (c < dist.shape[1])
        out = np.zeros(len(p), bool)
        out[inside] = dist[r[inside], c[inside]] >= clearance
        return out

    n = max(1, int(round(width / spacing)) + 1)
    offsets = side + (np.linspace(-width / 2, width / 2, n) if n > 1 else np.zeros(1))
    u = np.arange(start, start + length + 1e-9, STEP)
    lanes = []
    for v in offsets:
        pts = np.array([x, y]) + np.outer(u, f) + v * l
        free = ok(pts)
        # longest run of free samples
        best, i = None, 0
        while i < len(u):
            if not free[i]:
                i += 1
                continue
            j = i
            while j + 1 < len(u) and free[j + 1]:
                j += 1
            if best is None or u[j] - u[i] > best[1] - best[0]:
                best = (u[i], u[j])
            i = j + 1
        if best and best[1] - best[0] >= min_lane:
            a = np.array([x, y]) + best[0] * f + v * l
            b = np.array([x, y]) + best[1] * f + v * l
            lanes.append((v, (tuple(a), tuple(b))))
    # nearest side first, so the robot does not cross the whole area to begin
    if lanes and abs(lanes[-1][0]) < abs(lanes[0][0]):
        lanes.reverse()
    xy = []
    for k, (_, (a, b)) in enumerate(lanes):
        xy += [a, b] if k % 2 == 0 else [b, a]
    yaws = [math.atan2(q[1] - p[1], q[0] - p[0]) for p, q in zip(xy, xy[1:])]
    waypoints = [(px, py, yw) for (px, py), yw in zip(xy, yaws + yaws[-1:])]
    return waypoints, [ab for _, ab in lanes]


def covered_area(lanes, see=1.6):
    """Floor the camera covers, m^2: each lane's length times what the camera sees across."""
    return sum(math.dist(a, b) for a, b in lanes) * see
