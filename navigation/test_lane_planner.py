"""The lane planner on made-up rooms, without ROS: python3 -m pytest navigation/test_lane_planner.py"""
import math
import unittest

import numpy as np

import lane_planner as lp

RES = 0.05


def room(w=8.0, h=6.0):
    """Free floor w x h m with a 0.1 m wall all round, origin at the outer corner."""
    g = np.zeros((int(h / RES), int(w / RES)), np.int16)
    g[:2, :] = g[-2:, :] = g[:, :2] = g[:, -2:] = 100
    return g


def clearance_at(g, x, y):
    return lp.clearance_map(g, RES)[int(y / RES), int(x / RES)]


class PlanTest(unittest.TestCase):
    def test_corners_leave_room_for_the_arm_to_turn(self):
        g = room()
        wps, lanes = lp.plan(g, RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=6, width=5)
        self.assertGreaterEqual(len(lanes), 3)
        for x, y, _ in wps:
            self.assertGreaterEqual(clearance_at(g, x, y), lp.TURN_CLEARANCE - RES, (x, y))

    def test_lanes_keep_clear_of_the_walls(self):
        g = room()
        _, lanes = lp.plan(g, RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=6, width=5)
        for a, b in lanes:
            for s in np.linspace(0, 1, 20):
                p = np.array(a) + s * (np.array(b) - np.array(a))
                self.assertGreaterEqual(clearance_at(g, *p), lp.CLEARANCE - RES)

    def test_narrow_corridor_keeps_only_the_lanes_the_arm_can_turn_in(self):
        g = room(8.0, 3.0)            # 2.8 m of floor: only the middle 0.6 m has 1.1 m all round
        _, lanes = lp.plan(g, RES, (0.0, 0.0), (1.2, 1.5, 0.0), length=6, width=2.4)
        self.assertEqual(len(lanes), 1)

    def test_lanes_run_along_the_robot_heading(self):
        _, lanes = lp.plan(room(), RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=4, width=1.6)
        for a, b in lanes:
            self.assertAlmostEqual(a[1], b[1], places=6)

    def test_corners_face_the_next_leg(self):
        wps, _ = lp.plan(room(), RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=4, width=1.6)
        for (x0, y0, yaw), (x1, y1, _) in zip(wps, wps[1:]):
            self.assertAlmostEqual(yaw, math.atan2(y1 - y0, x1 - x0), places=6)

    def test_a_pillar_cuts_the_lane_to_its_longest_free_stretch(self):
        g = room(10.0, 6.0)
        g[56:64, 76:84] = 100                            # 0.4 m pillar at x 3.8-4.2, y 2.8-3.2
        _, lanes = lp.plan(g, RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=8, width=0.0)
        (a, b), = lanes
        self.assertGreater(min(a[0], b[0]), 4.2 + lp.TURN_CLEARANCE - RES)   # past the pillar, room to turn

    def test_unknown_floor_is_left_out(self):
        g = room()
        g[:, 100:] = -1                                  # nothing seen beyond x = 5 m
        wps, _ = lp.plan(g, RES, (0.0, 0.0), (1.2, 3.0, 0.0), length=6, width=2)
        self.assertLessEqual(max(x for x, _, _ in wps), 5.0 - lp.TURN_CLEARANCE + RES)

    def test_nearest_obstacle_ignores_unknown(self):
        g = np.full((100, 100), -1, np.int16)
        g[50, 80] = 100                                  # one wall cell 1.5 m to the right of (2.5, 2.5)
        self.assertAlmostEqual(lp.nearest_obstacle(g, RES, (0.0, 0.0), (2.5, 2.5)), 1.5, places=1)

    def test_no_room_no_lanes(self):
        g = np.full((40, 40), 100, np.int16)
        self.assertEqual(lp.plan(g, RES, (0.0, 0.0), (1.0, 1.0, 0.0)), ([], []))


if __name__ == "__main__":
    unittest.main()
