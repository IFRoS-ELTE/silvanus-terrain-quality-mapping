"""The lane planner on made-up rooms, without ROS: python3 -m pytest navigation/test_lane_planner.py"""
import math
import unittest

import numpy as np

import lane_planner as lp

RES = 0.05


def room(w=6.0, h=4.0):
    """Free floor w x h m with a 0.1 m wall all round, origin at the outer corner."""
    g = np.zeros((int(h / RES), int(w / RES)), np.int16)
    g[:2, :] = g[-2:, :] = g[:, :2] = g[:, -2:] = 100
    return g


class PlanTest(unittest.TestCase):
    def test_lanes_keep_clear_of_the_walls(self):
        g = room()
        wps, lanes = lp.plan(g, RES, (0.0, 0.0), (0.8, 2.0, 0.0), length=6, width=5)
        self.assertGreaterEqual(len(lanes), 3)
        for x, y, _ in wps:
            self.assertTrue(0.6 <= x <= 5.4 and 0.6 <= y <= 3.4, (x, y))

    def test_lanes_run_along_the_robot_heading(self):
        wps, lanes = lp.plan(room(), RES, (0.0, 0.0), (0.8, 2.0, 0.0), length=4, width=1.6)
        for a, b in lanes:
            self.assertAlmostEqual(a[1], b[1], places=6)

    def test_corners_face_the_next_leg(self):
        wps, _ = lp.plan(room(), RES, (0.0, 0.0), (0.8, 2.0, 0.0), length=4, width=1.6)
        for (x0, y0, yaw), (x1, y1, _) in zip(wps, wps[1:]):
            self.assertAlmostEqual(yaw, math.atan2(y1 - y0, x1 - x0), places=6)

    def test_a_pillar_cuts_the_lane_to_its_longest_free_stretch(self):
        g = room(8.0, 4.0)
        g[36:44, 60:68] = 100                            # 0.4 m pillar at x 3.0-3.4, y 1.8-2.2
        _, lanes = lp.plan(g, RES, (0.0, 0.0), (0.8, 2.0, 0.0), length=7, width=0.0)
        (a, b), = lanes
        self.assertGreater(min(a[0], b[0]), 3.4)        # the longer stretch, past the pillar

    def test_unknown_floor_is_left_out(self):
        g = room()
        g[:, 60:] = -1                                   # nothing seen beyond x = 3 m
        wps, _ = lp.plan(g, RES, (0.0, 0.0), (0.8, 2.0, 0.0), length=6, width=2)
        self.assertLessEqual(max(x for x, _, _ in wps), 3.0 - 0.6 + RES)

    def test_no_room_no_lanes(self):
        g = np.full((40, 40), 100, np.int16)
        self.assertEqual(lp.plan(g, RES, (0.0, 0.0), (1.0, 1.0, 0.0)), ([], []))


if __name__ == "__main__":
    unittest.main()
