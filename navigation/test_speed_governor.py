"""The governor's decision, without ROS: python3 -m pytest navigation/test_speed_governor.py"""
import math
import unittest

import numpy as np

from speed_governor import credit_jolt, decide, inside_robot, limit_command, nearest_ahead

P = dict(max_speed=0.5, confirm_speed=0.20, stop_height_m=0.10, stop_distance_m=0.35, stop_range_m=0.40,
         slow_ahead_m=1.6)


def hz(h, bx, by=0.0, half_len=0.1):
    return dict(name=f"bump {h * 1000:.0f} mm", h=h, bx=bx, by=by, half_len=half_len)


class DecideTest(unittest.TestCase):
    def test_free_without_hazards(self):
        self.assertEqual(decide([], math.inf, P)[:2], (0.5, "free"))

    def test_small_bump_ahead_slows_to_confirm_speed(self):
        self.assertEqual(decide([hz(0.02, 1.2)], math.inf, P)[:2], (0.20, "slow"))

    def test_slow_until_rear_wheels_are_past(self):
        self.assertEqual(decide([hz(0.02, -0.2)], math.inf, P)[:2], (0.20, "slow"))   # between the axles

    def test_tall_bump_just_ahead_stops(self):
        limit, mode, why = decide([hz(0.12, 0.249 + 0.1 + 0.2)], math.inf, P)          # 0.2 m from the front wheels
        self.assertEqual((limit, mode), (0.0, "stop"))
        self.assertIn("0.20 m ahead", why)

    def test_tall_bump_far_ahead_only_slows(self):
        self.assertEqual(decide([hz(0.12, 1.4)], math.inf, P)[:2], (0.20, "slow"))

    def test_hazard_beside_the_track_is_ignored(self):
        self.assertEqual(decide([hz(0.12, 0.6, by=0.8)], math.inf, P)[:2], (0.5, "free"))

    def test_lidar_obstacle_stops(self):
        self.assertEqual(decide([], 0.3, P)[:2], (0.0, "stop"))

    def test_deep_hole_stops_like_a_bump(self):
        self.assertEqual(decide([dict(name="hole 150 mm", h=0.15, bx=0.55, by=0.1, half_len=0.1)], math.inf, P)[1], "stop")


class NearestAheadTest(unittest.TestCase):
    A = np.radians(np.arange(-180, 180, 0.5))

    def scan_of(self, points):
        r = np.full(len(self.A), np.inf)
        for x, y in points:
            r[np.argmin(np.abs(self.A - np.arctan2(y, x)))] = np.hypot(x, y)
        return r

    def test_wall_straight_ahead(self):
        self.assertAlmostEqual(nearest_ahead(self.scan_of([(0.85, 0.0)]), self.A), 0.38, places=2)

    def test_corridor_wall_diagonally_beside_is_not_in_the_way(self):
        # 0.82 m ahead but 0.57 m to the side: the old +-35 deg window called this 0.35 m ahead
        self.assertEqual(nearest_ahead(self.scan_of([(0.82, 0.57), (0.82, -0.6)]), self.A), math.inf)

    def test_object_at_the_wheel(self):
        self.assertLess(nearest_ahead(self.scan_of([(0.8, 0.4)]), self.A), 0.40)


class CreditJoltTest(unittest.TestCase):
    def test_pieces_of_one_plate_share_its_jolt(self):
        used = []
        self.assertEqual(credit_jolt(True, 13.5, dict(id="G30", x=1.0, y=0.0), used), (True, None))
        self.assertEqual(credit_jolt(True, 13.5, dict(id="G27", x=1.2, y=0.1), used), (True, "G30"))

    def test_far_hazard_cannot_reuse_the_jolt(self):
        used = []
        credit_jolt(True, 5.0, dict(id="G1", x=0.0, y=0.0), used)
        self.assertEqual(credit_jolt(True, 5.1, dict(id="G2", x=0.0, y=1.0), used), (False, None))

    def test_weak_jolt_is_not_felt(self):
        self.assertEqual(credit_jolt(False, 5.0, dict(id="G1", x=0.0, y=0.0), []), (False, None))


class LimitCommandTest(unittest.TestCase):
    def test_slowing_keeps_the_curve(self):
        v, w = limit_command(0.35, 0.35, 0.20)
        self.assertAlmostEqual(v, 0.20)
        self.assertAlmostEqual(w / v, 1.0)

    def test_below_the_limit_passes(self):
        self.assertEqual(limit_command(0.15, 0.4, 0.20), (0.15, 0.4))

    def test_stop_may_still_turn_on_the_spot(self):
        self.assertEqual(limit_command(0.3, 0.2, 0.0), (0.0, 0.2))

    def test_reversing_passes(self):
        self.assertEqual(limit_command(-0.2, 0.1, 0.0), (-0.2, 0.1))


class InsideRobotTest(unittest.TestCase):
    def test_front_of_the_body_is_the_robot(self):
        self.assertTrue(inside_robot(0.43, -0.19))       # the 273 mm "bump" seen live with the arm tucked

    def test_just_ahead_of_the_bumper_is_real(self):
        self.assertFalse(inside_robot(0.55, 0.0))

    def test_beside_the_robot_is_real(self):
        self.assertFalse(inside_robot(0.2, 0.5))


if __name__ == "__main__":
    unittest.main()
