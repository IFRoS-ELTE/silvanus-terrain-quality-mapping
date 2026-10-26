"""The governor's decision, without ROS: python3 -m pytest navigation/test_speed_governor.py"""
import math
import unittest

from speed_governor import decide

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


if __name__ == "__main__":
    unittest.main()
