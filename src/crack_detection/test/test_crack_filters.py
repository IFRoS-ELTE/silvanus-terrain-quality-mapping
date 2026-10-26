"""The two false-crack filters: straight lines (tile grout) and depth steps (kerbs)."""

import unittest

import cv2
import numpy as np

from crack_detection.crack_filters import step_edges_in_color, straight_lines


class StraightLineTest(unittest.TestCase):
    def test_grout_line_is_removed_and_wiggly_crack_kept(self):
        mask = np.zeros((480, 640), np.uint8)
        cv2.line(mask, (20, 40), (600, 380), 255, 5)                      # grout: long and straight
        xs = np.arange(100, 400, 2)
        ys = (300 + 25 * np.sin(xs / 9.0)).astype(int)                     # crack: wiggles every few cm
        cv2.polylines(mask, [np.stack([xs, ys], 1).astype(np.int32)], False, 255, 4)
        hit = straight_lines(mask)
        self.assertGreater(hit[mask > 0].mean(), 0.0)
        line_px = np.zeros_like(mask); cv2.line(line_px, (20, 40), (600, 380), 255, 5)
        crack_px = (mask > 0) & ~(line_px > 0)
        self.assertGreater(hit[line_px > 0].mean(), 0.95)
        self.assertLess(hit[crack_px].mean(), 0.2)

    def test_empty_mask(self):
        self.assertFalse(straight_lines(np.zeros((48, 64), np.uint8)).any())


class StepEdgeTest(unittest.TestCase):
    def test_step_projects_to_the_matching_colour_columns(self):
        depth = np.full((480, 848), 0.80, np.float32)
        depth[:, 424:] = 0.72                                              # an 8 cm step, half way across
        intr = (423.0, 423.0, 424.0, 240.0)
        identity = np.eye(4)
        step = step_edges_in_color(depth, intr, intr, identity, (480, 848), grow=0)
        cols = np.nonzero(step.any(axis=0))[0]
        self.assertTrue(len(cols))
        self.assertTrue(np.all(np.abs(cols - 424) <= 3))

    def test_flat_tilted_ground_has_no_step(self):
        rows = np.linspace(0.75, 0.85, 480, dtype=np.float32)[:, None]     # tilted plane, about 0.2 mm per row
        depth = np.repeat(rows, 848, axis=1)
        intr = (423.0, 423.0, 424.0, 240.0)
        self.assertFalse(step_edges_in_color(depth, intr, intr, np.eye(4), (480, 848)).any())


if __name__ == "__main__":
    unittest.main()
