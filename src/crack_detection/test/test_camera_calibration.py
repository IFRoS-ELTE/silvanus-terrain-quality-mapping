import unittest

import numpy as np

from crack_detection.camera_calibration import find_transform, project_depth_points, transform_matrix, transform_points


class CameraCalibrationTest(unittest.TestCase):
    def test_legacy_optical_frames_resolve_to_positive_15_mm(self):
        optical = transform_matrix((0, 0, 0), (-0.5, 0.5, -0.5, 0.5))
        edges = {
            ("link", "depth_optical"): optical,
            ("link", "color"): transform_matrix((0, 0.015, 0), (0, 0, 0, 1)),
            ("color", "color_optical"): optical,
        }
        resolved = find_transform(edges, "depth_optical", "color_optical")
        expected = np.eye(4)
        expected[0, 3] = 0.015
        np.testing.assert_allclose(resolved, expected, atol=1e-12)
        np.testing.assert_allclose(find_transform(edges, "color_optical", "depth_optical"),
                                   np.linalg.inv(expected), atol=1e-12)
        self.assertIsNone(find_transform(edges, "unknown", "color_optical"))

    def test_recorded_rotation_changes_projection(self):
        matrix = transform_matrix((0.01, 0, 0), (0, np.sin(0.01), 0, np.cos(0.01)))
        points = np.array([[0, 0, 1], [0, 0, -1], [np.nan, 0, 1]])
        u, v, visible = project_depth_points(points, (600, 600, 320, 240), matrix)
        expected_x = np.sin(0.02) + 0.01
        expected_z = np.cos(0.02)
        self.assertAlmostEqual(u[0], 600 * expected_x / expected_z + 320)
        self.assertAlmostEqual(v[0], 240)
        self.assertEqual(visible.tolist(), [True, False, False])
        self.assertTrue(np.isnan(u[1:]).all())

    def test_transform_does_not_modify_source_points(self):
        points = np.array([[0.0, 0.0, 1.0]])
        transformed = transform_points(points, transform_matrix((0.015, 0, 0), (0, 0, 0, 1)))
        np.testing.assert_array_equal(points, [[0, 0, 1]])
        np.testing.assert_allclose(transformed, [[0.015, 0, 1]])


if __name__ == "__main__":
    unittest.main()
