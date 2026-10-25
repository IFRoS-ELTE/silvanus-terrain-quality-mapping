"""Image, detection, and geometric contracts used by the live crack node."""

import unittest
import json
from threading import Lock
from collections import deque

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

from crack_detection.crack_bbox_node import CrackBBoxNode, boxes_from_mask, color_bgr, decode_image, detection_message, image_message, point_cloud_message
from crack_detection.crack_projection import crack_surface_points


class CrackNodeDataTest(unittest.TestCase):
    def test_rgb_rows_with_padding_and_source_header(self):
        header = Header(frame_id="rgb_optical")
        header.stamp.sec, header.stamp.nanosec = 123, 456
        message = Image(header=header, height=2, width=1, encoding="rgb8", step=4,
                        data=bytes([255, 0, 0, 99, 0, 255, 0, 99]))
        bgr = color_bgr(message)
        np.testing.assert_array_equal(bgr, [[[0, 0, 255]], [[0, 255, 0]]])
        out = image_message(bgr, header, "bgr8")
        self.assertEqual(out.header, header)
        self.assertEqual(out.step, 3)
        self.assertEqual(len(out.data), 6)
        self.assertEqual(bytes(message.data), bytes([255, 0, 0, 99, 0, 255, 0, 99]))

    def test_depth_endianness_and_bad_buffers(self):
        message = Image(height=1, width=2, encoding="16UC1", step=6, is_bigendian=1,
                        data=bytes([3, 232, 7, 208, 0, 0]))
        np.testing.assert_array_equal(decode_image(message), [[1000, 2000]])
        message.step = 2
        with self.assertRaises(ValueError):
            decode_image(message)

    def test_boxes_have_pixel_coordinates_and_mask_confidence(self):
        mask = np.zeros((8, 10), np.uint8)
        mask[3, 4:6] = 255
        probability = np.full(mask.shape, 0.1, np.float32)
        probability[3, 4:6] = [0.85, 0.95]
        header = Header(frame_id="rgb_optical")
        header.stamp.sec, header.stamp.nanosec = 123, 456
        boxes = boxes_from_mask(mask, probability, min_area=2)
        payload = json.loads(detection_message(boxes, header).data)
        self.assertEqual(set(payload), {"stamp", "frame_id", "count", "boxes"})
        self.assertEqual(payload["stamp"], {"sec": 123, "nanosec": 456})
        self.assertEqual(payload["frame_id"], "rgb_optical")
        self.assertEqual(payload["count"], 1)
        self.assertEqual(payload["boxes"], [{"x": 4, "y": 3, "width": 2, "height": 1,
                                             "area": 2.0, "score": 0.9}])
        self.assertEqual(boxes_from_mask(mask, probability, min_area=3), [])
        self.assertEqual(json.loads(detection_message([], header).data)["count"], 0)

    def test_projection_handles_different_resolutions_and_translation(self):
        # Depth pixel (1,1) at one metre has XYZ (0,0,1).
        depth = np.ones((3, 3), np.float32)
        mask = np.zeros((7, 8), np.uint8)
        mask[3, 4] = 255
        transform = np.eye(4)
        transform[0, 3] = 0.5  # fx=2 translates the RGB projection by one pixel.
        points = crack_surface_points(mask, depth, (1, 1, 1, 1), (2, 2, 3, 3), transform,
                                      stride=1, zmin=0.3, zmax=1.6)
        np.testing.assert_allclose(points, [[0, 0, 1]])

    def test_projection_rejects_bad_depth_and_robot_pixels(self):
        depth = np.array([[0, np.nan, np.inf], [0.1, 1, 2], [0, 0, 0]], np.float32)
        mask = np.full((3, 3), 255, np.uint8)
        valid = np.ones((3, 3), bool)
        valid[1, 1] = False
        points = crack_surface_points(mask, depth, (1, 1, 1, 1), (1, 1, 1, 1), np.eye(4),
                                      valid_mask=valid, stride=1)
        self.assertEqual(points.shape, (0, 3))

    def test_cloud_point_limit_and_serialization(self):
        points = crack_surface_points(np.full((5, 5), 255, np.uint8), np.ones((5, 5), np.float32),
                                      (1, 1, 2, 2), (1, 1, 2, 2), np.eye(4), stride=1, max_points=3)
        self.assertEqual(points.shape, (3, 3))
        header = Header(frame_id="depth_optical")
        cloud = point_cloud_message(points, header)
        self.assertEqual(cloud.header, header)
        self.assertEqual(cloud.width, 3)
        self.assertEqual(cloud.point_step, 16)
        values = np.frombuffer(cloud.data, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        np.testing.assert_allclose(values["z"], 1)
        np.testing.assert_array_equal(values["rgb"], 0xFFFF00)
        empty = point_cloud_message(np.empty((0, 3), np.float32), header)
        self.assertEqual(empty.width, 0)
        self.assertEqual(len(empty.data), 0)

    def test_timestamp_pairing_and_map_transform(self):
        # Exercise projection without starting a DDS participant or faking frame IDs.
        node = object.__new__(CrackBBoxNode)
        node.input_lock = Lock()
        node.options = {"sync_tolerance_s": 0.05, "depth_scale_m": 0.001,
                        "zmin_m": 0.3, "zmax_m": 1.6, "cloud_stride": 1,
                        "max_cloud_points": 5000, "target_frame": "test_map"}
        rgb_header, depth_header = Header(frame_id="rgb"), Header(frame_id="depth")
        rgb_header.stamp.sec = depth_header.stamp.sec = 123
        rgb_header.stamp.nanosec = 10_000_000
        depth = image_message(np.full((3, 3), 1000, np.uint16), depth_header, "16UC1")
        node.depth_frames = deque([depth])
        node.color_info = CameraInfo(header=rgb_header, height=3, width=3,
                                     k=[1., 0., 1., 0., 1., 1., 0., 0., 1.])
        node.depth_info = CameraInfo(header=depth_header, height=3, width=3,
                                     k=[1., 0., 1., 0., 1., 1., 0., 0., 1.])
        node.robot_mask = None
        def lookup(target, source, stamp):
            matrix = np.eye(4)
            if target == "test_map":
                matrix[0, 3] = 5
            return matrix
        node.lookup_matrix = lookup
        mask = np.zeros((3, 3), np.uint8)
        mask[1, 1] = 255
        points, header = node.project_points(mask, rgb_header)
        np.testing.assert_allclose(points, [[5, 0, 1]])
        self.assertEqual(header.frame_id, "test_map")
        self.assertEqual(header.stamp, depth_header.stamp)
        self.assertEqual(depth_header.frame_id, "depth")
        rgb_header.stamp.nanosec = 100_000_000
        with self.assertRaisesRegex(ValueError, "timestamps differ"):
            node.project_points(mask, rgb_header)


if __name__ == "__main__":
    unittest.main()
