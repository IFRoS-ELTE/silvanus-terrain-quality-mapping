"""Check the exported model's input/output contract without an ONNX dependency."""

import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

from crack_detection.crack_onnx import CrackSegmenter


class FakeSession:
    def __init__(self, *args, **kwargs):
        self.input_tensor = None

    def get_inputs(self):
        return [types.SimpleNamespace(name="image", type="tensor(float)", shape=["batch", 3, "h", "w"])]

    def get_outputs(self):
        return [types.SimpleNamespace(name="logits", shape=["batch", 1, "h", "w"])]

    def run(self, names, feeds):
        self.input_tensor = feeds["image"]
        h, w = self.input_tensor.shape[-2:]
        logits = np.full((1, 1, h, w), -80, np.float32)
        logits[0, 0, 0, :4] = [math.log(0.79 / 0.21), math.log(0.81 / 0.19), 80, -80]
        return [logits]


class UnetDetectorTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.model_path = Path(self.directory.name) / "model.onnx"
        self.model_path.write_bytes(b"test model; fake session")
        self.metadata = {
            "mean": [0.485, 0.456, 0.406], "std": [0.229, 0.224, 0.225],
            "size_multiple": 16, "threshold": 0.8,
        }
        self.config_path = self.model_path.with_suffix(".json")
        self.config_path.write_text(json.dumps(self.metadata))
        fake_runtime = types.SimpleNamespace(SessionOptions=types.SimpleNamespace, InferenceSession=FakeSession)
        self.runtime_patch = patch("crack_detection.crack_onnx.ort", fake_runtime)
        self.runtime_patch.start()
        self.addCleanup(self.runtime_patch.stop)

    def detector(self, **kwargs):
        return CrackSegmenter(str(self.model_path), **kwargs)

    def test_rgb_normalization_and_native_resolution(self):
        detector = self.detector()
        image = np.zeros((32, 48, 3), np.uint8)
        image[:] = [0, 127, 255]  # BGR
        original = image.copy()
        detector.predict(image, bgr=True)
        tensor = detector.session.input_tensor
        self.assertEqual(tensor.shape, (1, 3, 32, 48))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertTrue(tensor.flags.c_contiguous)
        np.testing.assert_allclose(tensor[0, :, 0, 0],
                                   [(1 - 0.485) / 0.229, (127 / 255 - 0.456) / 0.224, -0.406 / 0.225],
                                   rtol=1e-6)
        np.testing.assert_array_equal(image, original)

    def test_reflect_padding_and_output_crop(self):
        detector = self.detector()
        image = np.random.default_rng(3).integers(0, 256, (17, 19, 3), dtype=np.uint8)
        probability, mask = detector.predict(image, bgr=True)
        tensor = detector.session.input_tensor
        self.assertEqual(tensor.shape, (1, 3, 32, 32))
        np.testing.assert_array_equal(tensor[0, :, 17, 0], tensor[0, :, 15, 0])
        np.testing.assert_array_equal(tensor[0, :, 0, 19], tensor[0, :, 0, 17])
        self.assertEqual(mask.shape, (17, 19))

    def test_sigmoid_saved_threshold_and_thin_cracks(self):
        probability, mask = self.detector().predict(np.zeros((32, 48, 3), np.uint8))
        self.assertEqual(mask.dtype, np.uint8)
        self.assertEqual(mask[0, :4].tolist(), [0, 255, 255, 0])
        self.assertEqual(np.count_nonzero(mask), 2)  # No erosion of thin pixels.
        _, overridden = self.detector(threshold=0.75).predict(np.zeros((32, 48, 3), np.uint8))
        self.assertEqual(overridden[0, :4].tolist(), [255, 255, 255, 0])

    def test_colour_robot_mask_zeros_probabilities_before_boxes(self):
        ground = np.full((32, 48), 255, np.uint8)
        ground[0, 1:3] = 0
        probability, mask = self.detector().predict(np.zeros((32, 48, 3), np.uint8),
                                                  ground_mask=ground)
        np.testing.assert_array_equal(probability[0, 1:3], 0)
        np.testing.assert_array_equal(mask[0, 1:3], 0)
        self.assertEqual(np.count_nonzero(mask), 0)


if __name__ == "__main__":
    unittest.main()
