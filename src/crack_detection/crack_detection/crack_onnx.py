"""Run the exported crack U-Net with ONNX Runtime. No PyTorch needed.

This file is meant to be copied as-is into the ROS crack node package.
It only needs numpy and onnxruntime (plus opencv if you use the mask
helper), which keeps the robot Docker image small.

    from crack_onnx import CrackSegmenter
    seg = CrackSegmenter("crack_unet16_v1.onnx")          # reads crack_unet16_v1.json next to it
    prob, mask = seg.predict(frame_bgr, bgr=True)          # cv_bridge gives bgr8

prob is the crack probability per pixel (float32, same size as the frame),
mask is prob >= threshold as uint8 0/255. The threshold comes from the
JSON file, where it was chosen on validation data.

Three details that are easy to get wrong, all handled here:
- The model was trained on RGB. cv_bridge and OpenCV give BGR.
- Inputs are normalised with the ImageNet mean and std from the JSON.
- Height and width must be multiples of 16; we pad and crop back.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnxruntime as ort


class CrackSegmenter:
    def __init__(self, onnx_path: str | Path, meta_path: str | Path | None = None,
                 threshold: float | None = None, num_threads: int = 0):
        onnx_path = Path(onnx_path)
        meta_path = Path(meta_path) if meta_path else onnx_path.with_suffix(".json")
        meta = json.loads(meta_path.read_text())

        self.mean = np.array(meta["mean"], dtype=np.float32)
        self.std = np.array(meta["std"], dtype=np.float32)
        self.multiple = int(meta["size_multiple"])
        self.threshold = float(threshold if threshold is not None else meta["threshold"])

        options = ort.SessionOptions()
        if num_threads:
            # On the robot the depth node and the rest of the stack also need
            # CPU, so it can make sense to cap the threads used here.
            options.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(str(onnx_path), options,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

    def predict(self, image: np.ndarray, bgr: bool = False,
                ground_mask: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """image: (H, W, 3) uint8. ground_mask: optional (H, W), non-zero where
        the ground is visible; pixels of the robot body are set to 0 in both
        outputs, so the gripper and bumper can never produce a detection."""
        rgb = image[..., ::-1] if bgr else image
        h, w = rgb.shape[:2]

        x = (rgb.astype(np.float32) / 255.0 - self.mean) / self.std
        x = x.transpose(2, 0, 1)[None]

        pad_h = (self.multiple - h % self.multiple) % self.multiple
        pad_w = (self.multiple - w % self.multiple) % self.multiple
        if pad_h or pad_w:
            # reflect, same as in training, so the border does not look like an edge
            x = np.pad(x, ((0, 0), (0, 0), (0, pad_h), (0, pad_w)), mode="reflect")

        logits = self.session.run(None, {self.input_name: np.ascontiguousarray(x)})[0]
        prob = 1.0 / (1.0 + np.exp(-logits[0, 0, :h, :w]))

        if ground_mask is not None:
            prob = np.where(ground_mask > 0, prob, 0.0).astype(np.float32)

        mask = (prob >= self.threshold).astype(np.uint8) * 255
        return prob, mask


def load_ground_mask(path: str | Path, width: int, height: int) -> np.ndarray:
    """Load a mask PNG (white = ground, black = robot) and fit it to a frame size.

    Nearest-neighbour resize keeps it strictly 0/255. Only valid if the mask
    was drawn on the same camera stream: the D435 colour and depth sensors
    have different fields of view, so a depth mask cannot be reused for colour.
    """
    import cv2

    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(path)
    if mask.shape != (height, width):
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
    return mask
