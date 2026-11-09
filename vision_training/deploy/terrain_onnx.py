"""Run the exported terrain classifier with ONNX Runtime. No PyTorch needed.

Meant to be copied into the ROS node package, like crack_onnx.py.

    from terrain_onnx import TerrainClassifier
    clf = TerrainClassifier("rgb_terrain_v1.onnx")        # reads rgb_terrain_v1.json next to it
    label, confidence, probs = clf.predict(frame_bgr, bgr=True)

The frame must be the full 640x480 colour image: the patch positions in the
JSON are pixel boxes in that frame, chosen above the robot mask.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnxruntime as ort


class TerrainClassifier:
    def __init__(self, onnx_path: str | Path, meta_path: str | Path | None = None, num_threads: int = 0):
        onnx_path = Path(onnx_path)
        meta = json.loads(Path(meta_path or onnx_path.with_suffix(".json")).read_text())
        self.classes = meta["classes"]
        self.boxes = meta["patch_boxes_xywh"]
        self.frame_wh = tuple(meta["frame_size_wh"])
        self.size = int(meta["input_size"])
        self.mean = np.array(meta["mean"], np.float32)
        self.std = np.array(meta["std"], np.float32)

        options = ort.SessionOptions()
        if num_threads:
            options.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(str(onnx_path), options, providers=["CPUExecutionProvider"])

    def _prepare(self, patch: np.ndarray) -> np.ndarray:
        import cv2

        # Bilinear, like torchvision's Resize in training (200 px patches are
        # scaled up to 224, so antialiasing plays no part).
        patch = cv2.resize(np.ascontiguousarray(patch), (self.size, self.size), interpolation=cv2.INTER_LINEAR)
        x = (patch.astype(np.float32) / 255.0 - self.mean) / self.std
        return x.transpose(2, 0, 1)

    def predict(self, image: np.ndarray, bgr: bool = False) -> tuple[str, float, np.ndarray]:
        h, w = image.shape[:2]
        if (w, h) != self.frame_wh:
            raise ValueError(f"expected a {self.frame_wh[0]}x{self.frame_wh[1]} frame, got {w}x{h}")
        rgb = image[..., ::-1] if bgr else image
        batch = np.stack([self._prepare(rgb[y:y + bh, x:x + bw]) for x, y, bw, bh in self.boxes])
        logits = self.session.run(None, {"patches": batch})[0]
        e = np.exp(logits - logits.max(axis=1, keepdims=True))
        probs = (e / e.sum(axis=1, keepdims=True)).mean(axis=0)
        best = int(probs.argmax())
        return self.classes[best], float(probs[best]), probs
