"""Segmentation metrics for cracks.

We report two families of numbers:

1. Strict pixel metrics (Dice, IoU, precision, recall). Every pixel must
   match exactly. These are what the evaluation plan in the handover asks
   for, and what we compare between our own models.

2. Tolerant precision, recall and F1. Crack masks are drawn by hand and a
   one pixel shift of a two pixel wide crack already halves the strict
   Dice, even when the prediction is fine for our purpose. The crack
   literature (DeepCrack, CrackForest) therefore counts a predicted pixel
   as correct if a ground truth pixel lies within a few pixels of it, and
   the other way round for recall. We use a 2 pixel tolerance by default.

Counts are summed over the whole dataset before dividing ("micro"
averaging), and we also keep the per-image Dice to look at the spread.
We track several thresholds at once, so the best threshold can be picked
on validation and then used unchanged on test.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import binary_dilation, generate_binary_structure, iterate_structure


def _safe_div(a: float, b: float) -> float:
    return float(a) / float(b) if b > 0 else 0.0


def _disk(radius: int) -> np.ndarray:
    return iterate_structure(generate_binary_structure(2, 1), radius) if radius > 0 else None


@dataclass
class _Counts:
    tp: int = 0
    fp: int = 0
    fn: int = 0
    # tolerant versions
    tol_tp_pred: int = 0  # predicted pixels close to some ground truth
    tol_pred: int = 0
    tol_tp_gt: int = 0  # ground truth pixels close to some prediction
    tol_gt: int = 0
    per_image_dice: list[float] = field(default_factory=list)


class SegmentationMeter:
    def __init__(self, thresholds=(0.3, 0.4, 0.5, 0.6, 0.7), tolerance_px: int = 2):
        self.thresholds = tuple(thresholds)
        self.tolerance_px = tolerance_px
        self._struct = _disk(tolerance_px)
        self.counts = {t: _Counts() for t in self.thresholds}

    def update(self, probs: np.ndarray, target: np.ndarray) -> None:
        """probs and target are (H, W) arrays for one image."""
        gt = target > 0.5
        gt_near = binary_dilation(gt, self._struct) if self._struct is not None else gt

        for t in self.thresholds:
            pred = probs >= t
            c = self.counts[t]
            tp = int(np.logical_and(pred, gt).sum())
            fp = int(np.logical_and(pred, ~gt).sum())
            fn = int(np.logical_and(~pred, gt).sum())
            c.tp += tp
            c.fp += fp
            c.fn += fn

            pred_near = binary_dilation(pred, self._struct) if self._struct is not None else pred
            c.tol_tp_pred += int(np.logical_and(pred, gt_near).sum())
            c.tol_pred += int(pred.sum())
            c.tol_tp_gt += int(np.logical_and(gt, pred_near).sum())
            c.tol_gt += int(gt.sum())

            # An image with no crack and no prediction is a perfect result.
            denom = 2 * tp + fp + fn
            c.per_image_dice.append(1.0 if denom == 0 else 2 * tp / denom)

    def compute(self, threshold: float) -> dict[str, float]:
        c = self.counts[threshold]
        tol_p = _safe_div(c.tol_tp_pred, c.tol_pred)
        tol_r = _safe_div(c.tol_tp_gt, c.tol_gt)
        return {
            "threshold": threshold,
            "dice": _safe_div(2 * c.tp, 2 * c.tp + c.fp + c.fn),
            "iou": _safe_div(c.tp, c.tp + c.fp + c.fn),
            "precision": _safe_div(c.tp, c.tp + c.fp),
            "recall": _safe_div(c.tp, c.tp + c.fn),
            "tol_precision": tol_p,
            "tol_recall": tol_r,
            "tol_f1": _safe_div(2 * tol_p * tol_r, tol_p + tol_r),
            "mean_image_dice": float(np.mean(c.per_image_dice)) if c.per_image_dice else 0.0,
        }

    def compute_all(self) -> list[dict[str, float]]:
        return [self.compute(t) for t in self.thresholds]

    def best(self, key: str = "dice") -> dict[str, float]:
        return max(self.compute_all(), key=lambda m: m[key])
