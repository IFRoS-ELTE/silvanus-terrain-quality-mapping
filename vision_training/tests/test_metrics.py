import numpy as np
import pytest

from crackseg.metrics import SegmentationMeter


def test_perfect_prediction():
    gt = np.zeros((32, 32))
    gt[10:12, :] = 1
    meter = SegmentationMeter([0.5])
    meter.update(gt.astype(float), gt)
    m = meter.compute(0.5)
    assert m["dice"] == pytest.approx(1.0)
    assert m["iou"] == pytest.approx(1.0)
    assert m["tol_f1"] == pytest.approx(1.0)


def test_one_pixel_shift_hurts_strict_but_not_tolerant():
    # A crack drawn one pixel off is the typical annotation disagreement.
    gt = np.zeros((32, 32))
    gt[10, :] = 1
    pred = np.zeros((32, 32))
    pred[11, :] = 1
    meter = SegmentationMeter([0.5], tolerance_px=2)
    meter.update(pred, gt)
    m = meter.compute(0.5)
    assert m["dice"] == pytest.approx(0.0)
    assert m["tol_f1"] == pytest.approx(1.0)


def test_empty_image_with_empty_prediction_counts_as_correct():
    meter = SegmentationMeter([0.5])
    meter.update(np.zeros((8, 8)), np.zeros((8, 8)))
    assert meter.compute(0.5)["mean_image_dice"] == 1.0


def test_best_threshold_is_picked():
    gt = np.zeros((16, 16))
    gt[4:8, 4:8] = 1
    probs = gt * 0.45  # only a low threshold catches these pixels
    meter = SegmentationMeter([0.3, 0.5, 0.7])
    meter.update(probs, gt)
    assert meter.best("dice")["threshold"] == 0.3
