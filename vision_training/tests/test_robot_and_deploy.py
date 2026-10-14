import json
import sys
from pathlib import Path

import numpy as np
import pytest
import torch
from PIL import Image

from crackseg.data.converters import convert_robot
from crackseg.data.dataset import MEAN, STD
from crackseg.data.splits import assign_splits
from crackseg.inference import predict_image
from crackseg.models import UNet

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"


def _write_frame(folder: Path, name: str, with_mask: bool):
    folder.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.full((48, 64, 3), 100, np.uint8)).save(folder / f"{name}.png")
    if with_mask:
        mask = np.zeros((48, 64), np.uint8)
        mask[20, 5:60] = 255
        Image.fromarray(mask).save(folder / f"{name}_mask.png")


def test_robot_frames_grouped_by_bag_and_unmasked_frames_are_negatives(tmp_path):
    raw = tmp_path / "raw" / "robot"
    _write_frame(raw / "garden_1618", "garden_1618_0001_1.0", with_mask=True)
    _write_frame(raw / "garden_1618", "garden_1618_0002_2.0", with_mask=False)
    _write_frame(raw / "corridor_1338", "corridor_1338_0001_1.0", with_mask=False)
    (raw / "splits.yaml").write_text("garden_1618: train\ncorridor_1338: test\n")

    samples = convert_robot(raw, tmp_path / "processed")
    assert len(samples) == 3
    assert {s.group for s in samples} == {"garden_1618", "corridor_1338"}

    negative = next(s for s in samples if s.sample_id.endswith("0002_2.0"))
    assert np.array(Image.open(tmp_path / "processed" / negative.mask)).max() == 0

    assign_splits(samples, val_fraction=0.0)
    assert {s.split for s in samples if s.group == "corridor_1338"} == {"test"}


def test_onnx_wrapper_matches_pytorch(tmp_path):
    pytest.importorskip("onnxruntime")
    sys.path.insert(0, str(DEPLOY_DIR))
    from crack_onnx import CrackSegmenter

    torch.manual_seed(0)
    model = UNet(base_channels=4).eval()
    onnx_path = tmp_path / "m.onnx"
    torch.onnx.export(model, torch.randn(1, 3, 64, 64), onnx_path,
                      input_names=["image"], output_names=["logits"],
                      dynamic_axes={"image": {2: "h", 3: "w"}, "logits": {2: "h", 3: "w"}},
                      opset_version=17, dynamo=False)
    (tmp_path / "m.json").write_text(json.dumps(
        {"mean": MEAN.tolist(), "std": STD.tolist(), "size_multiple": 16, "threshold": 0.5}))

    # odd size on purpose, so the padding path is used
    rgb = np.random.default_rng(0).integers(0, 255, (50, 70, 3), dtype=np.uint8)
    expected = predict_image(model, rgb, torch.device("cpu"))

    seg = CrackSegmenter(onnx_path)
    prob_rgb, _ = seg.predict(rgb)
    prob_bgr, _ = seg.predict(rgb[..., ::-1].copy(), bgr=True)
    assert prob_rgb.shape == (50, 70)
    assert np.abs(prob_rgb - expected).max() < 1e-4
    assert np.abs(prob_bgr - expected).max() < 1e-4

    ground = np.ones((50, 70), np.uint8)
    ground[40:] = 0  # robot body at the bottom
    prob_masked, mask = seg.predict(rgb, ground_mask=ground)
    assert prob_masked[40:].max() == 0 and mask[40:].max() == 0
