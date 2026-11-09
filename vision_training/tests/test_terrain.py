import json
import sys
from pathlib import Path

import numpy as np
import pytest
import torch
from PIL import Image

from terrainclf.bagframes import colour_frames
from terrainclf.data import Frame, PatchDataset, eval_transform, write_frames
from terrainclf.evaluate import confusion, save_all
from terrainclf.labels import Segment, SurfaceLabels
from terrainclf.model import build_model
from terrainclf.patches import patch_boxes

ROOT = Path(__file__).resolve().parents[1]
CLASSES = ["asphalt", "red_path", "tile"]
COLOURS = {"asphalt": (70, 70, 75), "red_path": (170, 60, 50), "tile": (215, 215, 205)}


def robot_mask():
    mask = np.full((480, 640), 255, np.uint8)
    mask[400:] = 0  # same strip as the committed colour mask
    return mask


def test_labels_drop_frames_near_boundaries():
    labels = SurfaceLabels(CLASSES, {"bag": [Segment(0, 10, "asphalt"), Segment(12, 30, "tile")]}, 2.0)
    assert labels.label_at("bag", 5) == "asphalt"
    assert labels.label_at("bag", 9) is None      # inside the margin before the boundary
    assert labels.label_at("bag", 11) is None     # between segments
    assert labels.label_at("bag", 14) == "tile"
    assert labels.label_at("other_bag", 5) is None


def test_label_file_matches_its_classes():
    labels = SurfaceLabels.from_yaml(ROOT / "configs" / "terrain_labels.yaml")
    assert labels.classes == ["asphalt", "red_path", "wood_chips", "grass_dirt", "tile"]
    assert "garden_surfaces_1628" in labels.bags


def test_patches_sit_above_the_robot_mask():
    boxes = patch_boxes(robot_mask(), size=200, rows=2, cols=3)
    assert len(boxes) == 6
    assert max(y + h for _, y, _, h in boxes) == 400   # lowest patch ends where the robot starts
    assert boxes[0] == (20, 0, 200, 200)


def _write_bag(path: Path, seconds: float, fps: float = 30):
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    Img, Header, Time = (ts.types[n] for n in
                         ("sensor_msgs/msg/Image", "std_msgs/msg/Header", "builtin_interfaces/msg/Time"))
    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conn = writer.add_connection("/camera/camera/color/image_raw", Img.__msgtype__, typestore=ts)
        t0 = 1_790_000_000 * 10**9
        for i in range(int(seconds * fps)):
            t = t0 + int(i * 1e9 / fps)
            # BGR pixels whose blue channel encodes the second, so we can check
            # both the timing and the BGR -> RGB conversion.
            bgr = np.zeros((480, 640, 3), np.uint8)
            bgr[..., 0] = int(i / fps)
            bgr[..., 2] = 200
            msg = Img(header=Header(stamp=Time(sec=t // 10**9, nanosec=t % 10**9), frame_id="cam"),
                      height=480, width=640, encoding="bgr8", is_bigendian=0, step=640 * 3,
                      data=bgr.reshape(-1))
            writer.write(conn, t, ts.serialize_cdr(msg, Img.__msgtype__))


def test_frames_from_a_real_mcap_bag(tmp_path):
    pytest.importorskip("rosbags")
    bag = tmp_path / "garden_test"
    _write_bag(bag, seconds=3)
    frames = list(colour_frames(bag, fps=2))
    times = [round(t, 2) for t, _ in frames]
    assert times == [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
    t, rgb = frames[2]
    assert rgb.shape == (480, 640, 3)
    assert rgb[0, 0, 0] == 200 and rgb[0, 0, 2] == int(t)   # channels swapped to RGB


def _fake_frames(root: Path, bag: str, n: int) -> list[Frame]:
    rng = np.random.default_rng(len(bag))
    out = []
    for i in range(n):
        label = CLASSES[i % len(CLASSES)]
        img = np.clip(np.array(COLOURS[label]) + rng.normal(0, 12, (480, 640, 3)), 0, 255).astype(np.uint8)
        rel = Path("frames") / bag / f"{bag}_{i:03d}.png"
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(img).save(root / rel)
        out.append(Frame(rel.as_posix(), bag, float(i), label))
    return out


def test_training_and_frame_vote_end_to_end(tmp_path):
    sys.path.insert(0, str(ROOT / "scripts"))
    from terrain_train import classify_frames, train_model

    train = _fake_frames(tmp_path, "bag_a", 12)
    test = _fake_frames(tmp_path, "bag_b", 6)
    write_frames(train + test, tmp_path / "index.csv")
    cfg = {"data": {"root": str(tmp_path), "input_size": 64, "num_workers": 0},
           "model": {"backbone": "mobilenet_v3_small", "pretrained": False},
           "train": {"epochs": 3, "batch_size": 16, "lr": 0.003, "weight_decay": 0.0, "label_smoothing": 0.0}}
    boxes = patch_boxes(robot_mask(), 200, 2, 3)

    torch.manual_seed(0)
    model = train_model(cfg, train, CLASSES, boxes, torch.device("cpu"))
    true, pred = classify_frames(model, cfg, test, CLASSES, boxes, torch.device("cpu"))
    assert len(true) == len(pred) == 6

    summary = save_all(confusion(true, pred, len(CLASSES)), CLASSES, tmp_path / "out", "test")
    assert (tmp_path / "out" / "confusion_matrix.png").exists()
    assert summary["per_class"]["tile"]["frames"] == 2
    assert "| tile |" in (tmp_path / "out" / "results.md").read_text()


def test_onnx_wrapper_matches_torch(tmp_path):
    pytest.importorskip("onnxruntime")
    sys.path.insert(0, str(ROOT / "deploy"))
    from terrain_onnx import TerrainClassifier

    torch.manual_seed(0)
    model = build_model("mobilenet_v3_small", len(CLASSES), pretrained=False).eval()
    torch.onnx.export(model, torch.randn(1, 3, 224, 224), tmp_path / "t.onnx",
                      input_names=["patches"], output_names=["logits"],
                      dynamic_axes={"patches": {0: "n"}, "logits": {0: "n"}}, opset_version=17, dynamo=False)
    boxes = patch_boxes(robot_mask(), 200, 2, 3)
    (tmp_path / "t.json").write_text(json.dumps({
        "classes": CLASSES, "patch_boxes_xywh": [list(b) for b in boxes], "frame_size_wh": [640, 480],
        "input_size": 224, "mean": [0.485, 0.456, 0.406], "std": [0.229, 0.224, 0.225]}))

    rgb = np.random.default_rng(1).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    rgb = np.array(Image.fromarray(rgb).resize((160, 120)).resize((640, 480)))  # smooth, like a real photo

    ds = PatchDataset(tmp_path, [Frame("x.png", "b", 0, "tile")], CLASSES, boxes, eval_transform(224))
    Image.fromarray(rgb).save(tmp_path / "x.png")
    with torch.no_grad():
        expected = torch.softmax(model(torch.stack([ds[i][0] for i in range(len(ds))])), 1).mean(0).numpy()

    label, conf, probs = TerrainClassifier(tmp_path / "t.onnx").predict(rgb[..., ::-1].copy(), bgr=True)
    assert np.abs(probs - expected).max() < 0.02    # cv2 and torchvision resize differ very slightly
    assert label == CLASSES[int(expected.argmax())]
