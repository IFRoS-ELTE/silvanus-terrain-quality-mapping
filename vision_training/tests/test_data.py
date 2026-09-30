import numpy as np
from PIL import Image

from crackseg.data.converters import Sample, convert_mask_suffix_folders
from crackseg.data.dataset import CrackDataset
from crackseg.data.splits import assign_splits, read_index, write_index
from crackseg.data.transforms import TrainTransform


def _fake_samples(n_groups, per_group, source="own"):
    return [
        Sample(f"{source}_{g}_{i}", source, f"img_{g}_{i}.png", f"mask_{g}_{i}.png",
               split=None, group=f"session{g}")
        for g in range(n_groups) for i in range(per_group)
    ]


def test_groups_never_cross_splits():
    samples = assign_splits(_fake_samples(20, 5), val_fraction=0.2, test_fraction=0.2)
    split_of_group = {}
    for s in samples:
        assert split_of_group.setdefault(s.group, s.split) == s.split
    assert {s.split for s in samples} == {"train", "val", "test"}


def test_official_test_split_is_kept():
    samples = _fake_samples(10, 1)
    for s in samples[:3]:
        s.split = "test"
    for s in samples[3:]:
        s.split = "train"
    assign_splits(samples, val_fraction=0.2)
    assert [s.split for s in samples[:3]] == ["test"] * 3
    assert any(s.split == "val" for s in samples[3:])


def test_train_crops_of_a_test_photo_are_dropped():
    samples = [
        Sample("a1", "ds", "a1.png", "a1m.png", "test", "photoA"),
        Sample("a2", "ds", "a2.png", "a2m.png", "train", "photoA"),
        Sample("b1", "ds", "b1.png", "b1m.png", "train", "photoB"),
    ]
    assign_splits(samples, val_fraction=0.0)
    assert samples[1].split == "unused"
    assert samples[2].split == "train"


def test_splits_are_reproducible():
    a = [s.split for s in assign_splits(_fake_samples(30, 1), seed=1)]
    b = [s.split for s in assign_splits(_fake_samples(30, 1), seed=1)]
    assert a == b


def test_train_transform_output_shape():
    rng = np.random.default_rng(0)
    image = np.zeros((100, 150, 3), np.uint8)  # smaller than the crop on one side
    mask = np.zeros((100, 150), np.uint8)
    mask[50, :] = 255
    img_out, mask_out = TrainTransform(128)(image, mask, rng)
    assert img_out.shape == (128, 128, 3)
    assert mask_out.shape == (128, 128)


def test_convert_and_load_roundtrip(tmp_path):
    raw = tmp_path / "raw" / "own"
    (raw / "train").mkdir(parents=True)
    for i in range(3):
        Image.fromarray(np.full((40, 60, 3), 120, np.uint8)).save(raw / "train" / f"f{i}.jpg")
        mask = np.zeros((40, 60), np.uint8)
        mask[20, 10:50] = 1  # 0/1 labels on purpose, the converter must handle them
        Image.fromarray(mask).save(raw / "train" / f"f{i}_mask.png")

    processed = tmp_path / "processed"
    samples = convert_mask_suffix_folders(raw, processed, "own", {"train": "train"})
    assert len(samples) == 3
    write_index(samples, processed / "index.csv")
    assert len(read_index(processed / "index.csv", split="train")) == 3

    ds = CrackDataset(processed, "train")
    image, mask, sid = ds[0]
    assert image.shape == (3, 40, 60)
    assert mask.shape == (1, 40, 60)
    assert mask.sum() == 40
