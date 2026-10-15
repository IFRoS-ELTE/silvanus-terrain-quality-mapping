"""Turn each raw dataset into one common layout.

Every public dataset ships in its own format (PNG masks, MATLAB files,
masks with a suffix in the name, ...). After conversion every dataset
looks the same:

    data/processed/<dataset>/images/<id>.<jpg|png>
    data/processed/<dataset>/masks/<id>.png      (uint8, 0 = background, 255 = crack)

Images are copied without re-encoding so we do not lose JPEG quality
twice. Masks are always rewritten, because each source uses different
label values.

Each converter returns a list of Sample rows. `split` is filled in when
the dataset has an official split, otherwise it stays None and
`splits.py` assigns one.
"""

from __future__ import annotations

import logging
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import scipy.io as sio
from PIL import Image

log = logging.getLogger(__name__)

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass
class Sample:
    sample_id: str
    source: str
    image: str  # path relative to the processed root
    mask: str
    split: str | None = None
    # Samples that share a group always land in the same split. For public
    # data each image is its own group. For our robot frames the group is
    # the recording session, so near-identical frames from one drive never
    # end up in both train and test.
    group: str | None = None


def _binary_mask(mask: np.ndarray) -> np.ndarray:
    """Any value above half range counts as crack. Handles 0/1 and 0/255 masks."""
    threshold = 0 if mask.max() <= 1 else 127
    return np.where(mask > threshold, 255, 0).astype(np.uint8)


def _save_pair(
    image_src: Path, mask: np.ndarray, out_root: Path, source: str, sample_id: str
) -> tuple[str, str]:
    img_rel = Path(source) / "images" / f"{sample_id}{image_src.suffix.lower()}"
    mask_rel = Path(source) / "masks" / f"{sample_id}.png"
    (out_root / img_rel).parent.mkdir(parents=True, exist_ok=True)
    (out_root / mask_rel).parent.mkdir(parents=True, exist_ok=True)

    shutil.copyfile(image_src, out_root / img_rel)
    Image.fromarray(mask).save(out_root / mask_rel)
    return img_rel.as_posix(), mask_rel.as_posix()


def _check_same_size(image_path: Path, mask: np.ndarray) -> bool:
    with Image.open(image_path) as im:
        w, h = im.size
    if mask.shape != (h, w):
        log.warning("size mismatch, skipping %s (image %dx%d, mask %s)",
                    image_path.name, w, h, mask.shape)
        return False
    return True


def convert_deepcrack(raw_dir: Path, out_root: Path) -> list[Sample]:
    samples = []
    for official_split, split_name in (("train", "train"), ("test", "test")):
        img_dir = raw_dir / f"{official_split}_img"
        lab_dir = raw_dir / f"{official_split}_lab"
        for img_path in sorted(img_dir.glob("*.jpg")):
            lab_path = lab_dir / f"{img_path.stem}.png"
            if not lab_path.exists():
                log.warning("deepcrack: no label for %s", img_path.name)
                continue
            mask = _binary_mask(np.array(Image.open(lab_path).convert("L")))
            if not _check_same_size(img_path, mask):
                continue
            sid = f"deepcrack_{img_path.stem}"
            # Names like 11134, 11134-3 and 11134-4 are crops of the same
            # photo. They have to stay in the same split, otherwise the
            # validation score is inflated by near duplicates.
            group = f"deepcrack_{img_path.stem.split('-')[0]}"
            img_rel, mask_rel = _save_pair(img_path, mask, out_root, "deepcrack", sid)
            samples.append(Sample(sid, "deepcrack", img_rel, mask_rel, split_name, group))
    return samples


def convert_crackforest(raw_dir: Path, out_root: Path) -> list[Sample]:
    # The zip from GitHub has one extra folder level (CrackForest-dataset-master/).
    base = next(raw_dir.glob("*/groundTruth")).parent
    samples = []
    for mat_path in sorted((base / "groundTruth").glob("*.mat")):
        img_path = base / "image" / f"{mat_path.stem}.jpg"
        if not img_path.exists():
            continue
        gt = sio.loadmat(mat_path)["groundTruth"][0, 0]
        # Segmentation uses 1 for background and 2 for crack.
        mask = np.where(gt["Segmentation"] == 2, 255, 0).astype(np.uint8)
        if not _check_same_size(img_path, mask):
            continue
        sid = f"crackforest_{mat_path.stem}"
        img_rel, mask_rel = _save_pair(img_path, mask, out_root, "crackforest", sid)
        samples.append(Sample(sid, "crackforest", img_rel, mask_rel, None, sid))
    return samples


def convert_mask_suffix_folders(
    raw_dir: Path,
    out_root: Path,
    source: str,
    folder_to_split: dict[str, str | None],
    mask_suffix: str = "_mask",
) -> list[Sample]:
    """Generic converter for folders where image.jpg sits next to image_mask.png.

    Crack500 uses this layout, and it is also the easiest layout to export
    our own annotations into from CVAT or Label Studio.
    """
    samples = []
    for folder, split in folder_to_split.items():
        folder_path = raw_dir / folder
        if not folder_path.exists():
            log.warning("%s: expected folder %s is missing", source, folder_path)
            continue
        for img_path in sorted(folder_path.iterdir()):
            if img_path.suffix.lower() not in IMAGE_EXTS or img_path.stem.endswith(mask_suffix):
                continue
            mask_path = folder_path / f"{img_path.stem}{mask_suffix}.png"
            if not mask_path.exists():
                continue
            mask = _binary_mask(np.array(Image.open(mask_path).convert("L")))
            if not _check_same_size(img_path, mask):
                continue
            sid = f"{source}_{img_path.stem}"
            img_rel, mask_rel = _save_pair(img_path, mask, out_root, source, sid)
            samples.append(Sample(sid, source, img_rel, mask_rel, split, sid))
    return samples


def convert_crack500(raw_dir: Path, out_root: Path) -> list[Sample]:
    return convert_mask_suffix_folders(
        raw_dir, out_root, "crack500",
        {"traincrop": "train", "valcrop": "val", "testcrop": "test"},
    )


def convert_robot(raw_dir: Path, out_root: Path) -> list[Sample]:
    """Frames from our own rosbags, one folder per bag.

        data/raw/robot/<bag name>/<frame>.png          frame from tools/bag_to_images.py
        data/raw/robot/<bag name>/<frame>_mask.png     crack mask, white = crack
        data/raw/robot/splits.yaml                     optional, bag name -> train/val/test

    A frame without a mask file counts as a negative (no crack). That is on
    purpose: most frames we keep from the indoor bags are tile floor with
    grout lines, and drawing empty masks for all of them is busywork. The
    count of negatives per bag is logged, so a forgotten mask shows up as an
    unexpected number.

    The group is the bag name, so every frame of one drive lands in the same
    split. With only a handful of bags a random split is too coarse, which is
    why splits.yaml can fix the split per bag. Without it, splits.py
    assigns bags at random.
    """
    fixed_splits = {}
    split_file = raw_dir / "splits.yaml"
    if split_file.exists():
        import yaml
        fixed_splits = yaml.safe_load(split_file.read_text()) or {}

    samples = []
    for bag_dir in sorted(p for p in raw_dir.iterdir() if p.is_dir()):
        bag = bag_dir.name
        positives = negatives = 0
        for img_path in sorted(bag_dir.iterdir()):
            if img_path.suffix.lower() not in IMAGE_EXTS or img_path.stem.endswith("_mask"):
                continue
            mask_path = bag_dir / f"{img_path.stem}_mask.png"
            if mask_path.exists():
                mask = _binary_mask(np.array(Image.open(mask_path).convert("L")))
                positives += 1
            else:
                with Image.open(img_path) as im:
                    mask = np.zeros((im.height, im.width), np.uint8)
                negatives += 1
            if not _check_same_size(img_path, mask):
                continue
            sid = f"robot_{img_path.stem}"
            img_rel, mask_rel = _save_pair(img_path, mask, out_root, "robot", sid)
            samples.append(Sample(sid, "robot", img_rel, mask_rel, fixed_splits.get(bag), bag))
        log.info("robot/%s: %d frames with a mask, %d without (negatives), split %s",
                 bag, positives, negatives, fixed_splits.get(bag, "random"))
    return samples


CONVERTERS: dict[str, Callable[[Path, Path], list[Sample]]] = {
    "deepcrack": convert_deepcrack,
    "crackforest": convert_crackforest,
    "crack500": convert_crack500,
    "robot": convert_robot,
}
