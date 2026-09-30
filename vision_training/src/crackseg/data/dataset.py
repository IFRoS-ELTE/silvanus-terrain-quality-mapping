"""PyTorch dataset over the rows of index.csv."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

from .splits import read_index
from .transforms import TrainTransform

# ImageNet statistics. Not needed for a network trained from scratch, but
# it keeps inputs centred around zero and lets us swap in a pretrained
# encoder later without touching the data code. The ROS node has to use
# the same numbers, so export_onnx writes them next to the model.
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def to_tensor(image: np.ndarray) -> torch.Tensor:
    """(H, W, 3) uint8 -> normalised (3, H, W) float tensor."""
    x = (image.astype(np.float32) / 255.0 - MEAN) / STD
    return torch.from_numpy(x.transpose(2, 0, 1).copy())


def load_pair(root: Path, image_rel: str, mask_rel: str) -> tuple[np.ndarray, np.ndarray]:
    image = np.array(Image.open(root / image_rel).convert("RGB"))
    mask = np.array(Image.open(root / mask_rel).convert("L"))
    return image, mask


class CrackDataset(Dataset):
    """Returns (image tensor, mask tensor, sample id).

    For training, pass a TrainTransform; images are then cropped to a
    fixed size and can be batched. For validation and test we return the
    full image, so those loaders run with batch_size=1.
    """

    def __init__(
        self,
        processed_root: Path,
        split: str,
        sources: list[str] | None = None,
        transform: TrainTransform | None = None,
    ):
        self.root = Path(processed_root)
        self.samples = read_index(self.root / "index.csv", split=split, sources=sources)
        if not self.samples:
            raise ValueError(
                f"No samples for split '{split}' (sources={sources}) in {self.root}/index.csv. "
                "Did you run scripts/prepare_data.py?"
            )
        self.transform = transform

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        s = self.samples[idx]
        image, mask = load_pair(self.root, s.image, s.mask)

        if self.transform is not None:
            # Draw the numpy seed from torch so each DataLoader worker gets
            # different crops, and torch.manual_seed still makes runs repeatable.
            rng = np.random.default_rng(int(torch.randint(0, 2**31 - 1, (1,))))
            image, mask = self.transform(image, mask, rng)

        mask_t = torch.from_numpy((mask > 127).astype(np.float32))[None]
        return to_tensor(image), mask_t, s.sample_id
