"""Frame index and the PyTorch dataset of ground patches."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision.transforms import v2 as T

from .patches import cut_patches

# The pretrained backbones were trained with these statistics, and the
# deployed node has to use the same numbers (export writes them out).
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]


@dataclass
class Frame:
    file: str       # relative to the frames root
    bag: str
    t: float
    label: str


def read_frames(index_csv: Path, bags: list[str] | None = None) -> list[Frame]:
    with open(index_csv, newline="") as f:
        rows = [Frame(r["file"], r["bag"], float(r["t"]), r["label"]) for r in csv.DictReader(f)]
    return [r for r in rows if bags is None or r.bag in bags]


def write_frames(frames: list[Frame], index_csv: Path) -> None:
    index_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(index_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["file", "bag", "t", "label"])
        w.writeheader()
        for fr in frames:
            w.writerow({"file": fr.file, "bag": fr.bag, "t": f"{fr.t:.3f}", "label": fr.label})


def train_transform(input_size: int):
    # Ground texture has no fixed orientation, so flips and turns are free.
    # Colour jitter is kept mild on hue: colour is a real cue here (the red
    # path is red), we only want robustness to sun, shade and exposure.
    return T.Compose([
        T.ToImage(),
        T.RandomResizedCrop(input_size, scale=(0.5, 1.0), antialias=True),
        T.RandomHorizontalFlip(),
        T.RandomVerticalFlip(),
        T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.02),
        T.ToDtype(torch.float32, scale=True),
        T.Normalize(MEAN, STD),
    ])


def eval_transform(input_size: int):
    return T.Compose([
        T.ToImage(),
        T.Resize(input_size, antialias=True),
        T.ToDtype(torch.float32, scale=True),
        T.Normalize(MEAN, STD),
    ])


class PatchDataset(Dataset):
    """Every (frame, patch) pair is one sample. Returns (tensor, class index, frame index)."""

    def __init__(self, root: Path, frames: list[Frame], classes: list[str], boxes, transform):
        if not frames:
            raise ValueError("No frames for this split. Did terrain_extract.py run for these bags?")
        self.root = Path(root)
        self.frames = frames
        self.class_index = {c: i for i, c in enumerate(classes)}
        self.boxes = boxes
        self.transform = transform

    def __len__(self):
        return len(self.frames) * len(self.boxes)

    def __getitem__(self, i):
        frame_idx, patch_idx = divmod(i, len(self.boxes))
        fr = self.frames[frame_idx]
        image = np.array(Image.open(self.root / fr.file).convert("RGB"))
        patch = cut_patches(image, [self.boxes[patch_idx]])[0]
        return self.transform(patch), self.class_index[fr.label], frame_idx

    def sample_weights(self) -> torch.Tensor:
        """Weight per sample so every class is drawn about equally often.
        Wood chips has five times fewer frames than grass; without this the
        model would learn to rarely say wood chips."""
        labels = np.array([self.class_index[f.label] for f in self.frames]).repeat(len(self.boxes))
        counts = np.bincount(labels, minlength=len(self.class_index)).astype(float)
        return torch.as_tensor(1.0 / counts[labels], dtype=torch.double)
