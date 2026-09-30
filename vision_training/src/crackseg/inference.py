"""Running a trained model on full size images."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from .data.dataset import to_tensor
from .models import build_model


def pad_to_multiple(x: torch.Tensor, multiple: int) -> tuple[torch.Tensor, tuple[int, int]]:
    """Pad the bottom and right edge so H and W are divisible by `multiple`.

    Returns the padded tensor and the original (H, W) so the output can be
    cropped back. Reflect padding avoids a hard black border that the
    network could mistake for an edge.
    """
    h, w = x.shape[-2:]
    pad_h = (multiple - h % multiple) % multiple
    pad_w = (multiple - w % multiple) % multiple
    if pad_h or pad_w:
        x = F.pad(x, (0, pad_w, 0, pad_h), mode="reflect")
    return x, (h, w)


@torch.no_grad()
def predict_logits(model: torch.nn.Module, x: torch.Tensor, multiple: int = 16) -> torch.Tensor:
    x_pad, (h, w) = pad_to_multiple(x, multiple)
    return model(x_pad)[..., :h, :w]


def load_checkpoint(path: str | Path, device: torch.device):
    """Returns (model in eval mode, checkpoint dict)."""
    ckpt = torch.load(path, map_location=device, weights_only=False)
    model = build_model(ckpt["config"]["model"]).to(device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, ckpt


@torch.no_grad()
def predict_image(model, image: np.ndarray, device: torch.device) -> np.ndarray:
    """(H, W, 3) uint8 RGB image -> (H, W) crack probability map."""
    x = to_tensor(image)[None].to(device)
    multiple = 2 ** getattr(model, "depth", 4)
    return torch.sigmoid(predict_logits(model, x, multiple))[0, 0].cpu().numpy()
