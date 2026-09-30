"""Training and evaluation loops."""

from __future__ import annotations

import logging
from contextlib import nullcontext

import numpy as np
import torch
from torch.utils.data import DataLoader
from tqdm import tqdm

from .inference import predict_logits
from .metrics import SegmentationMeter

log = logging.getLogger(__name__)


def train_one_epoch(
    model: torch.nn.Module,
    loader: DataLoader,
    criterion: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    scaler: torch.amp.GradScaler | None = None,
    max_batches: int | None = None,
    grad_clip: float | None = 1.0,
) -> float:
    model.train()
    use_amp = scaler is not None and device.type == "cuda"
    total, n = 0.0, 0

    bar = tqdm(loader, desc="train", leave=False, total=max_batches or len(loader))
    for i, (images, masks, _) in enumerate(bar):
        if max_batches is not None and i >= max_batches:
            break
        images = images.to(device, non_blocking=True)
        masks = masks.to(device, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)
        autocast = torch.autocast("cuda", dtype=torch.float16) if use_amp else nullcontext()
        with autocast:
            logits = model(images)
            loss = criterion(logits, masks)

        if use_amp:
            scaler.scale(loss).backward()
            if grad_clip:
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), grad_clip)
            scaler.step(optimizer)
            scaler.update()
        else:
            loss.backward()
            if grad_clip:
                torch.nn.utils.clip_grad_norm_(model.parameters(), grad_clip)
            optimizer.step()

        total += loss.item() * images.size(0)
        n += images.size(0)
        bar.set_postfix(loss=f"{total / n:.4f}")

    return total / max(n, 1)


@torch.no_grad()
def evaluate(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    criterion: torch.nn.Module | None = None,
    thresholds=(0.3, 0.4, 0.5, 0.6, 0.7),
    tolerance_px: int = 2,
    max_batches: int | None = None,
    keep_predictions: bool = False,
):
    """Run the model over full size images and collect metrics.

    Returns (meter, mean loss, predictions). `predictions` maps sample id to
    the probability map and is only filled when keep_predictions is True,
    because holding hundreds of full images in memory is wasteful during
    training.
    """
    model.eval()
    meter = SegmentationMeter(thresholds, tolerance_px)
    multiple = 2 ** getattr(model, "depth", 4)
    losses = []
    predictions: dict[str, np.ndarray] = {}

    for i, (images, masks, ids) in enumerate(tqdm(loader, desc="eval", leave=False)):
        if max_batches is not None and i >= max_batches:
            break
        images, masks = images.to(device), masks.to(device)
        logits = predict_logits(model, images, multiple)
        if criterion is not None:
            losses.append(criterion(logits, masks).item())

        probs = torch.sigmoid(logits).cpu().numpy()
        targets = masks.cpu().numpy()
        for p, t, sid in zip(probs, targets, ids):
            meter.update(p[0], t[0])
            if keep_predictions:
                predictions[sid] = p[0]

    mean_loss = float(np.mean(losses)) if losses else float("nan")
    return meter, mean_loss, predictions
