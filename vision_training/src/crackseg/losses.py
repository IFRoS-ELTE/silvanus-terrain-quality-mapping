"""Loss for thin-structure segmentation.

Crack pixels are rare (usually 2 to 5 percent of an image). Plain binary
cross entropy is then happy to predict "no crack" everywhere and still
get a low loss. We combine two terms:

- BCE with pos_weight: pushes up the weight of crack pixels, gives stable
  per-pixel gradients early in training.
- Soft Dice: measures overlap of the whole crack region, so it does not
  care how much background there is. It directly optimises what we
  report at the end.

This combination is the usual starting point in medical segmentation too
(small lesions on large backgrounds is the same problem).
"""

from __future__ import annotations

import torch
from torch import nn
import torch.nn.functional as F


def soft_dice_loss(logits: torch.Tensor, target: torch.Tensor, eps: float = 1.0) -> torch.Tensor:
    probs = torch.sigmoid(logits)
    dims = (1, 2, 3)
    intersection = (probs * target).sum(dims)
    union = probs.sum(dims) + target.sum(dims)
    # eps keeps the loss defined on crops without any crack pixels
    dice = (2 * intersection + eps) / (union + eps)
    return 1 - dice.mean()


class BceDiceLoss(nn.Module):
    def __init__(self, bce_weight: float = 0.5, dice_weight: float = 0.5, pos_weight: float = 5.0):
        super().__init__()
        self.bce_weight = bce_weight
        self.dice_weight = dice_weight
        self.register_buffer("pos_weight", torch.tensor([pos_weight]))

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        bce = F.binary_cross_entropy_with_logits(logits, target, pos_weight=self.pos_weight)
        dice = soft_dice_loss(logits, target)
        return self.bce_weight * bce + self.dice_weight * dice
