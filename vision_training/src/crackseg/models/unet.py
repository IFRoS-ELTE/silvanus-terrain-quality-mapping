"""A small U-Net for binary crack segmentation.

This is the standard U-Net (Ronneberger et al., 2015) with two changes:
batch norm after every convolution, and a configurable width. With
base_channels=16 the model has about 1.9 M parameters, which is small
enough to run on the robot CPU through ONNX Runtime. base_channels=32
is about four times bigger and a better choice on the GPU server when
we only care about accuracy.

The input size has to be divisible by 2**depth (16 for depth 4). The
`pad_to_multiple` helper in inference.py takes care of that.
"""

from __future__ import annotations

import torch
from torch import nn
import torch.nn.functional as F


class DoubleConv(nn.Module):
    def __init__(self, in_ch: int, out_ch: int, dropout: float = 0.0):
        super().__init__()
        layers = [
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        ]
        if dropout > 0:
            layers.append(nn.Dropout2d(dropout))
        self.block = nn.Sequential(*layers)

    def forward(self, x):
        return self.block(x)


class UNet(nn.Module):
    def __init__(self, in_channels: int = 3, base_channels: int = 16, depth: int = 4,
                 dropout: float = 0.1):
        super().__init__()
        self.depth = depth
        widths = [base_channels * 2**i for i in range(depth + 1)]

        self.encoders = nn.ModuleList()
        prev = in_channels
        for w in widths[:-1]:
            self.encoders.append(DoubleConv(prev, w))
            prev = w

        # Dropout only in the bottleneck. The datasets are small (a few
        # hundred images), so some regularisation helps, but putting it in
        # the high resolution layers tends to break thin cracks apart.
        self.bottleneck = DoubleConv(widths[-2], widths[-1], dropout=dropout)

        self.upsamples = nn.ModuleList()
        self.decoders = nn.ModuleList()
        for w_in, w_out in zip(widths[::-1][:-1], widths[::-1][1:]):
            self.upsamples.append(nn.ConvTranspose2d(w_in, w_out, kernel_size=2, stride=2))
            self.decoders.append(DoubleConv(w_out * 2, w_out))

        self.head = nn.Conv2d(widths[0], 1, kernel_size=1)

    def forward(self, x):
        skips = []
        for enc in self.encoders:
            x = enc(x)
            skips.append(x)
            x = F.max_pool2d(x, 2)

        x = self.bottleneck(x)

        for up, dec, skip in zip(self.upsamples, self.decoders, reversed(skips)):
            x = up(x)
            x = dec(torch.cat([x, skip], dim=1))

        # Raw logits. Sigmoid is applied in the loss and at inference time.
        return self.head(x)


def build_model(cfg: dict) -> nn.Module:
    name = cfg.get("name", "unet")
    if name != "unet":
        raise ValueError(f"Unknown model '{name}'. Only 'unet' is implemented so far.")
    return UNet(
        in_channels=3,
        base_channels=cfg.get("base_channels", 16),
        depth=cfg.get("depth", 4),
        dropout=cfg.get("dropout", 0.1),
    )


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)
