"""Small pretrained CNN with a new classification head."""

from __future__ import annotations

from torch import nn
from torchvision import models

# MobileNetV3-small: 1.5M parameters, about 8 ms per frame (6 patches) with
# ONNX Runtime on a laptop CPU.
# ResNet18 is there for comparison if the small one turns out too weak.
BACKBONES = {
    "mobilenet_v3_small": (models.mobilenet_v3_small, models.MobileNet_V3_Small_Weights.IMAGENET1K_V1),
    "resnet18": (models.resnet18, models.ResNet18_Weights.IMAGENET1K_V1),
}


def build_model(name: str, num_classes: int, pretrained: bool = True) -> nn.Module:
    if name not in BACKBONES:
        raise ValueError(f"Unknown backbone '{name}', choose from {sorted(BACKBONES)}")
    constructor, weights = BACKBONES[name]
    model = constructor(weights=weights if pretrained else None)

    # Swap the 1000-class ImageNet layer for one with our classes.
    if name.startswith("mobilenet"):
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, num_classes)
    else:
        model.fc = nn.Linear(model.fc.in_features, num_classes)
    return model
