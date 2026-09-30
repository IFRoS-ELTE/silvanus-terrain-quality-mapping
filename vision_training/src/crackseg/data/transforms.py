"""Augmentations written in plain numpy.

We keep these simple on purpose instead of pulling in albumentations.
Everything works on (H, W, 3) uint8 images and (H, W) uint8 masks, and
geometric changes are always applied to both.

Why these augmentations:
- Random crops instead of resizing: cracks are one to five pixels wide,
  and shrinking a whole image makes many of them disappear.
- Flips and 90 degree turns: a crack has no preferred direction.
- Brightness, contrast and a bit of blur: the robot drives in and out of
  shade, and the colour sensor is rolling shutter, so frames can smear.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter


def pad_to_min_size(image: np.ndarray, mask: np.ndarray, size: int):
    h, w = mask.shape
    pad_h, pad_w = max(0, size - h), max(0, size - w)
    if pad_h == 0 and pad_w == 0:
        return image, mask
    image = np.pad(image, ((0, pad_h), (0, pad_w), (0, 0)), mode="reflect")
    # Padding the mask with zeros is fine, reflected cracks would be fake labels.
    mask = np.pad(mask, ((0, pad_h), (0, pad_w)), mode="constant")
    return image, mask


def random_crop(image, mask, size: int, rng: np.random.Generator, crack_bias: float = 0.5):
    """Crop a size x size window.

    With probability crack_bias the window is centred near a random crack
    pixel. Cracks cover only 2 to 5 percent of the pixels, so purely random
    crops would show the network mostly empty road.
    """
    image, mask = pad_to_min_size(image, mask, size)
    h, w = mask.shape

    crack_pixels = np.argwhere(mask > 0)
    if len(crack_pixels) and rng.random() < crack_bias:
        cy, cx = crack_pixels[rng.integers(len(crack_pixels))]
        top = int(np.clip(cy - size // 2 + rng.integers(-size // 4, size // 4 + 1), 0, h - size))
        left = int(np.clip(cx - size // 2 + rng.integers(-size // 4, size // 4 + 1), 0, w - size))
    else:
        top = int(rng.integers(0, h - size + 1))
        left = int(rng.integers(0, w - size + 1))

    return (image[top:top + size, left:left + size],
            mask[top:top + size, left:left + size])


def random_flip_rotate(image, mask, rng: np.random.Generator):
    if rng.random() < 0.5:
        image, mask = image[:, ::-1], mask[:, ::-1]
    if rng.random() < 0.5:
        image, mask = image[::-1], mask[::-1]
    k = int(rng.integers(4))
    image, mask = np.rot90(image, k), np.rot90(mask, k)
    return np.ascontiguousarray(image), np.ascontiguousarray(mask)


def random_photometric(image, rng: np.random.Generator, strength: float = 0.25):
    img = image.astype(np.float32)
    contrast = 1.0 + rng.uniform(-strength, strength)
    brightness = rng.uniform(-strength, strength) * 255 * 0.5
    img = (img - img.mean()) * contrast + img.mean() + brightness

    if rng.random() < 0.2:
        sigma = rng.uniform(0.5, 1.2)
        img = gaussian_filter(img, sigma=(sigma, sigma, 0))

    return np.clip(img, 0, 255).astype(np.uint8)


class TrainTransform:
    def __init__(self, crop_size: int, crack_bias: float = 0.5, photometric: float = 0.25):
        self.crop_size = crop_size
        self.crack_bias = crack_bias
        self.photometric = photometric

    def __call__(self, image, mask, rng: np.random.Generator):
        image, mask = random_crop(image, mask, self.crop_size, rng, self.crack_bias)
        image, mask = random_flip_rotate(image, mask, rng)
        if self.photometric > 0:
            image = random_photometric(image, rng, self.photometric)
        return image, mask
