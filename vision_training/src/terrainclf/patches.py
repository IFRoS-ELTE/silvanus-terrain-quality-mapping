"""Where on the frame we cut the ground patches.

The camera is tilted down, so nearly the whole frame is ground. The robot
body and gripper sit at the bottom and are blacked out by the colour robot
mask (white = ground). We lay a grid of square patches over the frame and
keep the ones that are almost entirely ground.

One frame gives several patches. Training sees each patch as a sample; at
test time the patch probabilities of a frame are averaged into one answer.
Averaging over the frame is what makes a leaf or a small stone on asphalt
not flip the label.
"""

from __future__ import annotations

import numpy as np


def patch_boxes(ground_mask: np.ndarray, size: int, rows: int, cols: int,
                min_ground: float = 0.95) -> list[tuple[int, int, int, int]]:
    """Return (x, y, w, h) boxes of a rows x cols grid of size x size patches.

    The grid is centred horizontally and sits on the lowest ground row, so
    the patches cover the ground nearest the robot, where the image is
    sharpest and the surface is the one the wheels are about to reach.
    Patches with less than `min_ground` of their pixels on ground are dropped.
    """
    h, w = ground_mask.shape
    ground_rows = np.where((ground_mask > 0).mean(axis=1) > 0.5)[0]
    if len(ground_rows) == 0:
        raise ValueError("The robot mask leaves no ground in the frame")
    bottom = int(ground_rows.max()) + 1
    top = bottom - rows * size
    left = (w - cols * size) // 2
    if top < 0 or left < 0:
        raise ValueError(f"A {rows}x{cols} grid of {size} px patches does not fit in "
                         f"{w}x{h} above the robot mask")

    boxes = []
    for r in range(rows):
        for c in range(cols):
            x, y = left + c * size, top + r * size
            if (ground_mask[y:y + size, x:x + size] > 0).mean() >= min_ground:
                boxes.append((x, y, size, size))
    return boxes


def cut_patches(image: np.ndarray, boxes) -> list[np.ndarray]:
    return [image[y:y + h, x:x + w] for x, y, w, h in boxes]
