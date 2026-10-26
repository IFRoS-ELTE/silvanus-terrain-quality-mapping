"""Remove two kinds of false cracks from the U-Net mask.

The U-Net was trained on public close-ups of cracks. On the robot it also marks
- tile grout and painted lines: long, straight and thin, which real cracks are not
- the edge of a kerb or door step: a dark line exactly where the depth jumps

Measured on the 2026-10-08 recordings (tools/analysis/hazard_map.py, docs/PROGRESS.md):
straight runs of 100 px and more remove 95-97 % of the tile grout and keep 97-100 % of
the crack pixels on cracked asphalt.
"""

import cv2
import numpy as np

from crack_detection.camera_calibration import transform_points


def straight_lines(mask, min_length=100, votes=60, max_gap=8, width=11):
    """Pixels of `mask` (uint8, 0/255) on straight runs at least `min_length` px long."""
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=votes, minLineLength=min_length, maxLineGap=max_gap)
    out = np.zeros_like(mask)
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)), 255, width)
    return out > 0


def step_edges_in_color(depth_m, depth_intrinsics, color_intrinsics, depth_to_color, color_shape,
                        *, valid_mask=None, zmin=0.30, zmax=1.60, threshold=0.03, stride=2, grow=7):
    """Colour-image pixels that lie on a height step in the depth image (kerb, door edge).

    The depth is median filtered (5x5) and its Sobel gradient taken; on flat or gently
    tilted ground the gradient stays a few mm, on a step of a few cm it is far above
    `threshold`. The step pixels are projected into the colour image like the crack
    points, and grown by `grow` px so a crack line along the edge is covered.
    """
    depth = np.where((depth_m > zmin) & (depth_m < zmax), depth_m, 0).astype(np.float32)
    smooth = cv2.medianBlur(depth, 5)
    grad = np.hypot(cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3))
    valid = smooth > 0
    valid = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)   # no edge at the holes
    if valid_mask is not None:
        valid &= valid_mask
    step = valid & (grad > threshold)
    ys, xs = np.nonzero(step[::stride, ::stride])
    ys, xs = ys * stride, xs * stride
    out = np.zeros(color_shape, np.uint8)
    if not len(xs):
        return out > 0
    fx, fy, cx, cy = depth_intrinsics
    z = smooth[ys, xs]
    points = np.column_stack(((xs - cx) * z / fx, (ys - cy) * z / fy, z))
    colour = transform_points(points, depth_to_color)
    colour = colour[colour[:, 2] > 0.05]
    fx, fy, cx, cy = color_intrinsics
    u = np.rint(fx * colour[:, 0] / colour[:, 2] + cx).astype(int)
    v = np.rint(fy * colour[:, 1] / colour[:, 2] + cy).astype(int)
    inside = (u >= 0) & (u < color_shape[1]) & (v >= 0) & (v < color_shape[0])
    out[v[inside], u[inside]] = 255
    if grow > 0:
        out = cv2.dilate(out, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * grow + 1, 2 * grow + 1)))
    return out > 0
