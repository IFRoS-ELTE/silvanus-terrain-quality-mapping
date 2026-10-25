"""Associate RGB crack predictions with measured depth surface points."""

import cv2
import numpy as np

from crack_detection.camera_calibration import transform_points


def crack_surface_points(mask, depth_m, depth_intrinsics, color_intrinsics, depth_to_color,
                         *, valid_mask=None, zmin=0.30, zmax=1.60, stride=2,
                         max_points=5000, distortion=None, distortion_model="plumb_bob"):
    """Return XYZ in the depth optical frame, never synthetic crack depths.

    Depth and RGB may have different resolutions. Project each sampled depth
    point into RGB and keep it only if that RGB pixel is predicted as a crack.
    The caller handles timestamp pairing and transforms into a mapping frame.
    """
    if stride < 1 or max_points < 1 or not 0 < zmin < zmax:
        raise ValueError("Invalid depth range, stride, or point limit")
    if mask.ndim != 2 or depth_m.ndim != 2:
        raise ValueError("Crack mask and depth must be two-dimensional")
    if valid_mask is not None and valid_mask.shape != depth_m.shape:
        raise ValueError("Robot mask must match depth dimensions")
    fx, fy, cx, cy = depth_intrinsics
    if fx <= 0 or fy <= 0:
        raise ValueError("Invalid depth focal lengths")
    ys, xs = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = depth_m[ys, xs]
    valid = np.isfinite(z) & (z > zmin) & (z < zmax)
    if valid_mask is not None:
        valid &= valid_mask[ys, xs]
    z, xs, ys = z[valid], xs[valid], ys[valid]
    points = np.column_stack(((xs - cx) * z / fx, (ys - cy) * z / fy, z))
    if not len(points):
        return np.empty((0, 3), np.float32)
    colour = transform_points(points, depth_to_color)
    visible = np.isfinite(colour).all(axis=1) & (colour[:, 2] > 0.05)
    points, colour = points[visible], colour[visible]
    if not len(points):
        return np.empty((0, 3), np.float32)
    fx, fy, cx, cy = color_intrinsics
    if fx <= 0 or fy <= 0:
        raise ValueError("Invalid RGB focal lengths")
    if distortion is not None and np.any(distortion):
        if distortion_model not in ("plumb_bob", "rational_polynomial"):
            raise ValueError(f"Unsupported RGB distortion model: {distortion_model}")
        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        pixels, _ = cv2.projectPoints(colour, np.zeros(3), np.zeros(3), matrix,
                                      np.asarray(distortion, dtype=np.float64))
        u, v = pixels.reshape(-1, 2).T
    else:
        u = fx * colour[:, 0] / colour[:, 2] + cx
        v = fy * colour[:, 1] / colour[:, 2] + cy
    u, v = np.rint(u), np.rint(v)
    inside = np.isfinite(u) & np.isfinite(v) & (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
    points = points[inside]
    hits = mask[v[inside].astype(np.int32), u[inside].astype(np.int32)] > 0
    points = points[hits]
    if len(points) > max_points:
        points = points[np.linspace(0, len(points) - 1, max_points, dtype=np.int32)]
    return points.astype(np.float32)
