"""Read the recorded static transform between the two camera optical frames."""

from collections import deque

import numpy as np


def transform_matrix(translation, quaternion):
    """Build the child-to-parent transform represented by a ROS Transform."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm == 0:
        raise ValueError("Invalid camera calibration quaternion")
    x, y, z, w = np.array([x, y, z, w]) / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    result[:3, 3] = translation
    return result


def find_transform(edges, source, target):
    """Resolve a static TF chain into a matrix mapping source points to target."""
    graph = {}
    for (parent, child), matrix in edges.items():
        graph.setdefault(child, []).append((parent, matrix))
        graph.setdefault(parent, []).append((child, np.linalg.inv(matrix)))
    queue = deque([(source, np.eye(4))])
    seen = {source}
    while queue:
        frame, accumulated = queue.popleft()
        if frame == target:
            return accumulated
        for neighbour, step in graph.get(frame, []):
            if neighbour not in seen:
                seen.add(neighbour)
                queue.append((neighbour, step @ accumulated))
    return None


def read_depth_to_color(bag, depth_frame="camera_depth_optical_frame",
                        color_frame="camera_color_optical_frame"):
    """Return the recorded optical-frame transform, or None if unavailable."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from tf2_msgs.msg import TFMessage

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=["/tf_static"]))
    edges = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        for item in deserialize_message(data, TFMessage).transforms:
            t, q = item.transform.translation, item.transform.rotation
            key = (item.header.frame_id.lstrip("/"), item.child_frame_id.lstrip("/"))
            edges[key] = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        result = find_transform(edges, depth_frame, color_frame)
        if result is not None:
            return result
    return None


def transform_points(points, matrix):
    """Transform optical-frame XYZ points without modifying the input cloud."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def project_depth_points(points, intrinsics, depth_to_color):
    """Project depth XYZ into RGB pixels; invalid projections remain NaN."""
    colour = transform_points(points, depth_to_color)
    valid = np.isfinite(colour).all(axis=1) & (colour[:, 2] > 0.05)
    u, v = np.full(len(points), np.nan), np.full(len(points), np.nan)
    fx, fy, cx, cy = intrinsics
    u[valid] = fx * colour[valid, 0] / colour[valid, 2] + cx
    v[valid] = fy * colour[valid, 1] / colour[valid, 2] + cy
    return u, v, valid
