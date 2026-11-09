"""Read colour frames from a rosbag without a ROS installation.

tools/bag_to_images.py needs ROS 2 (rosbag2_py). The GPU server has none,
so this uses the pure-Python `rosbags` package, which reads the same .mcap
bags. Times are counted from the first colour frame, exactly like
bag_to_images.py, so frame times match the labels in docs/DATA.md.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

import numpy as np

COLOUR_TOPIC = "/camera/camera/color/image_raw"


def image_to_rgb(msg) -> np.ndarray:
    """sensor_msgs/Image (as deserialised by rosbags) -> (H, W, 3) uint8 RGB."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unhandled colour encoding {msg.encoding!r}")
    data = np.frombuffer(bytes(msg.data), np.uint8)
    # Rows can be padded, so step and width do not have to agree.
    img = data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return img[..., ::-1].copy() if msg.encoding == "bgr8" else img.copy()


def colour_frames(bag_dir: str | Path, fps: float,
                  topic: str = COLOUR_TOPIC) -> Iterator[tuple[float, np.ndarray]]:
    """Yield (seconds since the first colour frame, RGB image) at about `fps`.

    Consecutive frames at 30 Hz are nearly identical, so keeping all of them
    adds no information and makes a test score look more certain than it is.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    interval = 1.0 / fps
    with AnyReader([Path(bag_dir)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            topics = sorted({c.topic for c in reader.connections})
            raise ValueError(f"{topic} is not in {bag_dir}. Topics present: {topics}")

        t_first = None
        next_keep = 0.0
        for conn, stamp_ns, raw in reader.messages(connections=connections):
            t = stamp_ns * 1e-9
            t_first = t if t_first is None else t_first
            rel = t - t_first
            if rel < next_keep:
                continue
            next_keep = rel + interval
            yield rel, image_to_rgb(reader.deserialize(raw, conn.msgtype))
