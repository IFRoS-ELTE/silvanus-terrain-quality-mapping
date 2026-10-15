#!/usr/bin/env python3
"""
bag_to_images: pull frames out of a rosbag so they can be annotated.

A 33 second bag holds about 1000 colour frames, but consecutive frames are
nearly identical, so annotating all of them wastes effort and inflates any
score computed on them. Sampling at 1 to 2 frames per second gives a set of
genuinely different images.

Usage
    python3 bag_to_images.py BAG_DIR --out frames/ --fps 1
    python3 bag_to_images.py BAG_DIR --out frames/ --fps 2 --depth

Outputs
    frames/<bagname>_<index>_<seconds>.png     colour, ready to annotate
    frames/depth/...                           16-bit depth, if --depth
    frames/index.csv                           filename, bag time, source topic

Keep index.csv. When the dataset is split later, the split must be by
recording session, never by frame: two frames half a second apart show the
same crack from almost the same angle, so splitting by frame puts the same
object in train and test and the score comes out far too high.
"""

import argparse
import csv
import os

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

COLOUR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"


def image_to_array(msg):
    """sensor_msgs/Image -> numpy array, for the encodings the D435 produces."""
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == "rgb8" else a.copy()
    if enc in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
    if enc == "mono8":
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width).copy()
    raise ValueError(f"unhandled encoding {enc!r}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag directory (the folder, not the .mcap file)")
    ap.add_argument("--out", default="frames", help="output directory")
    ap.add_argument("--fps", type=float, default=1.0,
                    help="frames to keep per second (default 1)")
    ap.add_argument("--depth", action="store_true",
                    help="also save the matching 16-bit depth frames")
    ap.add_argument("--jpeg", action="store_true",
                    help="save colour as JPEG instead of PNG (smaller, lossy; "
                         "do not use for anything a model is trained on)")
    args = ap.parse_args()

    bag_name = os.path.basename(os.path.normpath(args.bag))
    os.makedirs(args.out, exist_ok=True)
    if args.depth:
        os.makedirs(os.path.join(args.out, "depth"), exist_ok=True)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="mcap"),
                ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if COLOUR_TOPIC not in types:
        raise SystemExit(f"{COLOUR_TOPIC} is not in this bag. Topics present:\n  "
                         + "\n  ".join(sorted(types)))

    interval = 1.0 / args.fps
    rows = []
    t_first = None
    next_keep = {COLOUR_TOPIC: -1e9, DEPTH_TOPIC: -1e9}
    kept = {COLOUR_TOPIC: 0, DEPTH_TOPIC: 0}
    seen = {COLOUR_TOPIC: 0, DEPTH_TOPIC: 0}
    ext = "jpg" if args.jpeg else "png"

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in (COLOUR_TOPIC, DEPTH_TOPIC):
            continue
        if topic == DEPTH_TOPIC and not args.depth:
            continue

        t = stamp * 1e-9
        t_first = t if t_first is None else t_first
        rel = t - t_first
        seen[topic] += 1
        if rel < next_keep[topic]:
            continue
        next_keep[topic] = rel + interval

        msg = deserialize_message(data, get_message(types[topic]))
        img = image_to_array(msg)
        idx = kept[topic]
        name = f"{bag_name}_{idx:04d}_{rel:06.2f}s.{'png' if topic == DEPTH_TOPIC else ext}"
        path = (os.path.join(args.out, "depth", name) if topic == DEPTH_TOPIC
                else os.path.join(args.out, name))
        cv2.imwrite(path, img)
        kept[topic] += 1
        if topic == COLOUR_TOPIC:
            rows.append((name, f"{rel:.3f}", f"{t:.6f}", bag_name, topic))

    with open(os.path.join(args.out, "index.csv"), "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(["file", "t_rel_s", "t_bag_s", "bag", "topic"])
        w.writerows(rows)

    print(f"bag            : {bag_name}")
    print(f"colour frames  : {seen[COLOUR_TOPIC]} in bag -> {kept[COLOUR_TOPIC]} saved "
          f"at {args.fps} fps")
    if args.depth:
        print(f"depth frames   : {seen[DEPTH_TOPIC]} in bag -> {kept[DEPTH_TOPIC]} saved")
    print(f"written to     : {args.out}/  (index.csv appended)")


if __name__ == "__main__":
    main()
