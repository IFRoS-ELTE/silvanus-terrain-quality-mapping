"""Build the colour-image robot mask (gripper and body) from recorded frames.

    python scripts/make_robot_mask.py --frames ../frames/garden_1618 ../frames/corridor_1338 \
        ../frames/garden_1628 --out models/robot_mask_color.png

The arm pose is fixed, so the gripper sits in the same place in every
frame while the ground under it keeps changing. A pixel whose colour
hardly changes over many frames from different bags (tile, asphalt,
grass) is therefore part of the robot. We take the per-pixel standard
deviation over all frames, call the low-variance pixels robot, keep only
the regions that touch the bottom edge (the gripper comes in from below,
so a flat patch of ground in the middle can never be mistaken for it),
and then grow the robot area by a safety margin, because the arm shakes
a little while driving.

Use frames from at least three different bags with different ground. If
all frames show the same plain asphalt, the ground also looks static and
the mask will be too big. Check the preview it writes.

If the automatic mask looks wrong, --bottom-rows N simply blocks the
bottom N rows. That loses a strip of ground but is always safe, and the
nearest ground is not where cracks are judged anyway.

Output: a PNG of the frame size, white = ground (use), black = robot
(ignore), plus <out>_preview.png showing the mask on a few frames.
"""

import argparse
import logging
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

from crackseg.data.converters import IMAGE_EXTS
from crackseg.utils import setup_logging

log = logging.getLogger("robot_mask")


def static_pixels(frames: np.ndarray, rel_threshold: float) -> np.ndarray:
    """frames: (N, H, W) grayscale float. True where the pixel barely changes."""
    std = frames.std(axis=0)
    return std < rel_threshold * np.median(std)


def robot_mask_from_frames(frames: np.ndarray, rel_threshold: float = 0.35,
                           margin_px: int = 8, min_area: int = 200) -> np.ndarray:
    """Returns a bool (H, W) array, True = robot."""
    static = static_pixels(frames, rel_threshold)
    static = ndimage.binary_opening(static, iterations=2)  # drop single noisy pixels

    labels, n = ndimage.label(static)
    touches_bottom = set(np.unique(labels[-1])) - {0}
    sizes = ndimage.sum(static, labels, index=range(1, n + 1))
    keep = [i for i in touches_bottom if sizes[i - 1] >= min_area]
    robot = np.isin(labels, keep)

    robot = ndimage.binary_fill_holes(robot)
    if margin_px > 0:
        robot = ndimage.binary_dilation(robot, iterations=margin_px)
    return robot


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", nargs="+", required=True, help="one or more folders of frames")
    parser.add_argument("--out", required=True, help="mask PNG to write")
    parser.add_argument("--max-frames", type=int, default=300, help="spread evenly over all folders")
    parser.add_argument("--rel-threshold", type=float, default=0.35,
                        help="robot if pixel std < this x median std (lower = smaller mask)")
    parser.add_argument("--margin", type=int, default=8, help="extra pixels around the robot")
    parser.add_argument("--bottom-rows", type=int, default=None,
                        help="skip the automatic mask, block the bottom N rows instead")
    args = parser.parse_args()
    setup_logging()

    paths = sorted(p for d in args.frames for p in Path(d).iterdir()
                   if p.suffix.lower() in IMAGE_EXTS and not p.stem.endswith("_mask"))
    if not paths:
        raise SystemExit("No frames found in " + ", ".join(args.frames))
    step = max(1, len(paths) // args.max_frames)
    paths = paths[::step]

    with Image.open(paths[0]) as im:
        width, height = im.size

    if args.bottom_rows is not None:
        robot = np.zeros((height, width), bool)
        robot[height - args.bottom_rows:] = True
        log.info("Blocking the bottom %d rows", args.bottom_rows)
    else:
        frames = np.stack([np.asarray(Image.open(p).convert("L"), np.float32) for p in paths])
        log.info("Using %d frames of %dx%d from %d folders", len(frames), width, height, len(args.frames))
        robot = robot_mask_from_frames(frames, args.rel_threshold, args.margin)

    log.info("Robot covers %.1f%% of the frame", 100 * robot.mean())
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.where(robot, 0, 255).astype(np.uint8)).save(out)

    # Preview: four frames spread over the set, robot area tinted red.
    picks = [paths[i] for i in np.linspace(0, len(paths) - 1, 4).astype(int)]
    tiles = []
    for p in picks:
        rgb = np.array(Image.open(p).convert("RGB"))
        rgb[robot] = (0.5 * rgb[robot] + [127, 0, 0]).astype(np.uint8)
        tiles.append(rgb)
    preview = out.with_name(out.stem + "_preview.png")
    Image.fromarray(np.concatenate(tiles, axis=1)).save(preview)
    log.info("Wrote %s and %s. Look at the preview before using the mask.", out, preview)


if __name__ == "__main__":
    main()
