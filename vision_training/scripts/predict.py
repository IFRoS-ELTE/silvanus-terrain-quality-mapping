"""Run a checkpoint on a folder of images that have no labels.

    python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images frames/garden_1618_cracked
    python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images frames/corridor_tile \
        --mask path/to/colour_robot_mask.png

Meant for our own frames (from rosbags or the spare D435) before they are
annotated. It shows where the public-data model works and where it fails
on our ground, which tells us which frames are worth labelling first.

Outputs, in <images>_pred/ unless --out is given:
    <frame>_overlay.png   the frame with crack pixels in red, masked area darkened
    <frame>_prob.png      crack probability as an 8-bit image
    summary.csv           one row per frame: crack pixels, share, and whether
                          the frame counts as "crack found"

A frame counts as "crack found" when at least --min-pixels crack pixels
survive the threshold. Run it once on frames that show cracked asphalt
(the share of frames with a crack found is the detection rate) and once on
plain indoor tile (every frame with a crack found is a false alarm).
"""

import argparse
import csv
import logging
from pathlib import Path

import numpy as np
from PIL import Image

from crackseg.data.converters import IMAGE_EXTS
from crackseg.inference import load_checkpoint, predict_image
from crackseg.utils import pick_device, setup_logging

log = logging.getLogger("predict")


def load_mask(path: str, size: tuple[int, int]) -> np.ndarray:
    """White = ground (keep), black = robot body (ignore). size is (width, height)."""
    mask = Image.open(path).convert("L")
    if mask.size != size:
        mask = mask.resize(size, Image.NEAREST)
    return np.array(mask) > 127


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--images", required=True, help="folder with images")
    parser.add_argument("--out", default=None, help="default: <images>_pred next to the folder")
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--mask", default=None,
                        help="PNG drawn on the colour stream, white = ground, black = robot")
    parser.add_argument("--min-pixels", type=int, default=150,
                        help="crack pixels needed before a frame counts as 'crack found'")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    setup_logging()

    device = pick_device(args.device)
    model, ckpt = load_checkpoint(args.checkpoint, device)
    threshold = args.threshold if args.threshold is not None else ckpt["threshold"]

    in_dir = Path(args.images)
    out_dir = Path(args.out) if args.out else in_dir.with_name(in_dir.name + "_pred")
    out_dir.mkdir(parents=True, exist_ok=True)

    paths = sorted(p for p in in_dir.iterdir()
                   if p.suffix.lower() in IMAGE_EXTS and not p.stem.endswith("_mask"))
    rows = []
    ground = None
    for path in paths:
        image = np.array(Image.open(path).convert("RGB"))
        prob = predict_image(model, image, device)

        if args.mask:
            if ground is None or ground.shape != prob.shape:
                ground = load_mask(args.mask, (image.shape[1], image.shape[0]))
            prob = np.where(ground, prob, 0.0)

        crack = prob >= threshold
        n = int(crack.sum())
        visible = int(ground.sum()) if ground is not None else crack.size
        rows.append({"frame": path.name, "crack_pixels": n,
                     "crack_percent": round(100 * n / visible, 3),
                     "crack_found": int(n >= args.min_pixels)})

        vis = image.copy()
        if ground is not None:
            vis[~ground] = (vis[~ground] * 0.35).astype(np.uint8)
        vis[crack] = (255, 0, 0)
        Image.fromarray(vis).save(out_dir / f"{path.stem}_overlay.png")
        Image.fromarray((prob * 255).astype(np.uint8)).save(out_dir / f"{path.stem}_prob.png")

    with open(out_dir / "summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["frame", "crack_pixels", "crack_percent", "crack_found"])
        writer.writeheader()
        writer.writerows(rows)

    found = sum(r["crack_found"] for r in rows)
    log.info("%d frames, crack found in %d (%.0f%%), threshold %.2f, min %d pixels",
             len(rows), found, 100 * found / max(len(rows), 1), threshold, args.min_pixels)
    log.info("Saved to %s", out_dir)


if __name__ == "__main__":
    main()
