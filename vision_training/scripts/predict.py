"""Run a checkpoint on a folder of images that have no labels.

    python scripts/predict.py --checkpoint runs/<run>/best.pt --images path/to/frames

Meant for our own frames (spare D435 at home, frames pulled from rosbags)
before they are annotated. It shows quickly where the public-data model
fails on our ground, which tells us which frames are worth labelling
first: tile joints, painted lines, shadows and so on.

For each image it saves the image with the predicted crack pixels in red,
plus the raw probability map as an 8-bit PNG.
"""

import argparse
import logging
from pathlib import Path

import numpy as np
from PIL import Image

from crackseg.data.converters import IMAGE_EXTS
from crackseg.inference import load_checkpoint, predict_image
from crackseg.utils import pick_device, setup_logging

log = logging.getLogger("predict")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--images", required=True, help="folder with images")
    parser.add_argument("--out", default=None, help="default: <images>_pred next to the folder")
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    setup_logging()

    device = pick_device(args.device)
    model, ckpt = load_checkpoint(args.checkpoint, device)
    threshold = args.threshold if args.threshold is not None else ckpt["threshold"]

    in_dir = Path(args.images)
    out_dir = Path(args.out) if args.out else in_dir.with_name(in_dir.name + "_pred")
    out_dir.mkdir(parents=True, exist_ok=True)

    paths = sorted(p for p in in_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    for path in paths:
        image = np.array(Image.open(path).convert("RGB"))
        prob = predict_image(model, image, device)
        vis = image.copy()
        vis[prob >= threshold] = (255, 0, 0)
        Image.fromarray(vis).save(out_dir / f"{path.stem}_overlay.png")
        Image.fromarray((prob * 255).astype(np.uint8)).save(out_dir / f"{path.stem}_prob.png")
        log.info("%s: %.2f%% of pixels flagged as crack", path.name, 100 * (prob >= threshold).mean())

    log.info("Saved %d predictions to %s", len(paths), out_dir)


if __name__ == "__main__":
    main()
