"""Evaluate a trained checkpoint on the test split (or any other split).

    python scripts/evaluate.py --checkpoint runs/<run>/best.pt
    python scripts/evaluate.py --checkpoint runs/<run>/best.pt --split val --sources deepcrack

Uses the threshold that was chosen on validation during training, unless
--threshold is given. Results go to <run>/eval_<split>/:
    metrics.json      overall numbers plus one block per dataset
    per_image.csv     Dice per image, sorted from worst to best
    overlays/         the worst and best images with prediction drawn on top

Looking at the worst images is the most useful part. In our experience
with medical data, most "model errors" there turn out to be label noise
or a data problem, and you only find that by looking.
"""

import argparse
import csv
import json
import logging
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image
from torch.utils.data import DataLoader

from crackseg.data.dataset import CrackDataset
from crackseg.engine import evaluate
from crackseg.inference import load_checkpoint
from crackseg.metrics import SegmentationMeter
from crackseg.utils import pick_device, resolve_path, setup_logging

log = logging.getLogger("evaluate")


def overlay(image: np.ndarray, gt: np.ndarray, pred: np.ndarray) -> np.ndarray:
    """Green = correct crack, red = false positive, blue = missed crack."""
    out = (image.astype(np.float32) * 0.6).astype(np.uint8)
    out[pred & gt] = (0, 255, 0)
    out[pred & ~gt] = (255, 0, 0)
    out[~pred & gt] = (0, 128, 255)
    return np.concatenate([image, out], axis=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--split", default="test")
    parser.add_argument("--sources", nargs="*", default=None)
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--num-overlays", type=int, default=8, help="worst and best N images")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    setup_logging()

    ckpt_path = Path(args.checkpoint)
    out_dir = ckpt_path.parent / f"eval_{args.split}"
    (out_dir / "overlays").mkdir(parents=True, exist_ok=True)

    device = pick_device(args.device)
    model, ckpt = load_checkpoint(ckpt_path, device)
    cfg = ckpt["config"]
    threshold = args.threshold if args.threshold is not None else ckpt["threshold"]
    log.info("Checkpoint from epoch %d, threshold %.2f", ckpt["epoch"], threshold)

    root = resolve_path(cfg["data"]["processed_root"])
    dataset = CrackDataset(root, args.split, args.sources or cfg["data"].get("eval_sources"))
    loader = DataLoader(dataset, batch_size=1, shuffle=False)
    tol = cfg["eval"]["tolerance_px"]

    meter, _, preds = evaluate(model, loader, device, thresholds=[threshold],
                               tolerance_px=tol, keep_predictions=True)

    # Per dataset numbers: public datasets differ a lot (asphalt vs concrete,
    # camera, label style), so one overall number hides a lot.
    per_source = defaultdict(lambda: SegmentationMeter([threshold], tol))
    per_image = []
    by_id = {s.sample_id: s for s in dataset.samples}
    for sid, prob in preds.items():
        sample = by_id[sid]
        gt = np.array(Image.open(root / sample.mask)) > 127
        per_source[sample.source].update(prob, gt)
        single = SegmentationMeter([threshold], tol)
        single.update(prob, gt)
        per_image.append({"sample_id": sid, "source": sample.source,
                          **{k: round(v, 4) for k, v in single.compute(threshold).items()}})

    results = {
        "checkpoint": str(ckpt_path),
        "split": args.split,
        "num_images": len(preds),
        "overall": meter.compute(threshold),
        "per_source": {src: m.compute(threshold) for src, m in sorted(per_source.items())},
    }
    with open(out_dir / "metrics.json", "w") as f:
        json.dump(results, f, indent=2)

    per_image.sort(key=lambda r: r["dice"])
    with open(out_dir / "per_image.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(per_image[0]))
        writer.writeheader()
        writer.writerows(per_image)

    n = args.num_overlays
    picks = [("worst", r) for r in per_image[:n]] + [("best", r) for r in per_image[-n:]]
    for tag, r in picks:
        sample = by_id[r["sample_id"]]
        image = np.array(Image.open(root / sample.image).convert("RGB"))
        gt = np.array(Image.open(root / sample.mask)) > 127
        pred = preds[r["sample_id"]] >= threshold
        Image.fromarray(overlay(image, gt, pred)).save(
            out_dir / "overlays" / f"{tag}_{r['dice']:.3f}_{r['sample_id']}.png")

    o = results["overall"]
    print(f"\n{args.split} set, {len(preds)} images, threshold {threshold:.2f}")
    print(f"  Dice {o['dice']:.4f}  IoU {o['iou']:.4f}  P {o['precision']:.4f}  R {o['recall']:.4f}")
    print(f"  tolerant ({tol} px): P {o['tol_precision']:.4f}  R {o['tol_recall']:.4f}  F1 {o['tol_f1']:.4f}")
    for src, m in results["per_source"].items():
        print(f"  {src:14s} Dice {m['dice']:.4f}  IoU {m['iou']:.4f}  tolF1 {m['tol_f1']:.4f}")
    print(f"\nSaved to {out_dir}")


if __name__ == "__main__":
    main()
