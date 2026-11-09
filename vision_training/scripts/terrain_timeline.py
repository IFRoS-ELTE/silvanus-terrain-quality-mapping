"""Run a terrain model over a whole bag and plot what it says over time.

    python scripts/terrain_timeline.py --checkpoint runs/<run>/test_1628_1447/model.pt \
        --bag D:/bags/garden_surfaces_1628

The fold results only score frames well inside a labelled stretch. This
script classifies every frame of a run (2 per second), including the
moments where two surfaces meet and stretches that have no class at all
(kerb, concrete patch, drain grate). Use a model whose fold held this bag
out, otherwise the run was seen in training.

Writes <out>/<bag>_timeline.csv and <out>/<bag>_timeline.png: the labelled
surface on top, the predicted one below, and the model's confidence.
Agreement is reported twice: inside the labelled stretches with the usual
2 s margin, and on every labelled frame including the boundaries.
"""

import argparse
import csv
import logging
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from matplotlib.patches import Patch

from crackseg.utils import resolve_path, setup_logging
from terrainclf.bagframes import colour_frames
from terrainclf.data import eval_transform
from terrainclf.evaluate import DISPLAY
from terrainclf.labels import SurfaceLabels
from terrainclf.model import build_model
from terrainclf.patches import cut_patches

log = logging.getLogger("terrain_timeline")

# Categorical palette, fixed order, one colour per class in the order of the
# label file. Grey is reserved for "no label".
COLOURS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
NO_LABEL = "#d4d4d4"


def segments_of(times, values):
    """Collapse a per-frame series into (start, end, value) runs for drawing."""
    out, start = [], 0
    step = np.median(np.diff(times)) if len(times) > 1 else 0.5
    for i in range(1, len(values) + 1):
        if i == len(values) or values[i] != values[start]:
            out.append((times[start], times[i - 1] + step, values[start]))
            start = i
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--labels", default="configs/terrain_labels.yaml")
    parser.add_argument("--out", default=None, help="default: next to the checkpoint")
    parser.add_argument("--fps", type=float, default=2.0)
    args = parser.parse_args()
    setup_logging()

    ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    classes, boxes, cfg = ckpt["classes"], ckpt["boxes"], ckpt["config"]
    if Path(args.bag).name in ckpt.get("trained_on", []) or (
            "test_bags" in ckpt and Path(args.bag).name not in ckpt["test_bags"]):
        log.warning("%s was not held out by this checkpoint; the run may have been seen in training",
                    Path(args.bag).name)
    model = build_model(cfg["model"]["backbone"], len(classes), pretrained=False)
    model.load_state_dict(ckpt["model"])
    model.eval()
    transform = eval_transform(cfg["data"]["input_size"])

    labels = SurfaceLabels.from_yaml(resolve_path(args.labels))
    strict, margin = labels.margin_s, labels.margin_s
    bag = Path(args.bag).name

    rows = []
    with torch.no_grad():
        for t, rgb in colour_frames(args.bag, args.fps):
            x = torch.stack([transform(p) for p in cut_patches(rgb, boxes)])
            probs = torch.softmax(model(x), 1).mean(0).numpy()
            labels.margin_s = 0.0
            label_any = labels.label_at(bag, t)
            labels.margin_s = strict
            rows.append({"t": round(t, 2), "predicted": classes[int(probs.argmax())],
                         "confidence": round(float(probs.max()), 3),
                         "label": label_any or "", "scored": int(labels.label_at(bag, t) is not None)})

    out = Path(args.out) if args.out else Path(args.checkpoint).parent
    out.mkdir(parents=True, exist_ok=True)
    with open(out / f"{bag}_timeline.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    def agreement(selected):
        return np.mean([r["predicted"] == r["label"] for r in selected]) if selected else float("nan")

    inner = agreement([r for r in rows if r["scored"]])
    every = agreement([r for r in rows if r["label"]])
    unlabelled = [r for r in rows if not r["label"]]
    log.info("%s: %d frames. Agreement %.1f%% inside the labelled stretches (%d s margin), "
             "%.1f%% on every labelled frame. %d frames have no label.",
             bag, len(rows), 100 * inner, margin, 100 * every, len(unlabelled))

    # Figure: two colour bands (labelled, predicted) over time, confidence below.
    times = np.array([r["t"] for r in rows])
    colour_of = {c: COLOURS[i] for i, c in enumerate(classes)} | {"": NO_LABEL}
    fig, (ax_bands, ax_conf) = plt.subplots(2, 1, figsize=(11, 3.6), sharex=True,
                                            gridspec_kw={"height_ratios": [2, 1]})
    for y, key in ((1, "label"), (0, "predicted")):
        for a, b, v in segments_of(times, [r[key] for r in rows]):
            ax_bands.barh(y, b - a, left=a, height=0.8, color=colour_of[v], edgecolor="white", linewidth=1)
    ax_bands.set_yticks([1, 0], ["labelled", "predicted"])
    ax_bands.set_title(f"{bag}: agreement {inner:.0%} inside labelled stretches, "
                       f"{every:.0%} including boundaries", fontsize=10, loc="left")
    for spine in ax_bands.spines.values():
        spine.set_visible(False)
    ax_bands.tick_params(left=False)

    ax_conf.plot(times, [r["confidence"] for r in rows], color="#3d3d3d", linewidth=1.5)
    ax_conf.set_ylim(0, 1.05)
    ax_conf.set_ylabel("confidence", fontsize=9)
    ax_conf.set_xlabel("time from start of run, s")
    ax_conf.grid(axis="y", color="#e6e6e6", linewidth=0.8)
    for side in ("top", "right"):
        ax_conf.spines[side].set_visible(False)

    handles = [Patch(color=colour_of[c], label=DISPLAY.get(c, c)) for c in classes]
    handles.append(Patch(color=NO_LABEL, label="no label (kerb, patch, edge)"))
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), frameon=False, fontsize=8.5)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(out / f"{bag}_timeline.png", dpi=150)
    plt.close(fig)
    log.info("Wrote %s", out / f"{bag}_timeline.png")


if __name__ == "__main__":
    main()
