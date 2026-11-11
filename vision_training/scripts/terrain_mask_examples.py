"""Draw the per-patch terrain mask on a few frames of a bag.

    python scripts/terrain_mask_examples.py --model ../src/rgb_terrain/models/rgb_terrain_v1.onnx \
        --bag D:/bags/garden_surfaces_1628 --out ../docs/media

One image per surface plus one at a surface change, where the patches can
disagree. The default times are for garden_surfaces_1628. For the change
the script looks around the given time and keeps the frame whose patches
disagree the most. Writes <out>/terrain_mask_<bag number>_<name>.png.
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy"))
from terrain_onnx import UNKNOWN, TerrainClassifier  # noqa: E402

from terrainclf.bagframes import colour_frames  # noqa: E402

# Same colours as the other surface figures (docs/media/roughness_per_surface.png).
COLOURS = {"asphalt": "#2a78d6", "red_path": "#eb6834", "wood_chips": "#1baf7a",
           "grass_dirt": "#eda100", "tile": "#e87ba4"}
NAMES = {"asphalt": "asphalt", "red_path": "red path", "wood_chips": "wood chips",
         "grass_dirt": "grass/dirt", "tile": "tile"}

# name -> (time in s, search window for the most mixed frame or 0)
SHOTS_1628 = {"red_path": (10, 0), "wood_chips": (35, 0), "grass_dirt": (55, 0),
              "asphalt": (75, 0), "change": (45, 3)}


def pick_frames(bag, shots, fps=2.0):
    """Frames nearest each requested time, plus every frame in the search windows."""
    wanted = {}
    for t, rgb in colour_frames(bag, fps):
        for name, (target, window) in shots.items():
            if abs(t - target) <= max(window, 0.5 / fps):
                wanted.setdefault(name, []).append((t, rgb))
    return wanted


def draw(rgb, mask, conf, patch_probs, clf, title, path):
    fig, ax = plt.subplots(figsize=(6.4, 5.3), dpi=120)
    ax.imshow(rgb)
    overlay = np.zeros((*mask.shape, 4))
    for i, name in enumerate(clf.classes):
        overlay[mask == i] = (*matplotlib.colors.to_rgb(COLOURS[name]), 0.45)
    ax.imshow(overlay)
    # Robot strip and side columns: never classified.
    h, w = mask.shape
    ax.add_patch(Rectangle((0, 400), w, h - 400, color="black", alpha=0.45, lw=0))
    for (x, y, bw, bh), p in zip(clf.boxes, patch_probs):
        best = int(p.argmax())
        unsure = mask[y + bh // 2, x + bw // 2] == UNKNOWN
        ax.add_patch(Rectangle((x, y), bw, bh, fill=False, ec="white", lw=1.2, ls="--" if unsure else "-"))
        text = f"{'unknown' if unsure else NAMES[clf.classes[best]]}\n{p[best]:.2f}"
        ax.text(x + bw / 2, y + bh / 2, text, ha="center", va="center", fontsize=10, color="white",
                bbox=dict(boxstyle="round,pad=0.25", fc="black", alpha=0.55, lw=0))
    ax.text(w / 2, 440, "robot, not classified (255)", ha="center", va="center", color="white", fontsize=9)
    ax.set_title(title, fontsize=11, loc="left")
    ax.set_axis_off()
    ax.legend(handles=[Patch(color=COLOURS[c], label=NAMES[c]) for c in clf.classes],
              loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=5, frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default="../docs/media")
    args = ap.parse_args()

    clf = TerrainClassifier(args.model)
    bag = Path(args.bag)
    number = bag.name.rsplit("_", 1)[-1]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    for name, frames in pick_frames(bag, SHOTS_1628).items():
        target, window = SHOTS_1628[name]
        results = [(t, rgb, *clf.predict_mask(rgb)) for t, rgb in frames]
        if window:   # the most mixed frame: most different labels, then the least sure
            t, rgb, mask, conf, probs = max(
                results, key=lambda r: (len(set(r[4].argmax(1))), -r[4].max(1).mean()))
        else:
            t, rgb, mask, conf, probs = min(results, key=lambda r: abs(r[0] - target))
        label, frame_conf, _ = clf.predict(rgb)
        title = f"{bag.name}, {t:.1f} s. Whole frame: {NAMES[label]} {frame_conf:.2f}"
        path = out / f"terrain_mask_{number}_{name}.png"
        draw(rgb, mask, conf, probs, clf, title, path)
        print(f"{path.name}: t={t:.1f} s, patches " +
              ", ".join(f"{clf.classes[p.argmax()]} {p.max():.2f}" for p in probs))


if __name__ == "__main__":
    main()
