"""Pull labelled frames for the terrain classifier out of the rosbags.

    python scripts/terrain_extract.py --bags /path/to/bags

`--bags` is the folder holding the bag folders from the shared drive
(garden_surfaces_1548/, corridor_book_1338_indoor/, ...). Only bags listed in
configs/terrain_labels.yaml are read, only frames inside a labelled time
range (minus the boundary margin) are kept, and a missing bag is reported
and skipped. Needs no ROS, so it runs on the GPU server.

Writes data/terrain/frames/<bag>/<bag>_<seconds>.png and data/terrain/index.csv,
then prints how many frames each surface got per bag. Check that table
against docs/DATA.md before training.
"""

import argparse
import logging
from collections import Counter
from pathlib import Path

from PIL import Image

from crackseg.utils import resolve_path, setup_logging
from terrainclf.bagframes import colour_frames
from terrainclf.data import Frame, write_frames
from terrainclf.labels import SurfaceLabels

log = logging.getLogger("terrain_extract")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bags", required=True, help="folder that contains the bag folders")
    parser.add_argument("--labels", default="configs/terrain_labels.yaml")
    parser.add_argument("--out", default="data/terrain")
    parser.add_argument("--fps", type=float, default=2.0,
                        help="frames kept per second (at 0.3 m/s, 2 fps keeps a frame every 15 cm)")
    args = parser.parse_args()
    setup_logging()

    labels = SurfaceLabels.from_yaml(resolve_path(args.labels))
    out = resolve_path(args.out)
    frames: list[Frame] = []

    for bag in labels.bags:
        bag_dir = Path(args.bags) / bag
        if not bag_dir.exists():
            log.warning("%s: not found in %s, skipping", bag, args.bags)
            continue
        kept = 0
        for t, rgb in colour_frames(bag_dir, args.fps):
            label = labels.label_at(bag, t)
            if label is None:
                continue
            rel = Path("frames") / bag / f"{bag}_{t:06.2f}s.png"
            (out / rel).parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(rgb).save(out / rel)  # PNG: no JPEG artefacts in training data
            frames.append(Frame(rel.as_posix(), bag, t, label))
            kept += 1
        log.info("%s: %d labelled frames", bag, kept)

    if not frames:
        raise SystemExit("No frames extracted. Check --bags and the bag folder names.")
    write_frames(frames, out / "index.csv")

    counts = Counter((f.bag, f.label) for f in frames)
    bags = sorted({f.bag for f in frames})
    width = max(len(b) for b in bags) + 2
    print(f"\n{'bag':{width}s}" + "".join(f"{c:>12s}" for c in labels.classes))
    for b in bags:
        print(f"{b:{width}s}" + "".join(f"{counts[(b, c)]:12d}" for c in labels.classes))
    print(f"{'total':{width}s}" + "".join(
        f"{sum(counts[(b, c)] for b in bags):12d}" for c in labels.classes))
    print(f"\nWrote {len(frames)} frames and {out / 'index.csv'}")


if __name__ == "__main__":
    main()
