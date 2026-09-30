"""Convert the raw datasets into one layout and write the split index.

    python scripts/prepare_data.py

Looks at every dataset folder in data/raw/, converts the ones it knows,
assigns splits and writes data/processed/index.csv. At the end it prints
a table with image counts and the share of crack pixels per split, which
is worth a look before training (a split with almost no crack pixels
means something went wrong in the conversion).
"""

import argparse
import logging
import shutil
from collections import defaultdict

import numpy as np
from PIL import Image

from crackseg.data.converters import CONVERTERS
from crackseg.data.splits import assign_splits, write_index
from crackseg.utils import resolve_path, setup_logging

log = logging.getLogger("prepare")


def summarise(samples, processed_root):
    stats = defaultdict(lambda: [0, 0, 0])  # images, crack pixels, all pixels
    for s in samples:
        mask = np.array(Image.open(processed_root / s.mask)) > 0
        row = stats[(s.source, s.split)]
        row[0] += 1
        row[1] += int(mask.sum())
        row[2] += mask.size

    print(f"\n{'dataset':14s}{'split':7s}{'images':>8s}{'crack %':>9s}")
    for (source, split), (n, crack, total) in sorted(stats.items()):
        print(f"{source:14s}{split:7s}{n:8d}{100 * crack / total:9.2f}")
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--raw-root", default="data/raw")
    parser.add_argument("--processed-root", default="data/processed")
    parser.add_argument("--val-fraction", type=float, default=0.15)
    parser.add_argument("--test-fraction", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    setup_logging()

    raw_root = resolve_path(args.raw_root)
    out_root = resolve_path(args.processed_root)

    samples = []
    for name, convert in CONVERTERS.items():
        raw_dir = raw_root / name
        if not raw_dir.exists() or not any(raw_dir.iterdir()):
            log.info("%s: not downloaded, skipping", name)
            continue
        # Start clean so images removed upstream do not linger in processed/.
        if (out_root / name).exists():
            shutil.rmtree(out_root / name)
        converted = convert(raw_dir, out_root)
        log.info("%s: %d image/mask pairs", name, len(converted))
        samples.extend(converted)

    if not samples:
        raise SystemExit("Nothing to prepare. Run scripts/download_data.py first.")

    assign_splits(samples, args.val_fraction, args.test_fraction, args.seed)
    write_index(samples, out_root / "index.csv")
    log.info("Wrote %s", out_root / "index.csv")
    summarise(samples, out_root)


if __name__ == "__main__":
    main()
