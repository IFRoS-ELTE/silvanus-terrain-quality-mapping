"""Assign train/val/test splits and keep them in one CSV index.

The index file (data/processed/index.csv) is the single source of truth
for which image is in which split. Training and evaluation only read
this file, so a split can be checked, versioned or edited by hand.

Rules:
- If a dataset has an official test split we keep it, so our numbers are
  comparable with published ones.
- Validation is carved out of train when the dataset does not have one.
- Splitting works on groups, not single images (see Sample.group). This
  matters for our own robot frames, the same way you would split medical
  data by patient and never by slice.
"""

from __future__ import annotations

import csv
import logging
import random
from collections import defaultdict
from dataclasses import asdict, fields
from pathlib import Path

from .converters import Sample

log = logging.getLogger(__name__)

INDEX_COLUMNS = [f.name for f in fields(Sample)]


def _split_groups(groups: list[str], fractions: dict[str, float], seed: int) -> dict[str, str]:
    """Shuffle groups with a fixed seed and cut them by the given fractions."""
    groups = sorted(groups)  # sort first so the result does not depend on input order
    random.Random(seed).shuffle(groups)

    assignment = {}
    start = 0
    names = list(fractions)
    for i, name in enumerate(names):
        if i == len(names) - 1:
            end = len(groups)  # last split takes the rounding leftovers
        else:
            end = start + round(fractions[name] * len(groups))
        for g in groups[start:end]:
            assignment[g] = name
        start = end
    return assignment


def assign_splits(
    samples: list[Sample],
    val_fraction: float = 0.15,
    test_fraction: float = 0.15,
    seed: int = 42,
) -> list[Sample]:
    by_source: dict[str, list[Sample]] = defaultdict(list)
    for s in samples:
        by_source[s.source].append(s)

    for source_samples in by_source.values():
        # Samples without an official split: three-way split by group.
        unassigned = [s for s in source_samples if s.split is None]
        if unassigned:
            groups = sorted({s.group or s.sample_id for s in unassigned})
            fractions = {
                "val": val_fraction,
                "test": test_fraction,
                "train": 1.0 - val_fraction - test_fraction,
            }
            mapping = _split_groups(groups, fractions, seed)
            for s in unassigned:
                s.split = mapping[s.group or s.sample_id]

        # Official train but no official val: move part of train to val.
        has_val = any(s.split == "val" for s in source_samples)
        if not has_val:
            train = [s for s in source_samples if s.split == "train"]
            groups = sorted({s.group or s.sample_id for s in train})
            mapping = _split_groups(groups, {"val": val_fraction, "train": 1.0 - val_fraction}, seed)
            for s in train:
                s.split = mapping[s.group or s.sample_id]

    _drop_groups_leaking_into_test(samples)
    return samples


def _drop_groups_leaking_into_test(samples: list[Sample]) -> None:
    """Official splits are not always clean.

    DeepCrack's own split has a few photos with crops in both train and
    test. We keep the official test set as it is and mark the train/val
    crops of those photos as "unused", so no split reads them.
    """
    test_groups = {s.group for s in samples if s.split == "test"}
    dropped = [s for s in samples if s.split in ("train", "val") and s.group in test_groups]
    for s in dropped:
        s.split = "unused"
    if dropped:
        log.info("Marked %d train/val images as unused because their source photo is in test",
                 len(dropped))


def write_index(samples: list[Sample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=INDEX_COLUMNS)
        writer.writeheader()
        for s in sorted(samples, key=lambda s: (s.source, s.sample_id)):
            writer.writerow(asdict(s))


def read_index(
    path: Path, split: str | None = None, sources: list[str] | None = None
) -> list[Sample]:
    with open(path, newline="") as f:
        rows = [Sample(**row) for row in csv.DictReader(f)]
    if split is not None:
        rows = [r for r in rows if r.split == split]
    if sources:
        rows = [r for r in rows if r.source in sources]
    return rows
