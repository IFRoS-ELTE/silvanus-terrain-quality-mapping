"""Time ranges from docs/DATA.md turned into a label per frame."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Segment:
    start: float
    end: float
    label: str


class SurfaceLabels:
    def __init__(self, classes: list[str], bags: dict[str, list[Segment]], margin_s: float):
        self.classes = classes
        self.bags = bags
        self.margin_s = margin_s
        for bag, segments in bags.items():
            for s in segments:
                if s.label not in classes:
                    raise ValueError(f"{bag}: unknown class '{s.label}', expected one of {classes}")
                if s.end <= s.start:
                    raise ValueError(f"{bag}: segment {s} ends before it starts")

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SurfaceLabels":
        cfg = yaml.safe_load(Path(path).read_text())
        bags = {bag: [Segment(float(a), float(b), lab) for a, b, lab in segs]
                for bag, segs in cfg["bags"].items()}
        return cls(cfg["classes"], bags, float(cfg.get("margin_s", 2.0)))

    def label_at(self, bag: str, t: float) -> str | None:
        """Label of the frame at time t, or None if it is unlabelled or too
        close to a boundary to trust."""
        for s in self.bags.get(bag, []):
            if s.start + self.margin_s <= t <= s.end - self.margin_s:
                return s.label
        return None
