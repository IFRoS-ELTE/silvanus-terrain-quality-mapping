"""Small helpers shared by the scripts: config, seeding, device, logging."""

from __future__ import annotations

import copy
import logging
import random
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import yaml

# vision_training/ folder. Relative paths in the configs are resolved
# against this, so the scripts work no matter where they are started from.
PROJECT_ROOT = Path(__file__).resolve().parents[2]


def resolve_path(p: str | Path) -> Path:
    p = Path(p)
    return p if p.is_absolute() else PROJECT_ROOT / p


def load_config(path: str | Path, overrides: list[str] | None = None) -> dict:
    """Read a YAML config and apply command line overrides.

    Overrides use dotted keys, e.g. `train.epochs=5 model.base_channels=32`.
    Values are parsed with YAML, so numbers, lists and null work as expected.
    """
    with open(path) as f:
        cfg = yaml.safe_load(f)

    for item in overrides or []:
        key, sep, raw = item.partition("=")
        if not sep:
            raise ValueError(f"Override '{item}' must look like key=value")
        node = cfg
        *parents, leaf = key.split(".")
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = yaml.safe_load(raw)
    return cfg


def save_config(cfg: dict, path: Path) -> None:
    with open(path, "w") as f:
        yaml.safe_dump(copy.deepcopy(cfg), f, sort_keys=False)


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def pick_device(preference: str = "auto") -> torch.device:
    if preference == "auto":
        if torch.cuda.is_available():
            return torch.device("cuda")
        if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            return torch.device("mps")
        return torch.device("cpu")
    return torch.device(preference)


def make_run_dir(output_root: Path, experiment: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / f"{experiment}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def setup_logging(log_file: Path | None = None, level: int = logging.INFO) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]
    if log_file is not None:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers,
        force=True,
    )
