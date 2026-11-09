"""Train and test the RGB terrain classifier, one fold per held-out run.

    python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml
    python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml --fold test_1628_1447
    python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml --final

For every fold in the config: train on all labelled bags except the fold's
test bags, then classify every frame of the test bags. A frame's answer is
the average of its patch probabilities. Results go to
runs/<experiment>_<time>/<fold>/ (metrics.json, results.md with the
per-surface table, confusion_matrix.png, model.pt), plus summary.md for all
folds together.

--final trains one model on every labelled bag with the same recipe, for
the robot. It has no test score of its own: the folds are its test.
"""

import argparse
import logging
import time

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, WeightedRandomSampler
from tqdm import tqdm

from crackseg.utils import (load_config, make_run_dir, pick_device, resolve_path, save_config,
                            seed_everything, setup_logging)
from terrainclf.data import PatchDataset, eval_transform, read_frames, train_transform
from terrainclf.evaluate import confusion, save_all
from terrainclf.labels import SurfaceLabels
from terrainclf.model import build_model
from terrainclf.patches import patch_boxes

log = logging.getLogger("terrain_train")


def train_model(cfg, frames, classes, boxes, device):
    d, t = cfg["data"], cfg["train"]
    ds = PatchDataset(resolve_path(d["root"]), frames, classes, boxes, train_transform(d["input_size"]))
    sampler = WeightedRandomSampler(ds.sample_weights(), num_samples=len(ds), replacement=True)
    loader = DataLoader(ds, batch_size=t["batch_size"], sampler=sampler,
                        num_workers=d["num_workers"], pin_memory=device.type == "cuda",
                        persistent_workers=d["num_workers"] > 0)

    model = build_model(cfg["model"]["backbone"], len(classes), cfg["model"].get("pretrained", True)).to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=t["lr"], weight_decay=t["weight_decay"])
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=t["epochs"])
    loss_fn = torch.nn.CrossEntropyLoss(label_smoothing=t.get("label_smoothing", 0.0))

    for epoch in range(1, t["epochs"] + 1):
        model.train()
        start, total, correct, n = time.time(), 0.0, 0, 0
        for x, y, _ in tqdm(loader, desc=f"epoch {epoch}", leave=False):
            x, y = x.to(device, non_blocking=True), y.to(device, non_blocking=True)
            opt.zero_grad(set_to_none=True)
            logits = model(x)
            loss = loss_fn(logits, y)
            loss.backward()
            opt.step()
            total += loss.item() * len(y)
            correct += (logits.argmax(1) == y).sum().item()
            n += len(y)
        sched.step()
        log.info("epoch %2d | loss %.4f | train patch accuracy %.3f | %.0fs",
                 epoch, total / n, correct / n, time.time() - start)
    return model


@torch.no_grad()
def classify_frames(model, cfg, frames, classes, boxes, device):
    """Returns (true class index, predicted index) per frame, from averaged patch probabilities."""
    d = cfg["data"]
    ds = PatchDataset(resolve_path(d["root"]), frames, classes, boxes, eval_transform(d["input_size"]))
    loader = DataLoader(ds, batch_size=128, shuffle=False, num_workers=d["num_workers"])
    model.eval()
    probs = np.zeros((len(frames), len(classes)))
    for x, _, frame_idx in tqdm(loader, desc="test", leave=False):
        p = torch.softmax(model(x.to(device)), dim=1).cpu().numpy()
        np.add.at(probs, frame_idx.numpy(), p)
    true = np.array([classes.index(f.label) for f in frames])
    return true, probs.argmax(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True)
    parser.add_argument("--fold", default=None, help="run only this fold")
    parser.add_argument("--final", action="store_true", help="train one model on all bags, no test")
    parser.add_argument("overrides", nargs="*")
    args = parser.parse_args()

    cfg = load_config(args.config, args.overrides)
    run_dir = make_run_dir(resolve_path(cfg["output_dir"]),
                           cfg["experiment"] + ("_final" if args.final else ""))
    setup_logging(run_dir / "train.log")
    save_config(cfg, run_dir / "config.yaml")
    device = pick_device(cfg.get("device", "auto"))

    d = cfg["data"]
    classes = SurfaceLabels.from_yaml(resolve_path(d["labels"])).classes
    mask = np.array(Image.open(resolve_path(d["robot_mask"])).convert("L"))
    boxes = patch_boxes(mask, d["patch_size"], d["patch_rows"], d["patch_cols"])
    all_frames = read_frames(resolve_path(d["root"]) / "index.csv")
    log.info("Run folder %s, device %s, %d frames, %d patches per frame",
             run_dir, device, len(all_frames), len(boxes))

    if args.final:
        seed_everything(cfg["seed"])
        model = train_model(cfg, all_frames, classes, boxes, device)
        torch.save({"model": model.state_dict(), "config": cfg, "classes": classes,
                    "boxes": boxes, "trained_on": sorted({f.bag for f in all_frames})},
                   run_dir / "model.pt")
        log.info("Saved %s. Export with scripts/terrain_export_onnx.py", run_dir / "model.pt")
        return

    folds = [f for f in cfg["folds"] if args.fold in (None, f["name"])]
    if not folds:
        raise SystemExit(f"No fold named {args.fold}")
    summary_md = [f"# Terrain classifier, {cfg['experiment']}\n",
                  "Frame-level results on runs the model never trained on. "
                  "Accuracy per surface is the share of that surface's test frames classified correctly.\n"]
    pooled = np.zeros((len(classes), len(classes)), dtype=int)

    for fold in folds:
        seed_everything(cfg["seed"])
        test_bags = set(fold["test_bags"])
        train = [f for f in all_frames if f.bag not in test_bags]
        test = [f for f in all_frames if f.bag in test_bags]
        missing = test_bags - {f.bag for f in all_frames}
        if missing:
            log.warning("%s: test bags not extracted: %s", fold["name"], sorted(missing))
        log.info("=== %s: train %d frames from %s; test %d frames from %s", fold["name"], len(train),
                 sorted({f.bag for f in train}), len(test), sorted(test_bags))

        model = train_model(cfg, train, classes, boxes, device)
        true, pred = classify_frames(model, cfg, test, classes, boxes, device)
        cm = confusion(true, pred, len(classes))
        pooled += cm
        fold_dir = run_dir / fold["name"]
        title = f"Test runs: {', '.join(sorted(test_bags))}"
        s = save_all(cm, classes, fold_dir, title)
        torch.save({"model": model.state_dict(), "config": cfg, "classes": classes, "boxes": boxes,
                    "test_bags": sorted(test_bags)}, fold_dir / "model.pt")
        log.info("%s: frame accuracy %.3f, balanced %.3f", fold["name"], s["accuracy"], s["balanced_accuracy"])
        summary_md.append((fold_dir / "results.md").read_text())

    if len(folds) > 1:
        s = save_all(pooled, classes, run_dir / "pooled", "All folds together")
        summary_md.append((run_dir / "pooled" / "results.md").read_text())
    (run_dir / "summary.md").write_text("\n".join(summary_md))
    print("\n".join(summary_md))
    log.info("Done. Results in %s", run_dir)


if __name__ == "__main__":
    main()
