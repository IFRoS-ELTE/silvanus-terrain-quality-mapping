"""Train the crack segmentation model.

    python scripts/train.py --config configs/smoke_cpu.yaml
    python scripts/train.py --config configs/unet_public.yaml train.epochs=80

Every run gets its own folder under runs/ with:
    config.yaml   the exact config used, including command line overrides
    train.log     everything printed during the run
    history.csv   loss and validation metrics per epoch
    curves.png    the same as a plot
    best.pt       checkpoint with the best validation Dice
    last.pt       checkpoint after the last epoch (to resume or inspect)

The best threshold on validation is stored in best.pt. evaluate.py and
export_onnx.py pick it up from there, so test results never use a
threshold that was tuned on test.
"""

import argparse
import csv
import logging
import time

import matplotlib

matplotlib.use("Agg")  # no display on the GPU server
import matplotlib.pyplot as plt
import torch
from torch.utils.data import DataLoader

from crackseg.data.dataset import MEAN, STD, CrackDataset
from crackseg.data.transforms import TrainTransform
from crackseg.engine import evaluate, train_one_epoch
from crackseg.losses import BceDiceLoss
from crackseg.models import build_model, count_parameters
from crackseg.utils import (load_config, make_run_dir, pick_device, resolve_path,
                            save_config, seed_everything, setup_logging)

log = logging.getLogger("train")


def build_loaders(cfg):
    d = cfg["data"]
    root = resolve_path(d["processed_root"])
    train_ds = CrackDataset(
        root, "train", d.get("train_sources"),
        TrainTransform(d["crop_size"], d.get("crack_bias", 0.5), d.get("photometric", 0.25)),
    )
    val_ds = CrackDataset(root, "val", d.get("eval_sources"))

    workers = d.get("num_workers", 0)
    train_loader = DataLoader(
        train_ds, batch_size=cfg["train"]["batch_size"], shuffle=True,
        num_workers=workers, pin_memory=torch.cuda.is_available(),
        drop_last=len(train_ds) > cfg["train"]["batch_size"],
        persistent_workers=workers > 0,
    )
    # Validation images keep their original size, which differs between
    # datasets, so they cannot be stacked into batches.
    val_loader = DataLoader(val_ds, batch_size=1, shuffle=False, num_workers=workers)
    return train_loader, val_loader


def save_checkpoint(path, model, optimizer, epoch, cfg, val_metrics):
    torch.save({
        "model": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "epoch": epoch,
        "config": cfg,
        "val_metrics": val_metrics,
        "threshold": val_metrics["threshold"],
        "normalization": {"mean": MEAN.tolist(), "std": STD.tolist()},
    }, path)


def plot_history(rows, path):
    epochs = [r["epoch"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    ax1.plot(epochs, [r["train_loss"] for r in rows], label="train")
    ax1.plot(epochs, [r["val_loss"] for r in rows], label="val")
    ax1.set_xlabel("epoch"); ax1.set_ylabel("loss"); ax1.legend()
    ax2.plot(epochs, [r["val_dice"] for r in rows], label="Dice")
    ax2.plot(epochs, [r["val_tol_f1"] for r in rows], label="F1 (2 px tolerance)")
    ax2.set_xlabel("epoch"); ax2.set_ylabel("validation"); ax2.set_ylim(0, 1); ax2.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True)
    parser.add_argument("overrides", nargs="*", help="dotted key=value pairs, e.g. train.lr=0.0005")
    args = parser.parse_args()

    cfg = load_config(args.config, args.overrides)
    run_dir = make_run_dir(resolve_path(cfg["output_dir"]), cfg["experiment"])
    setup_logging(run_dir / "train.log")
    save_config(cfg, run_dir / "config.yaml")
    seed_everything(cfg["seed"])

    device = pick_device(cfg.get("device", "auto"))
    log.info("Run folder: %s", run_dir)
    log.info("Device: %s", device)

    train_loader, val_loader = build_loaders(cfg)
    log.info("Train images: %d, val images: %d",
             len(train_loader.dataset), len(val_loader.dataset))

    model = build_model(cfg["model"]).to(device)
    log.info("Model: %s with %.2fM parameters", cfg["model"]["name"], count_parameters(model) / 1e6)

    criterion = BceDiceLoss(**cfg["loss"]).to(device)
    t = cfg["train"]
    optimizer = torch.optim.AdamW(model.parameters(), lr=t["lr"], weight_decay=t["weight_decay"])
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=t["epochs"])
    scaler = torch.amp.GradScaler("cuda") if t.get("amp") and device.type == "cuda" else None
    eval_cfg = cfg["eval"]

    history = []
    best_dice, epochs_without_gain = -1.0, 0

    for epoch in range(1, t["epochs"] + 1):
        start = time.time()
        train_loss = train_one_epoch(
            model, train_loader, criterion, optimizer, device, scaler,
            max_batches=t.get("max_train_batches"), grad_clip=t.get("grad_clip"),
        )
        meter, val_loss, _ = evaluate(
            model, val_loader, device, criterion,
            thresholds=eval_cfg["thresholds"], tolerance_px=eval_cfg["tolerance_px"],
            max_batches=t.get("max_val_batches"),
        )
        scheduler.step()

        val = meter.best("dice")
        row = {
            "epoch": epoch,
            "lr": optimizer.param_groups[0]["lr"],
            "train_loss": round(train_loss, 5),
            "val_loss": round(val_loss, 5),
            "val_threshold": val["threshold"],
            "val_dice": round(val["dice"], 4),
            "val_iou": round(val["iou"], 4),
            "val_tol_f1": round(val["tol_f1"], 4),
            "seconds": round(time.time() - start, 1),
        }
        history.append(row)
        log.info("epoch %3d | train %.4f | val %.4f | Dice %.3f IoU %.3f tolF1 %.3f @%.1f | %.0fs",
                 epoch, train_loss, val_loss, val["dice"], val["iou"], val["tol_f1"],
                 val["threshold"], row["seconds"])

        with open(run_dir / "history.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            writer.writerows(history)

        save_checkpoint(run_dir / "last.pt", model, optimizer, epoch, cfg, val)
        if val["dice"] > best_dice:
            best_dice, epochs_without_gain = val["dice"], 0
            save_checkpoint(run_dir / "best.pt", model, optimizer, epoch, cfg, val)
            log.info("  new best model saved (Dice %.4f)", best_dice)
        else:
            epochs_without_gain += 1
            if epochs_without_gain >= t["early_stop_patience"]:
                log.info("No improvement for %d epochs, stopping early.", epochs_without_gain)
                break

    plot_history(history, run_dir / "curves.png")
    log.info("Finished. Best val Dice %.4f. Evaluate with:", best_dice)
    log.info("  python scripts/evaluate.py --checkpoint %s", run_dir / "best.pt")


if __name__ == "__main__":
    main()
