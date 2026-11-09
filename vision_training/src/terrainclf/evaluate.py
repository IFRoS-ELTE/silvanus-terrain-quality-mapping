"""Accuracy per surface and the confusion matrix, on whole frames."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# One blue ramp, light to dark (the team's chart palette), so a darker cell
# simply means "more frames". Rainbows make amounts look like categories.
RAMP = ["#ffffff", "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


def confusion(true_idx: np.ndarray, pred_idx: np.ndarray, n: int) -> np.ndarray:
    cm = np.zeros((n, n), dtype=int)
    np.add.at(cm, (true_idx, pred_idx), 1)
    return cm


def summarise(cm: np.ndarray, classes: list[str]) -> dict:
    """Per-class recall ("accuracy per surface"), precision and frame counts.
    Classes absent from the test runs get None instead of a fake 0."""
    per_class = {}
    for i, c in enumerate(classes):
        n_true, n_pred = cm[i].sum(), cm[:, i].sum()
        per_class[c] = {
            "frames": int(n_true),
            "accuracy": round(cm[i, i] / n_true, 4) if n_true else None,
            "precision": round(cm[i, i] / n_pred, 4) if n_pred else None,
        }
    present = [c for c in classes if per_class[c]["frames"]]
    return {
        "frames": int(cm.sum()),
        "accuracy": round(np.trace(cm) / cm.sum(), 4) if cm.sum() else None,
        # Mean over surfaces, so the big grass class does not hide a weak one.
        "balanced_accuracy": round(float(np.mean([per_class[c]["accuracy"] for c in present])), 4)
        if present else None,
        "per_class": per_class,
    }


def results_markdown(summary: dict, title: str) -> str:
    lines = [f"**{title}**: {summary['frames']} frames, accuracy {summary['accuracy']:.1%}, "
             f"balanced accuracy {summary['balanced_accuracy']:.1%}", "",
             "| Surface | Test frames | Accuracy | Precision |", "|---|---|---|---|"]
    for c, m in summary["per_class"].items():
        if not m["frames"]:
            continue
        prec = "n/a" if m["precision"] is None else f"{m['precision']:.1%}"
        lines.append(f"| {c} | {m['frames']} | {m['accuracy']:.1%} | {prec} |")
    return "\n".join(lines) + "\n"


def plot_confusion(cm: np.ndarray, classes: list[str], path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    rows = cm.sum(axis=1, keepdims=True)
    share = np.divide(cm, rows, out=np.zeros_like(cm, dtype=float), where=rows > 0)
    cmap = LinearSegmentedColormap.from_list("blue_ramp", RAMP)

    fig, ax = plt.subplots(figsize=(1.25 * len(classes) + 2.2, 1.05 * len(classes) + 1.6))
    ax.imshow(share, cmap=cmap, vmin=0, vmax=1)
    for i in range(len(classes)):
        for j in range(len(classes)):
            if rows[i, 0] == 0:
                continue
            dark = share[i, j] > 0.55
            ax.text(j, i, f"{share[i, j]:.0%}\n{cm[i, j]}", ha="center", va="center", fontsize=9,
                    color="white" if dark else "#1f1f1f")
    # Thin white lines between cells so neighbours stay apart even when equal.
    ax.set_xticks(np.arange(len(classes) + 1) - 0.5, minor=True)
    ax.set_yticks(np.arange(len(classes) + 1) - 0.5, minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", length=0)
    ax.set_xticks(range(len(classes)), [c.replace("_", " ") for c in classes], rotation=30, ha="right")
    ax.set_yticks(range(len(classes)), [c.replace("_", " ") for c in classes])
    ax.set_xlabel("predicted")
    ax.set_ylabel("true surface")
    ax.set_title(title, fontsize=10, loc="left")
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.text(0.01, 0.01, "Share of each true surface's frames (row %) and frame count.",
             fontsize=8, color="#555555")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_all(cm, classes, out_dir: Path, title: str) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = summarise(cm, classes)
    summary["confusion_matrix"] = {"classes": classes, "counts": cm.tolist()}
    (out_dir / "metrics.json").write_text(json.dumps(summary, indent=2))
    (out_dir / "results.md").write_text(results_markdown(summary, title))
    plot_confusion(cm, classes, out_dir / "confusion_matrix.png", title)
    return summary
