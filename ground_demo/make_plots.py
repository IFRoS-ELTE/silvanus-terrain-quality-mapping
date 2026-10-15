#!/usr/bin/env python3
"""Turn the logs from ground_probe into the three figures worth showing.

  fig1_noise_vs_range.png   unevenness on FLAT ground against range.
                            This is the sensor noise floor, and it is what
                            sets the usable band, which sets the arm pose.
  fig2_surface.png          unevenness per surface type. Shows the metric
                            separates real surfaces above that noise floor.
  fig3_detection.png        measured object size against distance ahead.
                            Shows how far out a hazard is still measurable.

Run:  python3 make_plots.py
"""
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# validated categorical palette, fixed order, never cycled
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
       "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
BUMP, HOLE = "#e34948", "#2a78d6"     # diverging poles, same as the live view

# colour follows the surface, not its rank, so adding one never repaints the rest
SLOT = {"tile": 0, "concrete": 1, "carpet": 2, "asphalt": 3,
        "grass": 4, "gravel": 5, "dirt": 6, "other": 7}
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#b8b6b0"
SURFACE = "#fcfcfb"


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, color=INK, fontsize=12, pad=12, loc="left")
    ax.set_xlabel(xlabel, color=INK2, fontsize=10)
    ax.set_ylabel(ylabel, color=INK2, fontsize=10)
    ax.grid(True, color=MUTED, linewidth=0.6, alpha=0.45)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=INK2, labelsize=9, length=3, width=0.8)


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=170, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


def read(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [r for r in csv.DictReader(f)]


def f(row, key):
    try:
        return float(row[key])
    except (TypeError, ValueError, KeyError):
        return float("nan")


def fig_noise(frames):
    flat = [r for r in frames if r["label"] == "flat_calib"]
    if len(flat) < 10:
        print("skip fig1: log at least 10 frames on flat ground with label 1")
        return
    x = np.array([f(r, "mean_range_m") for r in flat])
    y = np.array([f(r, "unevenness_mm") for r in flat])
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]

    fig, ax = plt.subplots(figsize=(7.2, 4.4), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.scatter(x, y, s=26, color=CAT[0], alpha=0.35, linewidths=0, zorder=2)

    edges = np.arange(x.min(), x.max() + 0.101, 0.10)
    cx, cy = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (x >= a) & (x < b)
        if m.sum() >= 3:
            cx.append((a + b) / 2)
            cy.append(np.median(y[m]))
    if cx:
        ax.plot(cx, cy, color=CAT[0], linewidth=2, marker="o", markersize=5,
                markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=3)

    # the working limit: where noise reaches a third of a 20 mm step
    ax.axhline(6.7, color=INK2, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
    ax.annotate("6.7 mm: noise budget for a 20 mm step",
                xy=(x.max(), 6.7), xytext=(-6, 6), textcoords="offset points",
                ha="right", color=INK2, fontsize=9)

    style(ax, "Depth noise sets the usable band",
          "range to ground (m)", "unevenness on flat ground (mm RMS)")
    save(fig, "fig1_noise_vs_range.png")


def fig_surface(frames):
    by = defaultdict(list)
    for r in frames:
        if r["label"] not in ("flat_calib", "unlabelled"):
            v = f(r, "unevenness_mm")
            if np.isfinite(v):
                by[r["label"]].append(v)
    by = {k: v for k, v in by.items() if len(v) >= 5}
    if len(by) < 2:
        print("skip fig2: log at least two labelled surfaces (keys 2-9)")
        return

    names = sorted(by, key=lambda k: np.median(by[k]))
    data = [by[n] for n in names]
    fig, ax = plt.subplots(figsize=(7.2, 4.4), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    bp = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=False,
                    medianprops=dict(color=INK, linewidth=2),
                    whiskerprops=dict(color=MUTED, linewidth=1.2),
                    capprops=dict(color=MUTED, linewidth=1.2))
    for name, box in zip(names, bp["boxes"]):
        box.set(facecolor=CAT[SLOT.get(name, 7) % len(CAT)], alpha=0.85,
                edgecolor=SURFACE, linewidth=2)
    for i, (n, vals) in enumerate(zip(names, data), start=1):
        ax.annotate(f"{np.median(vals):.1f}", xy=(i, np.median(vals)),
                    xytext=(0, 9), textcoords="offset points",
                    ha="center", color=INK, fontsize=9, fontweight="bold")
    ax.set_xticks(range(1, len(names) + 1))
    ax.set_xticklabels([n.replace("_", " ") for n in names])
    style(ax, "Surface roughness, measured by the camera alone",
          "", "unevenness (mm RMS)")
    save(fig, "fig2_surface.png")


def fig_detection(dets):
    if len(dets) < 10:
        print("skip fig3: no detections logged yet")
        return
    fig, ax = plt.subplots(figsize=(7.2, 4.4), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for kind, col in (("bump", BUMP), ("hole", HOLE)):
        rows = [r for r in dets if r["type"] == kind]
        if not rows:
            continue
        ax.scatter([f(r, "ahead_m") for r in rows], [f(r, "size_mm") for r in rows],
                   s=34, color=col, alpha=0.6, linewidths=0, label=kind, zorder=2)
    leg = ax.legend(frameon=False, loc="upper left", fontsize=9)
    for t in leg.get_texts():
        t.set_color(INK2)
    style(ax, "Measured size against distance ahead",
          "distance ahead of the camera (m)", "measured size (mm)")
    save(fig, "fig3_detection.png")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    frames = read(os.path.join(OUT, "frames.csv"))
    dets = read(os.path.join(OUT, "detections.csv"))
    print(f"{len(frames)} frame rows, {len(dets)} detection rows")
    fig_noise(frames)
    fig_surface(frames)
    fig_detection(dets)
