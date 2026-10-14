#!/usr/bin/env python3
"""Score the hazard heights against the tape-measured objects of the 2026-10-08 indoor runs.
Run eval_groundtruth.py on the three bags first; it writes <bag>_repo.json here."""
import json, numpy as np
from groundtruth_objects import T, TR

# before = old height (90th percentile of the pixels above 15 mm), after = median over the grown outline
print(f"{'bag':26s} {'object':11s} {'true':>4s} {'before':>6s} {'after':>6s}")
allb, alla = [], []
for bag, tracks in TR.items():
    D = [d for d in json.load(open(f"{bag}_repo.json"))["mask"] if d["type"] == "bump"]
    for obj, ta, tb, sel in tracks:
        G = [d for d in D if ta <= d["t"] <= tb and sel(d)]
        if not G:
            print(f"{bag:26s} {obj:11s} not found"); continue
        c = [d for d in G if d["y"] > 3 and d["y"] + d["h"] < 340] or G
        b, a = np.median([d["old_p90"] for d in c]), np.median([d["g_med"] for d in c]); tr = T[obj][2]
        allb.append(b - tr); alla.append(a - tr)
        print(f"{bag:26s} {obj:11s} {tr:4d} {b:6.0f} {a:6.0f}")
allb, alla = np.array(allb), np.array(alla)
print(f"\n{len(alla)} passes. before: mean |err| {np.abs(allb).mean():.1f} mm, max {np.abs(allb).max():.1f}, bias {allb.mean():+.1f}   "
      f"after: mean |err| {np.abs(alla).mean():.1f} mm, max {np.abs(alla).max():.1f}, bias {alla.mean():+.1f}")
