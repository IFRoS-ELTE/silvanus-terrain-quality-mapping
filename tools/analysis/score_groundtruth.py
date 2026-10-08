#!/usr/bin/env python3
"""Score the hazard heights against the tape-measured objects of the 2026-10-08 indoor runs.
Run eval_groundtruth.py on the three bags first; it writes <bag>_repo.json here."""
import json, numpy as np
T = {  # object: (length, width, height) mm
 "foam block": (255, 205, 50), "foam strip": (950, 200, 20), "wood block": (315, 65, 15), "book": (200, 140, 30), "long plank": (1110, 82, 20)}
TR = {
 "corridor_groundtruth_1447": [("foam block", 7.0, 11.3, lambda d: d["cx"] > 450), ("foam strip", 16.3, 20.1, lambda d: d["L_old"] > 500),
   ("wood block", 19.6, 23.0, lambda d: 520 < d["cx"] < 620 and d["old_p90"] < 30), ("wood block", 22.8, 25.6, lambda d: 280 < d["cx"] < 520 and d["old_p90"] < 40),
   ("foam strip", 29.2, 33.6, lambda d: d["L_old"] > 500)],
 "crossing_slow_1652": [("foam block", 6.8, 9.0, lambda d: d["cx"] > 450), ("foam strip", 11.9, 14.8, lambda d: d["L_old"] > 600),
   ("wood block", 17.2, 19.9, lambda d: d["W_old"] < 100), ("foam strip", 21.3, 23.5, lambda d: d["L_old"] > 600),
   ("wood block", 25.8, 28.6, lambda d: d["W_old"] < 100), ("book", 30.1, 33.4, lambda d: True), ("long plank", 38.5, 41.2, lambda d: d["L_old"] > 900),
   ("long plank", 65.5, 68.3, lambda d: d["L_old"] > 900), ("book", 72.0, 74.1, lambda d: True), ("wood block", 75.5, 77.6, lambda d: d["W_old"] < 100),
   ("foam strip", 78.6, 80.3, lambda d: d["L_old"] > 600), ("foam strip", 84.8, 86.3, lambda d: d["L_old"] > 600), ("foam block", 88.5, 89.5, lambda d: True)],
 "crossing_fast_1703": [("foam block", 5.5, 6.4, lambda d: True), ("foam strip", 9.0, 10.6, lambda d: d["L_old"] > 600),
   ("wood block", 11.0, 11.9, lambda d: d["W_old"] < 100), ("foam strip", 13.1, 14.0, lambda d: d["L_old"] > 600),
   ("wood block", 14.9, 15.8, lambda d: d["W_old"] < 100), ("book", 17.0, 17.9, lambda d: True), ("long plank", 30.5, 31.4, lambda d: d["L_old"] > 300)],
}

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
