"""Tape-measured objects of the 2026-10-08 indoor runs, and when each one is in view.

TR[bag] lists (object, t_from, t_to, selector): a bump detected in that time window
(seconds from the start of the bag) for which selector(d) is true is that object.
Used by score_groundtruth.py and render_detection.py."""

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
