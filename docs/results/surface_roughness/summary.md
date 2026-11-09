# Roughness per surface

One record per 0.25 m of driven path. IMU values only at 0.25-0.50 m/s. Node replay = the 4 Hz values the IMU roughness node publishes on these bags.

| Surface | Records (0.25 m) | Camera unevenness, median mm | IMU roughness, median | Node replay, median | PROGRESS.md medians (camera, IMU) |
|---|---|---|---|---|---|
| tile (indoor) | 111 | 3.57 | 0.83 | 0.77 | n/a |
| asphalt | 89 | 4.19 | 2.55 | 2.88 | 3.9 mm, 2.2 |
| red path | 33 | 3.65 | 0.31 | 0.35 | 3.6 mm, 0.51 |
| wood chips | 19 | 7.57 | 0.39 | 0.40 | 7.5 mm, 0.41 |
| grass/dirt | 158 | 6.66 | 0.29 | 0.30 | 6.8 mm, 0.29 |

Camera unevenness against IMU roughness, 239 records with both: Spearman rho = -0.54 (p = 2e-19).
