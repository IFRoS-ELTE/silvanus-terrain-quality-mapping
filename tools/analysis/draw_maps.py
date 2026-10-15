#!/usr/bin/env python3
import numpy as np, cv2, yaml, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
SURF, INK, INK2, BLUE, ORANGE = "#fcfcfb", "#0b0b0b", "#52514e", "#2a78d6", "#eb6834"
runs = [("indoor_raw", "Indoor, scan as recorded"), ("indoor_clean", "Indoor, floor ring removed"),
        ("out1628_raw", "Outdoor 1628, scan as recorded"), ("out1628_clean", "Outdoor 1628, floor ring removed"),
        ("out1548_clean", "Outdoor 1548, floor ring removed")]
fig, axs = plt.subplots(1, 5, figsize=(22, 5.2)); fig.patch.set_facecolor(SURF)
for ax, (name, title) in zip(axs, runs):
    m = yaml.safe_load(open(f"{name}_map.yaml")); img = cv2.imread(f"{name}_map.pgm", cv2.IMREAD_GRAYSCALE)
    res, (ox, oy, _) = m["resolution"], m["origin"]; h, w = img.shape
    ax.imshow(img, cmap="gray", extent=[ox, ox + w * res, oy, oy + h * res], vmin=0, vmax=255)
    P = np.genfromtxt(f"{name}_poses.csv", delimiter=",", names=True)
    ax.plot(P["odom_x"], P["odom_y"], color=ORANGE, lw=2, label="wheel odometry")
    ax.plot(P["map_x"], P["map_y"], color=BLUE, lw=2, label="SLAM")
    occ = (img < 100).sum() * res * res
    gap = np.hypot(P["map_x"][-1] - P["odom_x"][-1], P["map_y"][-1] - P["odom_y"][-1])
    path = np.hypot(np.diff(P["odom_x"]), np.diff(P["odom_y"])).sum()
    ax.set_title(f"{title}\nSLAM vs wheels at end: {gap:.2f} m over {path:.0f} m driven", fontsize=10, color=INK)
    pad = 2.0; xs = np.r_[P["map_x"], P["odom_x"]]; ys = np.r_[P["map_y"], P["odom_y"]]
    c = ((xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2); r = max(np.ptp(xs), np.ptp(ys)) / 2 + pad
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_aspect("equal")
    ax.tick_params(colors=INK2, labelsize=8); ax.set_facecolor(SURF)
    print(f"{name:14s} map {w*res:.0f} x {h*res:.0f} m, occupied {occ:.1f} m2, SLAM-odom gap at end {gap:.2f} m, path {path:.1f} m, "
          f"start-end SLAM {np.hypot(P['map_x'][-1]-P['map_x'][0], P['map_y'][-1]-P['map_y'][0]):.2f} m / odom {np.hypot(P['odom_x'][-1]-P['odom_x'][0], P['odom_y'][-1]-P['odom_y'][0]):.2f} m")
axs[0].legend(loc="lower left", fontsize=9, frameon=False)
fig.text(0.01, 0.01, "Metres. Black = what SLAM marked as obstacles. Lab SLAM Toolbox config, run offline from the bags.", fontsize=9, color=INK2)
fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig("slam_maps.png", dpi=95, facecolor=SURF)
