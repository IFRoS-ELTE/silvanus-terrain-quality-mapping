"""Runs without ROS: python -m pytest src/terrain_imu/test"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from terrain_imu.roughness import RoughnessWindow, band_ratio  # noqa: E402

t = np.arange(180) / 100.0


def test_fast_shaking_gives_a_high_ratio_and_slow_a_low_one():
    fast = np.sin(2 * np.pi * 20 * t)     # 20 Hz: like hard asphalt
    slow = np.sin(2 * np.pi * 3 * t)      # 3 Hz: like soft grass
    assert band_ratio(fast) > 10
    assert band_ratio(slow) < 0.1


def test_ratio_ignores_amplitude():
    # Faster driving shakes harder, but the same surface keeps the same mix of frequencies.
    x = np.sin(2 * np.pi * 20 * t) + np.sin(2 * np.pi * 3 * t)
    assert math.isclose(band_ratio(x), band_ratio(5 * x), rel_tol=1e-9)


def test_matches_the_formula_in_outdoor_figures():
    rng = np.random.default_rng(0)
    x = rng.normal(size=180)
    fr = np.fft.rfftfreq(180, 1 / 100.0)
    p = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(180))) ** 2
    expected = p[(fr >= 8) & (fr < 50)].sum() / p[(fr >= 0.5) & (fr < 8)].sum()
    assert math.isclose(band_ratio(x), expected, rel_tol=1e-12)


def test_window_needs_180_samples_and_the_right_speed():
    w = RoughnessWindow()
    for i in range(179):
        w.add_accel(math.sin(i))
    w.add_speed(1.0, 0.3)
    assert math.isnan(w.roughness())            # window not full yet
    w.add_accel(0.0)
    assert not math.isnan(w.roughness())        # full, 0.3 m/s
    w.add_speed(1.1, 1.0)                       # mean speed now 0.65 m/s
    assert math.isnan(w.roughness())
    assert not math.isnan(w.ratio())            # the raw ratio is still there for debugging


def test_old_speeds_leave_the_window():
    w = RoughnessWindow()
    w.add_speed(0.0, 1.0)
    w.add_speed(5.0, 0.3)                       # 5 s later: the 1.0 m/s sample is older than 1.8 s
    assert math.isclose(w.mean_speed(), 0.3)
