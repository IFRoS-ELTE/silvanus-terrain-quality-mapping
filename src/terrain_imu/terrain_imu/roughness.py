"""IMU roughness: how much of the shaking is fast rather than slow.

The first idea, the RMS of vertical acceleration, turned out to follow only
the speed (about 1.0 x speed on every surface, docs/PROGRESS.md section 3).
What does tell surfaces apart is the frequency of the shaking: hard ground
like asphalt shakes the chassis fast, soft ground like grass or wood chips
shakes it slowly. So the roughness number is

    power of vertical free acceleration at 8-50 Hz
    -----------------------------------------------
    power at 0.5-8 Hz

over the last 180 samples (1.8 s at 100 Hz), with a Hann window. This is the
same calculation as tools/analysis/outdoor_figures.py, so the node and the
published figures agree.

Two details that matter:
- The Xsens stamps arrive in bursts (74 % of messages less than 2 ms apart,
  docs/DATA.md), so the window is the last 180 messages, not the last 1.8 s
  of header stamps.
- The ratio depends on speed, so it is only compared at 0.25-0.50 m/s, the
  speed range all our runs were driven at. Outside it the value is NaN.

Plain numpy, no ROS, so the same class runs in the node and in offline
replays of the bags.
"""

from __future__ import annotations

from collections import deque

import numpy as np

SAMPLES = 180          # 1.8 s at 100 Hz
RATE_HZ = 100.0
FAST_BAND = (8.0, 50.0)
SLOW_BAND = (0.5, 8.0)
SPEED_RANGE = (0.25, 0.50)


def band_ratio(accel_z: np.ndarray, rate_hz: float = RATE_HZ) -> float:
    """Fast/slow power ratio of one window of vertical acceleration."""
    x = np.asarray(accel_z, dtype=np.float64)
    spectrum = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / rate_hz)
    fast = spectrum[(freqs >= FAST_BAND[0]) & (freqs < FAST_BAND[1])].sum()
    slow = spectrum[(freqs >= SLOW_BAND[0]) & (freqs < SLOW_BAND[1])].sum()
    return float(fast / slow) if slow > 0 else float("nan")


class RoughnessWindow:
    """Keeps the last 1.8 s of acceleration and speed and gives the current ratio."""

    def __init__(self, samples: int = SAMPLES, rate_hz: float = RATE_HZ,
                 speed_range: tuple[float, float] = SPEED_RANGE):
        self.samples = samples
        self.rate_hz = rate_hz
        self.speed_range = speed_range
        self.accel = deque(maxlen=samples)
        self.speeds: deque[tuple[float, float]] = deque()   # (time s, |forward speed| m/s)

    def add_accel(self, z: float) -> None:
        self.accel.append(z)

    def add_speed(self, t: float, v: float) -> None:
        self.speeds.append((t, abs(v)))
        window_s = self.samples / self.rate_hz
        while self.speeds and self.speeds[0][0] < t - window_s:
            self.speeds.popleft()

    def mean_speed(self) -> float:
        return float(np.mean([v for _, v in self.speeds])) if self.speeds else float("nan")

    def ratio(self) -> float:
        """Raw ratio, whatever the speed. NaN until the window is full."""
        if len(self.accel) < self.samples:
            return float("nan")
        return band_ratio(np.fromiter(self.accel, float), self.rate_hz)

    def roughness(self) -> float:
        """The published value: the ratio if the speed is in range, else NaN."""
        lo, hi = self.speed_range
        v = self.mean_speed()
        return self.ratio() if lo <= v <= hi else float("nan")
