"""From raw sensor readings to boresight errors, and window statistics."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from . import geometry as g
from .config import Calibration
from .interfaces import BeaconMeasurement

ERROR_AXES = ("radar_az", "radar_el", "horn_az", "horn_el")


def compute_errors_deg(
    radar_meas: BeaconMeasurement,
    horn_meas: BeaconMeasurement,
    radar_roll_deg: float,
    horn_roll_deg: float,
    distance_m: float,
    calib: Calibration,
    intr: g.Intrinsics,
) -> dict[str, float]:
    """Radar error and horn error (deg) from one pair of beacon centroids.

    Each camera's pixel angle is corrected for its calibrated mount offset
    (delta) and for parallax: the camera is not on its head's reference
    axis and the beacon it sees is not at the other head's reference point.
    """
    out = {}
    sides = (
        ("radar", radar_meas, calib.radar, calib.horn, radar_roll_deg, horn_roll_deg),
        ("horn", horn_meas, calib.horn, calib.radar, horn_roll_deg, radar_roll_deg),
    )
    for side, meas, own, other, own_roll, other_roll in sides:
        az, el = g.pixel_to_angles_rad(meas.u_px, meas.v_px, intr)
        net = g.net_offset_m(
            own.camera_offset_m,
            other.beacon_offset_m,
            math.radians(own_roll),
            math.radians(other_roll),
        )
        err_az, err_el = g.boresight_error_rad(
            az, el, math.radians(own.delta_az_deg), math.radians(own.delta_el_deg), net, distance_m
        )
        out[f"{side}_az"] = math.degrees(err_az)
        out[f"{side}_el"] = math.degrees(err_el)
    return out


@dataclass(frozen=True)
class AxisStats:
    mean: float
    sigma: float  # sample standard deviation of the individual samples
    n: int

    @property
    def score(self) -> float:
        """Left side of the pass criterion: |mean| + 2*sigma."""
        return abs(self.mean) + 2.0 * self.sigma


class WindowEstimator:
    """Moving average over a window of samples, per named axis.

    With `window_s` set it is a rolling window (old samples drop out);
    without it, it accumulates until `reset()`.
    """

    def __init__(self, window_s: float | None = None):
        self.window_s = window_s
        self._samples: deque[tuple[float, Mapping[str, float]]] = deque()

    def reset(self) -> None:
        self._samples.clear()

    def add(self, sample: Mapping[str, float], t_s: float) -> None:
        self._samples.append((t_s, sample))
        if self.window_s is not None:
            while t_s - self._samples[0][0] > self.window_s + 1e-9:
                self._samples.popleft()

    @property
    def n(self) -> int:
        return len(self._samples)

    @property
    def span_s(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    def stats(self) -> dict[str, AxisStats]:
        if not self._samples:
            return {}
        n = len(self._samples)
        out = {}
        for axis in self._samples[0][1]:
            values = np.array([sample[axis] for _, sample in self._samples])
            sigma = float(values.std(ddof=1)) if n > 1 else 0.0
            out[axis] = AxisStats(float(values.mean()), sigma, n)
        return out


class RunningMean:
    """Plain cumulative mean of a slowly varying quantity."""

    def __init__(self):
        self._sum = 0.0
        self._n = 0

    def add(self, value: float) -> None:
        self._sum += value
        self._n += 1

    @property
    def mean(self) -> float | None:
        return self._sum / self._n if self._n else None
