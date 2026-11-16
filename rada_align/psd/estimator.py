"""Error estimate from the PSD readings and the pass criterion."""

from __future__ import annotations

import numpy as np

from .config import TOLERANCE_DEG
from .model import Geometry, PSDS

ROWS = {"A": (0, 1), "B": (2, 3), "C": (4, 5)}


class PsdEstimator:
    """Least squares over the valid PSDs, using the controller's NOMINAL model
    (measured range, no knowledge of the hidden calibration residuals)."""

    def __init__(self, geo_nominal: Geometry):
        lin = geo_nominal.jacobians()
        self.G = lin["G"]
        self.Jj = lin["Jj"]
        self._cache: dict[tuple[str, ...], np.ndarray] = {}

    def estimator_for(self, use: tuple[str, ...]) -> np.ndarray:
        if use not in self._cache:
            rows = [r for k in use for r in ROWS[k]]
            E = np.zeros((5, 6))
            E[:, rows] = np.linalg.pinv(self.G[rows])
            self._cache[use] = E
        return self._cache[use]

    def estimate(self, readings: np.ndarray, use: tuple[str, ...]) -> np.ndarray:
        """[eh_az, eh_el, er_az, er_el, roll] in degrees. Roll is only meaningful
        when both A and C are used."""
        return self.estimator_for(use) @ readings


def readings_vector(samples) -> np.ndarray:
    return np.array([v for k in PSDS for v in (samples[k].x_mm, samples[k].y_mm)])


def window_score(errors: np.ndarray) -> dict:
    """|mean| + 2 sigma per axis over the 4 Az/El error axes (CLAUDE.md)."""
    e = np.asarray(errors)[:, :4]
    mean = e.mean(axis=0)
    sigma = e.std(axis=0, ddof=1) if len(e) > 1 else np.zeros(4)
    score = np.abs(mean) + 2 * sigma
    return {"mean": mean, "sigma": sigma, "score": score, "pass": bool(np.all(score < TOLERANCE_DEG))}
