"""Error estimate from the PSD readings and the pass criterion."""

from __future__ import annotations

import numpy as np

from .config import TOLERANCE_DEG
from .model import DEG, PSDS, Biases, Geometry, unit_sigmas_deg

ROWS = {"A": (0, 1), "B": (2, 3), "C": (4, 5)}


class PsdEstimator:
    """Least squares over the valid PSDs, using the controller's NOMINAL model
    (measured range, no knowledge of the hidden calibration residuals)."""

    def __init__(self, geo_nominal: Geometry):
        self.geo = geo_nominal
        lin = geo_nominal.jacobians()
        self.G = lin["G"]
        self.Jj = lin["Jj"]
        self.roll_incl = lin["roll_incl"]
        self.roll_from_incl = geo_nominal.cfg.control.roll_from_inclinometers
        self._cache4: dict[tuple[str, ...], np.ndarray] = {}
        self._cache: dict[tuple[str, ...], np.ndarray] = {}

    def estimator_for(self, use: tuple[str, ...]) -> np.ndarray:
        if use not in self._cache:
            rows = [r for k in use for r in ROWS[k]]
            E = np.zeros((5, 6))
            E[:, rows] = np.linalg.pinv(self.G[rows])
            self._cache[use] = E
        return self._cache[use]

    def estimator4_for(self, use: tuple[str, ...]) -> np.ndarray:
        """Az/El errors only (4 x 6); roll is supplied by the inclinometers."""
        if use not in self._cache4:
            rows = [r for k in use for r in ROWS[k]]
            E = np.zeros((4, 6))
            E[:, rows] = np.linalg.pinv(self.G[rows][:, :4])
            self._cache4[use] = E
        return self._cache4[use]

    def roll_from_inclinometers(self, inc: dict) -> float:
        """Relative roll (deg) from the two gravity rolls."""
        return self.roll_incl[0] * inc["horn_roll"] + self.roll_incl[1] * inc["radar_roll"]

    def u_bias_deg(self, use: tuple[str, ...]) -> np.ndarray:
        """Expanded (k = 2) calibration uncertainty per Az/El error axis, in degrees.

        Propagates the 1-sigma residuals the calibration file declares (config.yaml
        psd.calib and laser.differential_sigma_deg, assigned by mounting point)
        through the readings and this estimator. The PSDs cannot see these biases,
        so they are reported next to the measured score, never subtracted."""
        cfg = self.geo.cfg
        board, b_null = unit_sigmas_deg(cfg, self.geo.emitter)
        diff = cfg.laser.differential_sigma_deg
        pos = cfg.calib.psd_position_sigma_mm
        E = self.estimator4_for(use) if self.roll_from_incl else self.estimator_for(use)
        z6, z4 = self.geo.att0, np.zeros(4)
        r0 = self.geo.readings_vector(z6, z4)

        def effect(bias: Biases) -> np.ndarray:
            return (E @ (self.geo.readings_vector(z6, z4, bias) - r0))[:4]

        var = np.zeros(4)
        for ax in (0, 1):
            unit = [0.0, 0.0]
            unit[ax] = 1e-3 * DEG  # 0.001 deg probe, linear regime
            common = Biases(laser_dir={k: tuple(unit) for k in PSDS})
            var += (effect(common) / 1e-3 * board) ** 2
            var += (effect(Biases(b_null=tuple(unit))) / 1e-3 * b_null) ** 2
            for k in PSDS:
                single = Biases(laser_dir={j: (tuple(unit) if j == k else (0.0, 0.0)) for j in PSDS})
                var += (effect(single) / 1e-3 * diff) ** 2
                mm = [0.0, 0.0]
                mm[ax] = 1.0
                shifted = Biases(psd_pos_mm={j: (tuple(mm) if j == k else (0.0, 0.0)) for j in PSDS})
                var += (effect(shifted) * pos) ** 2
        if self.roll_from_incl:
            # inclinometer roll bias (one per side) through the units' lever below the axis
            s = cfg.inclinometer.bias_sigma_deg
            per_deg = E @ self.G[:, 4]
            var += (per_deg * s * self.roll_incl[0]) ** 2 + (per_deg * s * self.roll_incl[1]) ** 2
        return 2.0 * np.sqrt(var)

    def estimate(self, readings: np.ndarray, use: tuple[str, ...], inc: dict | None = None) -> np.ndarray:
        """[eh_az, eh_el, er_az, er_el, roll] in degrees.

        With `inc` (inclinometer readings) and control.roll_from_inclinometers the
        roll comes from gravity and the PSDs solve Az/El only: the units hang below
        the axes, so a roll error would otherwise leak into azimuth through that
        lever. Without it roll is only meaningful when both A and C are used."""
        if inc is not None and self.roll_from_incl:
            roll = self.roll_from_inclinometers(inc)
            e4 = self.estimator4_for(use) @ (readings - self.G[:, 4] * roll)
            return np.array([*e4, roll])
        return self.estimator_for(use) @ readings


def readings_vector(samples) -> np.ndarray:
    return np.array([v for k in PSDS for v in (samples[k].x_mm, samples[k].y_mm)])


def window_score(errors: np.ndarray, u_bias: np.ndarray | None = None) -> dict:
    """|mean| + 2 sigma per axis over the 4 Az/El error axes (CLAUDE.md).

    With `u_bias` (k = 2 calibration uncertainty) also score + U_bias, the same
    convention as the superseded controller (score_with_bias_deg)."""
    e = np.asarray(errors)[:, :4]
    mean = e.mean(axis=0)
    sigma = e.std(axis=0, ddof=1) if len(e) > 1 else np.zeros(4)
    score = np.abs(mean) + 2 * sigma
    out = {"mean": mean, "sigma": sigma, "score": score, "pass": bool(np.all(score < TOLERANCE_DEG))}
    if u_bias is not None:
        swb = score + u_bias
        out.update({"u_bias": np.asarray(u_bias), "score_with_bias": swb,
                    "pass_with_bias": bool(np.all(swb < TOLERANCE_DEG))})
    return out
