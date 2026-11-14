"""Truth state of the simulated scene.

The world knows where everything really is. The controller never sees it:
it only sees the simulated sensors. Monte Carlo results are scored against
this truth, so biases the system cannot observe (calibration, fixture) show
up in the results.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import geometry as g
from .config import Calibration, Config, HeadCalibration
from .disturbances import WindModel

ERROR_AXES = ("radar_az", "radar_el", "horn_az", "horn_el")


@dataclass(frozen=True)
class Scenario:
    """Everything that is drawn once per setup."""

    distance_m: float
    radar_az_deg: float  # radar boresight in the world
    radar_el_deg: float
    radar_roll_deg: float
    horn_az_deg: float  # horn boresight relative to "facing the radar"
    horn_el_deg: float
    horn_roll_deg: float
    # TRUE camera offset delta (az, el): the pixel angle the camera reads for a
    # target that sits exactly on the head's reference axis.
    radar_delta_deg: tuple[float, float]
    horn_delta_deg: tuple[float, float]
    radar_drift_deg_per_s: tuple[float, float]
    horn_drift_deg_per_s: tuple[float, float]


def draw_scenario(
    cfg: Config, rng: np.random.Generator, distance_m: float | None = None
) -> tuple[Scenario, Calibration]:
    """Draw one random setup and the calibration the system believes in.

    The number and order of random draws never depends on the config values,
    so the same seed gives the same setup when single error sources are
    switched off for the sensitivity analysis.
    """
    geo, cal = cfg.geometry, cfg.calibration
    pointing = rng.uniform(-1.0, 1.0, 4) * geo.initial_pointing_error_deg
    rolls = rng.uniform(-1.0, 1.0, 2) * geo.initial_roll_deg
    nominal = rng.uniform(-1.0, 1.0, 4) * cal.mount_misalignment_deg
    cal_error = rng.standard_normal(4) * cal.residual_sigma_deg
    fixture_error = rng.standard_normal(2) * cal.fixture_repeat_sigma_deg
    drift = rng.standard_normal(4) * cfg.disturbances.drift_rate_sigma_deg_per_min / 60.0

    scenario = Scenario(
        distance_m=geo.distance_m if distance_m is None else distance_m,
        radar_az_deg=pointing[0],
        radar_el_deg=pointing[1],
        radar_roll_deg=rolls[0],
        horn_az_deg=pointing[2],
        horn_el_deg=pointing[3],
        horn_roll_deg=rolls[1],
        # The radar head is re-clamped to the datum on every setup.
        radar_delta_deg=(nominal[0] + fixture_error[0], nominal[1] + fixture_error[1]),
        horn_delta_deg=(nominal[2], nominal[3]),
        radar_drift_deg_per_s=(drift[0], drift[1]),
        horn_drift_deg_per_s=(drift[2], drift[3]),
    )
    calibration = Calibration(
        radar=HeadCalibration(
            delta_az_deg=nominal[0] + cal_error[0],
            delta_el_deg=nominal[1] + cal_error[1],
            u_delta_deg=math.hypot(cal.residual_sigma_deg, cal.fixture_repeat_sigma_deg),
            camera_offset_m=cfg.radar_head.camera_offset_m,
            beacon_offset_m=cfg.radar_head.beacon_offset_m,
        ),
        horn=HeadCalibration(
            delta_az_deg=nominal[2] + cal_error[2],
            delta_el_deg=nominal[3] + cal_error[3],
            u_delta_deg=cal.residual_sigma_deg,
            camera_offset_m=cfg.horn_head.camera_offset_m,
            beacon_offset_m=cfg.horn_head.beacon_offset_m,
        ),
    )
    return scenario, calibration


def _xz(offset: tuple[float, float]) -> np.ndarray:
    return np.array([offset[0], 0.0, offset[1]])


class World:
    """Radar head at the origin, horn head `distance_m` in front of it."""

    def __init__(self, cfg: Config, scenario: Scenario, actuators, rng_radar, rng_horn):
        self.cfg = cfg
        self.scenario = scenario
        self.actuators = actuators  # needs advance(dt) and mechanical(axis)
        self.t = 0.0
        dist = cfg.disturbances
        self._wind = {
            "radar": WindModel(dist.wind_jitter_deg, dist.wind_sway_deg, dist.wind_sway_tau_s, rng_radar),
            "horn": WindModel(dist.wind_jitter_deg, dist.wind_sway_deg, dist.wind_sway_tau_s, rng_horn),
        }
        self._lever = np.array(cfg.geometry.horn_pivot_to_center_m)
        self._offsets = {
            "radar": (_xz(cfg.radar_head.camera_offset_m), _xz(cfg.radar_head.beacon_offset_m)),
            "horn": (_xz(cfg.horn_head.camera_offset_m), _xz(cfg.horn_head.beacon_offset_m)),
        }
        self._delta = {
            "radar": g.mount_rotation(*np.radians(scenario.radar_delta_deg)),
            "horn": g.mount_rotation(*np.radians(scenario.horn_delta_deg)),
        }
        self._update()

    def advance(self, dt_s: float) -> None:
        self.t += dt_s
        self.actuators.advance(dt_s)
        for wind in self._wind.values():
            wind.step(dt_s)
        self._update()

    def _update(self) -> None:
        sc, act = self.scenario, self.actuators
        wr, wh = self._wind["radar"].value, self._wind["horn"].value

        radar_az = sc.radar_az_deg + wr[0] + sc.radar_drift_deg_per_s[0] * self.t
        radar_el = sc.radar_el_deg + wr[1] + sc.radar_drift_deg_per_s[1] * self.t
        r_radar = g.body_rotation(
            math.radians(radar_az), math.radians(radar_el), math.radians(sc.radar_roll_deg)
        )

        horn_az = sc.horn_az_deg + act.mechanical("pan") + wh[0] + sc.horn_drift_deg_per_s[0] * self.t
        horn_el = sc.horn_el_deg + act.mechanical("tilt") + wh[1] + sc.horn_drift_deg_per_s[1] * self.t
        r_horn = g.FACE_BACK @ g.body_rotation(
            math.radians(horn_az), math.radians(horn_el), math.radians(sc.horn_roll_deg)
        )
        # The X slide moves the horn to ITS right, which is world -x.
        pivot = np.array(
            [-act.mechanical("x") / 1000.0, sc.distance_m, act.mechanical("z") / 1000.0]
        )

        self._rot = {"radar": r_radar, "horn": r_horn}
        self._ref = {"radar": np.zeros(3), "horn": pivot + r_horn @ self._lever}

    def reference_point(self, side: str) -> np.ndarray:
        """Radar reference point, or horn aperture center."""
        return self._ref[side]

    def true_errors_deg(self) -> dict[str, float]:
        """Radar error and horn error, as defined in CLAUDE.md."""
        out = {}
        for side, other in (("radar", "horn"), ("horn", "radar")):
            v = self._rot[side].T @ (self._ref[other] - self._ref[side])
            az, el = g.tangent_angles_rad(v)
            out[f"{side}_az"] = math.degrees(az)
            out[f"{side}_el"] = math.degrees(el)
        return out

    def true_distance_m(self) -> float:
        return float(np.linalg.norm(self._ref["horn"] - self._ref["radar"]))

    def camera_view_rad(self, side: str) -> tuple[float, float, bool]:
        """Angles of the other head's beacon in this head's camera frame."""
        other = "horn" if side == "radar" else "radar"
        camera_pos = self._ref[side] + self._rot[side] @ self._offsets[side][0]
        beacon_pos = self._ref[other] + self._rot[other] @ self._offsets[other][1]
        v = (self._rot[side] @ self._delta[side]).T @ (beacon_pos - camera_pos)
        az, el = g.tangent_angles_rad(v)
        return az, el, bool(v[1] > 0)

    def true_tilt_deg(self, side: str) -> tuple[float, float]:
        """(pitch, roll) of a head relative to gravity."""
        r = self._rot[side]
        pitch = math.asin(max(-1.0, min(1.0, r[2, 1])))
        roll = math.atan2(-r[2, 0], r[2, 2])
        return math.degrees(pitch), math.degrees(roll)
