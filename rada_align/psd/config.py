"""Strict loader for the `psd:` section of config.yaml (final design)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from ..config import DEFAULT_CONFIG_PATH, _from_dict, _replace_path


@dataclass(frozen=True)
class GeometryCfg:
    emitter: str  # 'radar' (lasers + camera on the radar, PSDs on the horn) or 'horn'
    horn_lever_m: tuple[float, float, float]
    radar_lever_m: tuple[float, float, float]
    laser_a_m: tuple[float, float]
    laser_b_m: tuple[float, float]
    laser_c_m: tuple[float, float]
    setup_pointing_deg: float
    setup_roll_deg: float
    setup_offset_m: float
    setup_offset_short_m: float
    radar_height_m: float
    radar_height_short_m: float


@dataclass(frozen=True)
class SensorCfg:
    ac_size_mm: float
    b_size_mm: float
    b_focal_mm: float
    b_aperture_mm: float
    noise_um: float
    rate_hz: float


@dataclass(frozen=True)
class LaserCfg:
    wavelength_nm: float
    launch_radius_mm: float
    differential_sigma_deg: float


@dataclass(frozen=True)
class CalibCfg:
    radar_datum_sigma_deg: float
    radar_seat_sigma_deg: float
    horn_axis_sigma_deg: float
    psd_position_sigma_mm: float


@dataclass(frozen=True)
class ServosCfg:
    travel_deg: float
    max_speed_deg_s: float
    encoder_deg: float


@dataclass(frozen=True)
class WindCfg:
    jitter_deg: float
    jitter_tau_s: float
    sway_deg: float
    sway_tau_s: float
    drift_deg_per_min: float


@dataclass(frozen=True)
class InclinometerCfg:
    noise_deg: float
    bias_sigma_deg: float


@dataclass(frozen=True)
class CameraCfg:
    acquire_sigma_deg: float
    fov_half_deg: float


@dataclass(frozen=True)
class RangefinderCfg:
    sigma_m: float


@dataclass(frozen=True)
class ControlCfg:
    roll_from_inclinometers: bool
    deadband_deg: float
    kp: float
    ki: float
    track_kp: float
    settle_s: float
    align_window_s: float
    verify_window_s: float
    track_window_s: float
    track_threshold_deg: float
    radar_alert_deg: float
    max_iterations: int
    max_verify_attempts: int
    min_valid_fraction: float
    scan_sigmas: float
    beam_lost_s: float


@dataclass(frozen=True)
class PsdConfig:
    geometry: GeometryCfg
    sensor: SensorCfg
    laser: LaserCfg
    calib: CalibCfg
    servos: ServosCfg
    wind: WindCfg
    inclinometer: InclinometerCfg
    camera: CameraCfg
    rangefinder: RangefinderCfg
    control: ControlCfg

    def replace_path(self, path: str, value) -> "PsdConfig":
        """Copy with one nested field changed, e.g. 'wind.jitter_deg'."""
        return _replace_path(self, path.split("."), value)


TOLERANCE_DEG = 0.05  # REQUIREMENT R7 (geometry.tolerance_deg in the legacy section)

# Error sources and the fields that switch each off (zero-noise tests, studies).
ERROR_SOURCES: dict[str, tuple[str, ...]] = {
    "wind": ("wind.jitter_deg", "wind.sway_deg"),
    "drift": ("wind.drift_deg_per_min",),
    "laser": ("laser.differential_sigma_deg",),
    "calib": ("calib.radar_datum_sigma_deg", "calib.radar_seat_sigma_deg", "calib.horn_axis_sigma_deg",
              "calib.psd_position_sigma_mm"),
    "psd_noise": ("sensor.noise_um",),
    "inclinometer": ("inclinometer.noise_deg", "inclinometer.bias_sigma_deg"),
    "rangefinder": ("rangefinder.sigma_m",),
    "camera": ("camera.acquire_sigma_deg",),
}


def quiet(cfg: PsdConfig, keep: tuple[str, ...] = ()) -> PsdConfig:
    """Every error source switched off except `keep`."""
    for name, paths in ERROR_SOURCES.items():
        if name not in keep:
            for path in paths:
                cfg = cfg.replace_path(path, 0.0)
    return cfg


def load_psd_config(path: str | Path | None = None) -> PsdConfig:
    path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if "psd" not in data:
        raise ValueError("config: no 'psd' section (final design)")
    return _from_dict(PsdConfig, data["psd"], "psd")
