"""Load and validate config.yaml into typed, immutable dataclasses.

Loading is strict: a missing or unknown key is an error, so a typo can never
silently fall back to a default.
"""

from __future__ import annotations

import dataclasses
import typing
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config.yaml"


@dataclass(frozen=True)
class GeometryCfg:
    distance_m: float
    tolerance_deg: float
    initial_pointing_error_deg: float
    initial_roll_deg: float
    horn_pivot_to_center_m: tuple[float, float, float]


@dataclass(frozen=True)
class HeadCfg:
    camera_offset_m: tuple[float, float]
    beacon_offset_m: tuple[float, float]


@dataclass(frozen=True)
class CameraCfg:
    focal_length_mm: float
    pixel_size_um: float
    width_px: int
    height_px: int
    rate_hz: float
    centroid_noise_px: float
    turbulence_deg: float
    turbulence_tau_s: float


@dataclass(frozen=True)
class CalibrationCfg:
    mount_misalignment_deg: float
    residual_sigma_deg: float
    fixture_repeat_sigma_deg: float


@dataclass(frozen=True)
class InclinometerCfg:
    noise_deg: float
    bias_sigma_deg: float


@dataclass(frozen=True)
class RangefinderCfg:
    bias_sigma_m: float
    noise_m: float


@dataclass(frozen=True)
class AxisCfg:
    step: float
    backlash: float
    max_speed: float
    min: float
    max: float


@dataclass(frozen=True)
class ActuatorsCfg:
    x: AxisCfg
    z: AxisCfg
    pan: AxisCfg
    tilt: AxisCfg


@dataclass(frozen=True)
class DisturbancesCfg:
    wind_jitter_deg: float
    wind_sway_deg: float
    wind_sway_tau_s: float
    drift_rate_sigma_deg_per_min: float


@dataclass(frozen=True)
class EstimatorCfg:
    align_window_s: float
    verify_window_s: float
    settle_time_s: float


@dataclass(frozen=True)
class ControllerCfg:
    deadband_deg: float
    gain: float
    max_iterations: int
    max_verify_attempts: int
    track_drift_threshold_deg: float
    acquire_frames: int
    acquire_timeout_s: float
    lost_frames: int
    unstable_windows: int
    search_step_deg: float
    search_max_deg: float
    search_dwell_s: float


@dataclass(frozen=True)
class MonteCarloCfg:
    runs: int
    seed: int
    max_time_s: float
    track_time_s: float
    sensitivity_runs: int


@dataclass(frozen=True)
class Config:
    mode: str
    geometry: GeometryCfg
    radar_head: HeadCfg
    horn_head: HeadCfg
    camera: CameraCfg
    calibration: CalibrationCfg
    inclinometer: InclinometerCfg
    rangefinder: RangefinderCfg
    actuators: ActuatorsCfg
    disturbances: DisturbancesCfg
    estimator: EstimatorCfg
    controller: ControllerCfg
    montecarlo: MonteCarloCfg

    def replace_path(self, path: str, value) -> "Config":
        """Return a copy with one nested field changed, e.g. 'camera.rate_hz'."""
        return _replace_path(self, path.split("."), value)


@dataclass(frozen=True)
class HeadCalibration:
    """What the system believes about one head (from the calibration file)."""

    delta_az_deg: float  # camera boresight minus the head's reference axis
    delta_el_deg: float
    camera_offset_m: tuple[float, float]  # [right, up] in the head's body frame
    beacon_offset_m: tuple[float, float]


@dataclass(frozen=True)
class Calibration:
    radar: HeadCalibration
    horn: HeadCalibration


def _replace_path(obj, parts: list[str], value):
    name = parts[0]
    if not dataclasses.is_dataclass(obj) or name not in {f.name for f in dataclasses.fields(obj)}:
        raise KeyError(f"no config field '{name}'")
    if len(parts) == 1:
        return dataclasses.replace(obj, **{name: value})
    return dataclasses.replace(obj, **{name: _replace_path(getattr(obj, name), parts[1:], value)})


def _convert(tp, value, path: str):
    if dataclasses.is_dataclass(tp):
        return _from_dict(tp, value, path)
    if typing.get_origin(tp) is tuple:
        args = typing.get_args(tp)
        if not isinstance(value, (list, tuple)) or len(value) != len(args):
            raise ValueError(f"config '{path}': expected a list of {len(args)} numbers")
        return tuple(_convert(a, v, path) for a, v in zip(args, value))
    if tp is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"config '{path}': expected a number, got {value!r}")
        return float(value)
    if tp is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"config '{path}': expected an integer, got {value!r}")
        return value
    if tp is str:
        if not isinstance(value, str):
            raise ValueError(f"config '{path}': expected a string, got {value!r}")
        return value
    raise TypeError(f"config '{path}': unsupported field type {tp!r}")


def _from_dict(cls, data, path: str):
    if not isinstance(data, dict):
        raise ValueError(f"config '{path or 'root'}': expected a mapping")
    hints = typing.get_type_hints(cls)
    names = [f.name for f in dataclasses.fields(cls)]
    missing = [n for n in names if n not in data]
    unknown = [k for k in data if k not in names]
    if missing or unknown:
        raise ValueError(
            f"config '{path or 'root'}': missing keys {missing}, unknown keys {unknown}"
        )
    return cls(**{n: _convert(hints[n], data[n], f"{path}.{n}" if path else n) for n in names})


def config_from_dict(data: dict) -> Config:
    cfg = _from_dict(Config, data, "")
    if cfg.mode not in ("sim", "hw"):
        raise ValueError(f"config 'mode': must be 'sim' or 'hw', got {cfg.mode!r}")
    return cfg


def load_config(path: str | Path | None = None) -> Config:
    path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    return config_from_dict(yaml.safe_load(path.read_text(encoding="utf-8")))
