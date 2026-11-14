"""Simulated Camera, Inclinometer, Rangefinder and Actuators.

Each one reads the truth from the World and adds its own error model.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np

from . import geometry as g
from .geometry import intrinsics_from_config
from .config import ActuatorsCfg, Calibration, CameraCfg, Config, InclinometerCfg, RangefinderCfg
from .disturbances import Backlash, OUProcess, quantize
from .interfaces import (
    AXES,
    ActuatorError,
    Actuators,
    BeaconMeasurement,
    Camera,
    Hardware,
    Inclinometer,
    Rangefinder,
    TiltReading,
)
from .world import Scenario, World, draw_scenario


class SimCamera(Camera):
    """Pinhole camera: true beacon direction + turbulence + centroid noise."""

    def __init__(self, world: World, side: str, cam: CameraCfg, rng: np.random.Generator):
        self._world = world
        self._side = side
        self._cam = cam
        self._rng = rng
        self._intr = intrinsics_from_config(cam)
        turbulence_rad = math.radians(cam.turbulence_deg)
        self._turbulence = [OUProcess(turbulence_rad, cam.turbulence_tau_s, rng) for _ in range(2)]
        self._last_t = world.t

    def measure_beacon(self) -> BeaconMeasurement:
        dt = self._world.t - self._last_t
        self._last_t = self._world.t
        turb = [p.step(dt) for p in self._turbulence]
        noise = self._rng.standard_normal(2) * self._cam.centroid_noise_px

        az, el, in_front = self._world.camera_view_rad(self._side)
        u, v = g.angles_to_pixel(az + turb[0], el + turb[1], self._intr)
        u, v = u + noise[0], v + noise[1]
        valid = in_front and 0.0 <= u < self._intr.width_px and 0.0 <= v < self._intr.height_px
        return BeaconMeasurement(u, v, self._cam.centroid_noise_px, valid, self._world.t)


class SimInclinometer(Inclinometer):
    def __init__(self, world: World, side: str, cfg: InclinometerCfg, rng: np.random.Generator):
        self._world = world
        self._side = side
        self._cfg = cfg
        self._rng = rng
        self._bias = rng.standard_normal(2) * cfg.bias_sigma_deg

    def read_tilt(self) -> TiltReading:
        pitch, roll = self._world.true_tilt_deg(self._side)
        noise = self._rng.standard_normal(2) * self._cfg.noise_deg
        return TiltReading(
            pitch + self._bias[0] + noise[0], roll + self._bias[1] + noise[1], self._world.t
        )


class SimRangefinder(Rangefinder):
    def __init__(self, world: World, cfg: RangefinderCfg, rng: np.random.Generator):
        self._world = world
        self._cfg = cfg
        self._rng = rng
        self._bias = rng.standard_normal() * cfg.bias_sigma_m

    def read_distance_m(self) -> float:
        noise = self._rng.standard_normal() * self._cfg.noise_m
        return self._world.true_distance_m() + self._bias + noise


class SimActuators(Actuators):
    """Four stepper axes with step quantization, finite speed and backlash.

    `position()` is what a real driver knows (the motor position). The world
    uses `mechanical()`, which lags by the backlash.
    """

    def __init__(self, cfg: ActuatorsCfg, rng: np.random.Generator):
        self._cfg = {axis: getattr(cfg, axis) for axis in AXES}
        self._motor = {axis: 0.0 for axis in AXES}
        self._target = {axis: 0.0 for axis in AXES}
        play = rng.uniform(-0.5, 0.5, len(AXES))
        self._backlash = {
            axis: Backlash(self._cfg[axis].backlash, play[i] * self._cfg[axis].backlash)
            for i, axis in enumerate(AXES)
        }

    def _check(self, axis: str) -> None:
        if axis not in self._cfg:
            raise ActuatorError(f"unknown axis '{axis}'")

    def move_relative(self, axis: str, delta: float) -> None:
        self._check(axis)
        cfg = self._cfg[axis]
        target = quantize(self._target[axis] + delta, cfg.step)
        if not cfg.min <= target <= cfg.max:
            raise ActuatorError(
                f"{axis} soft limit: target {target:.3f} outside [{cfg.min}, {cfg.max}]"
            )
        self._target[axis] = target

    def position(self, axis: str) -> float:
        self._check(axis)
        return self._motor[axis]

    def limits(self, axis: str) -> tuple[float, float]:
        self._check(axis)
        return self._cfg[axis].min, self._cfg[axis].max

    def is_moving(self) -> bool:
        return any(self._target[a] != self._motor[a] for a in AXES)

    def home(self, axis: str) -> None:
        self._check(axis)
        self._target[axis] = 0.0

    def stop(self) -> None:
        self._target = dict(self._motor)

    # --- simulation side -------------------------------------------------
    def advance(self, dt_s: float) -> None:
        for axis in AXES:
            remaining = self._target[axis] - self._motor[axis]
            max_step = self._cfg[axis].max_speed * dt_s
            if abs(remaining) <= max_step:
                self._motor[axis] = self._target[axis]
            else:
                self._motor[axis] += math.copysign(max_step, remaining)
            self._backlash[axis].update(self._motor[axis])

    def mechanical(self, axis: str) -> float:
        return self._backlash[axis].mechanism


@dataclass
class Simulation:
    world: World
    hardware: Hardware
    calibration: Calibration
    scenario: Scenario


def build_simulation(
    cfg: Config,
    seed,
    distance_m: float | None = None,
    calibration: Calibration | None = None,
    **scenario_overrides,
) -> Simulation:
    """Build one simulated setup.

    `scenario_overrides` pin Scenario fields. Pinning a true camera delta
    also pins the calibrated delta to the same value (perfect calibration),
    unless an explicit `calibration` is given.
    """
    streams = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(9)]
    scenario, drawn_calibration = draw_scenario(cfg, streams[0], distance_m)
    if scenario_overrides:
        scenario = dataclasses.replace(scenario, **scenario_overrides)
    if calibration is None:
        calibration = drawn_calibration
        for side in ("radar", "horn"):
            if f"{side}_delta_deg" in scenario_overrides:
                az, el = scenario_overrides[f"{side}_delta_deg"]
                head = dataclasses.replace(getattr(calibration, side), delta_az_deg=az, delta_el_deg=el)
                calibration = dataclasses.replace(calibration, **{side: head})

    actuators = SimActuators(cfg.actuators, streams[1])
    world = World(cfg, scenario, actuators, streams[2], streams[3])
    hardware = Hardware(
        radar_camera=SimCamera(world, "radar", cfg.camera, streams[4]),
        horn_camera=SimCamera(world, "horn", cfg.camera, streams[5]),
        radar_inclinometer=SimInclinometer(world, "radar", cfg.inclinometer, streams[6]),
        horn_inclinometer=SimInclinometer(world, "horn", cfg.inclinometer, streams[7]),
        rangefinder=SimRangefinder(world, cfg.rangefinder, streams[8]),
        actuators=actuators,
    )
    return Simulation(world, hardware, calibration, scenario)
