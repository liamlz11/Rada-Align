"""Simulated rig of the final design: truth, servos, disturbances and sensors.

Everything the controller may read goes through the sensor methods
(`read_psds`, `read_inclinometers`, `read_range`, `camera_acquire`, servo
encoders). Truth (`true_errors`, biases, base offsets) is for scoring and the
"truth (simulation only)" panel of the demo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..disturbances import OUProcess
from .config import PsdConfig
from .model import DEG, PSDS, SERVO_NAMES, Biases, Geometry, unit_sigmas_deg


class Servo:
    """Position servo with an optical encoder, rate limit, travel limit and brake."""

    def __init__(self, name: str, cfg: PsdConfig):
        self.name = name
        self.travel = cfg.servos.travel_deg
        self.speed = cfg.servos.max_speed_deg_s
        self.enc = cfg.servos.encoder_deg
        self.pos = 0.0  # deg, mechanism
        self.target = 0.0
        self.braked = False

    def command(self, target_deg: float) -> bool:
        if self.braked:
            return False
        self.target = float(np.clip(target_deg, -self.travel, self.travel))
        return True

    def step(self, dt: float) -> None:
        if self.braked:
            return
        delta = self.target - self.pos
        lim = self.speed * dt
        self.pos += float(np.clip(delta, -lim, lim))

    @property
    def encoder(self) -> float:
        return round(self.pos / self.enc) * self.enc if self.enc > 0 else self.pos

    @property
    def moving(self) -> bool:
        return abs(self.target - self.pos) > 1e-9

    @property
    def at_limit(self) -> bool:
        return abs(self.target) >= self.travel - 1e-9


@dataclass
class PsdSample:
    x_mm: float
    y_mm: float
    valid: bool  # whole spot on the active area (linear reading)
    detect: bool  # any light on the active area
    power: float  # fraction of the beam on the active area, 0..1


def _fraction_1d(c: float, half: float, sigma: float) -> float:
    if sigma <= 0:
        return 1.0 if abs(c) <= half else 0.0
    k = 1.0 / (sigma * math.sqrt(2))
    return 0.5 * (math.erf((half - c) * k) + math.erf((half + c) * k))


class World:
    def __init__(self, cfg: PsdConfig, seed: int = 0, distance_m: float = 12.0,
                 calibration_loaded: bool = True):
        self.cfg = cfg
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.geo = Geometry(cfg, distance_m)
        self.t = 0.0
        self.calibration_loaded = calibration_loaded
        self.servos = [Servo(n, cfg) for n in SERVO_NAMES]
        self.ac_size_mm = cfg.sensor.ac_size_mm
        self.wind_scale = 1.0
        self.blocked_until = -1.0
        self._setup()

    # --------------------------------------------------------------- setup
    def _setup(self) -> None:
        g, rng = self.cfg.geometry, self.rng
        p, r = g.setup_pointing_deg, g.setup_roll_deg
        self.base = np.array([rng.uniform(-p, p), rng.uniform(-p, p), rng.uniform(-r, r),
                              rng.uniform(-p, p), rng.uniform(-p, p), rng.uniform(-r, r)])
        off = g.setup_offset_short_m if self.geo.d <= 3.0 else g.setup_offset_m
        self.trans = np.array([rng.uniform(-off, off), rng.uniform(-off, off), 0.0, 0.0])
        lz = self.cfg.laser
        c = self.cfg.calib
        emitter_sigma, receiver_sigma = unit_sigmas_deg(self.cfg, self.geo.emitter)
        common = rng.normal(0, emitter_sigma, 2) * DEG  # laser board vs its side's axis
        self.bias = Biases(
            laser_dir={k: tuple(common + rng.normal(0, lz.differential_sigma_deg, 2) * DEG) for k in PSDS},
            b_null=tuple(rng.normal(0, receiver_sigma, 2) * DEG),  # PSD-B null vs its side's axis
            psd_pos_mm={k: tuple(rng.normal(0, c.psd_position_sigma_mm, 2)) for k in PSDS},
        )
        w = self.cfg.wind
        self.jitter = [OUProcess(w.jitter_deg, w.jitter_tau_s, rng) for _ in range(6)]
        self.sway = [OUProcess(w.sway_deg, w.sway_tau_s, rng) for _ in range(6)]
        self.drift_rate = rng.normal(0, w.drift_deg_per_min / 60.0, 6)
        self.drift_rate[[2, 5]] = 0.0
        self.drift = np.zeros(6)
        self.push = np.zeros(6)
        inc = self.cfg.inclinometer
        self.incl_bias = rng.normal(0, inc.bias_sigma_deg, 4)
        self.range_bias = rng.normal(0, self.cfg.rangefinder.sigma_m)
        self.camera_bias = rng.normal(0, self.cfg.camera.acquire_sigma_deg, 2)
        self._wind = np.zeros(6)

    # --------------------------------------------------------------- truth
    def attitude_deg(self) -> np.ndarray:
        q = np.array([s.pos for s in self.servos])
        return self.base + q + self._wind + self.drift + self.push

    def attitude(self) -> np.ndarray:
        return self.attitude_deg() * DEG

    def true_errors(self) -> np.ndarray:
        return self.geo.errors(self.attitude(), self.trans)

    def true_errors_vs_datum(self) -> np.ndarray:
        """Same as true_errors: the radar error is defined against its datum."""
        return self.true_errors()

    # --------------------------------------------------------------- time
    def step(self, dt: float) -> None:
        self.t += dt
        for s in self.servos:
            s.step(dt)
        ws = self.wind_scale
        self._wind = np.array([(j.step(dt) + sw.step(dt)) * ws for j, sw in zip(self.jitter, self.sway)])
        self._wind[[2, 5]] *= 0.2  # roll is much stiffer than pan/tilt (ASSUMPTION)
        self.drift += self.drift_rate * dt

    # --------------------------------------------------------------- sensors
    @property
    def blocked(self) -> bool:
        return self.t < self.blocked_until

    def read_psds(self) -> dict[str, PsdSample]:
        beams = self.geo.beams(self.attitude(), self.trans, self.bias)
        noise = self.cfg.sensor.noise_um * 1e-3
        r99 = self.geo.spot_r99_mm
        sig = r99 / 3.035  # 1/e^2 radius / 2 = Gaussian sigma (r99 = 1.517 w, sigma = w/2)
        out = {}
        for k in PSDS:
            if k == "B":
                lens = beams["B_lens"]
                ap = self.cfg.sensor.b_aperture_mm / 2
                frac_lens = 1.0 if np.all(np.abs(lens) + r99 <= ap) else (
                    _fraction_1d(lens[0], ap, sig) * _fraction_1d(lens[1], ap, sig))
                pos = beams["B"]
                half = self.cfg.sensor.b_size_mm / 2
                focal_r = 0.05  # mm, focused spot in the focal plane (FINAL_DESIGN 7.1, ~f*divergence)
                inside = bool(np.all(np.abs(pos) + focal_r <= half))
                power = frac_lens * (1.0 if inside else 0.0)
                valid = inside and np.all(np.abs(lens) + r99 <= ap)
            else:
                pos = beams[k]
                half = self.ac_size_mm / 2
                power = _fraction_1d(pos[0], half, sig) * _fraction_1d(pos[1], half, sig)
                valid = bool(np.all(np.abs(pos) + r99 <= half))
            if self.blocked:
                power, valid = 0.0, False
            detect = power > 0.02
            if detect and not valid and k != "B":
                # part of the spot is off the active area: the PSD reports the
                # centroid of the part it sees, pulled towards the inside.
                lim = max(half - 0.5 * r99, 0.1)
                pos = np.clip(pos, -lim, lim)
            meas = pos + self.rng.normal(0, noise, 2) if noise > 0 else pos.copy()
            out[k] = PsdSample(float(meas[0]), float(meas[1]), bool(valid and detect), bool(detect), float(power))
        return out

    def read_inclinometers(self) -> dict:
        gtruth = self.geo.gravity(self.attitude())
        n = self.cfg.inclinometer.noise_deg
        keys = ("horn_tilt", "horn_roll", "radar_tilt", "radar_roll")
        return {k: gtruth[k] + self.incl_bias[i] + (self.rng.normal(0, n) if n > 0 else 0.0)
                for i, k in enumerate(keys)}

    def read_range(self) -> float:
        return self.geo.d + self.range_bias

    def camera_acquire(self) -> tuple[float, float] | None:
        """Template match of the PSD unit -> pointing error of the EMITTER side
        (the side that carries the camera and the lasers), in degrees.

        Camera-to-laser-board boresight error is a fixed bias per setup."""
        e = self.true_errors()
        i = 2 if self.geo.emitter == "radar" else 0
        if self.blocked or max(abs(e[i]), abs(e[i + 1])) > self.cfg.camera.fov_half_deg:
            return None
        return float(e[i] + self.camera_bias[0]), float(e[i + 1] + self.camera_bias[1])

    # --------------------------------------------------------------- events
    def block_beam(self, seconds: float) -> None:
        self.blocked_until = self.t + seconds

    def push_side(self, side: str, deg: float) -> None:
        """A knock on one tripod: permanent pan/tilt offset (degrees)."""
        i = 0 if side == "horn" else 3
        a = self.rng.uniform(0, 2 * math.pi)
        self.push[i] += deg * math.cos(a)
        self.push[i + 1] += deg * math.sin(a)
