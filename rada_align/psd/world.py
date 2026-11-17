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
from .config import WEATHERS, PsdConfig
from .model import DEG, PSDS, SERVO_NAMES, Biases, Geometry, unit_sigmas_deg


class Servo:
    """Position servo with an output-side optical encoder, rate limit, travel limit
    and brake. Non-ideal motion (config psd.servos, zero = ideal):
      min_step_deg  minimum incremental motion: the stage moves in steps of this size
      repeat_deg    uni-directional repeatability, peak to peak (uniform settle error)
      reversal_deg  lost motion on a reversal of direction (output falls short by
                    up to this much; the output encoder sees it, so the loops can
                    correct it with a further step)."""

    def __init__(self, name: str, cfg: PsdConfig, rng: np.random.Generator | None = None):
        sc = cfg.servos
        self.name = name
        self.travel = sc.travel_deg
        self.speed = sc.max_speed_deg_s
        self.enc = sc.encoder_deg
        self.mim, self.repeat, self.reversal = sc.min_step_deg, sc.repeat_deg, sc.reversal_deg
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.pos = 0.0  # deg, mechanism
        self._target = 0.0  # commanded
        self.goal = 0.0  # where the mechanism will actually stop
        self._dir = 0
        self.braked = False

    @property
    def target(self) -> float:
        return self._target

    @target.setter
    def target(self, value: float) -> None:  # "stop here": no new motion, no settle error
        self._target = self.goal = float(value)

    def command(self, target_deg: float) -> bool:
        if self.braked:
            return False
        self._target = float(np.clip(target_deg, -self.travel, self.travel))
        goal = self._target
        if self.mim > 0:  # the stage can only land on its own step grid
            goal = self.pos + round((goal - self.pos) / self.mim) * self.mim
        if abs(goal - self.pos) < 1e-12:
            self.goal = self.pos
            return True
        d = 1 if goal > self.pos else -1
        if self.repeat > 0:
            goal += self.rng.uniform(-0.5, 0.5) * self.repeat
        if self.reversal > 0 and self._dir and d != self._dir:
            goal -= d * self.rng.uniform(0.0, self.reversal)
        self._dir = d
        self.goal = float(np.clip(goal, -self.travel, self.travel))
        return True

    def step(self, dt: float) -> None:
        if self.braked:
            return
        delta = self.goal - self.pos
        lim = self.speed * dt
        self.pos += float(np.clip(delta, -lim, lim))

    @property
    def encoder(self) -> float:
        return round(self.pos / self.enc) * self.enc if self.enc > 0 else self.pos

    @property
    def moving(self) -> bool:
        return abs(self.goal - self.pos) > 1e-9

    @property
    def at_limit(self) -> bool:
        return abs(self.target) >= self.travel - 1e-9


@dataclass
class PsdSample:
    x_mm: float
    y_mm: float
    valid: bool  # whole spot on the active area (linear reading)
    detect: bool  # any light on the active area
    power: float  # lock-in amplitude: fraction of the launched beam reaching the active area, 0..1
    dip: bool = False  # amplitude below the gate vs the strongest PSD (drop on this window)


def _fraction_1d(c: float, half: float, sigma: float) -> float:
    if sigma <= 0:
        return 1.0 if abs(c) <= half else 0.0
    k = 1.0 / (sigma * math.sqrt(2))
    return 0.5 * (math.erf((half - c) * k) + math.erf((half + c) * k))


class World:
    def __init__(self, cfg: PsdConfig, seed: int = 0, distance_m: float = 12.0,
                 calibration_loaded: bool = True, height_m: float | None = None):
        self.cfg = cfg
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        if height_m is None:  # scenario default: the radar stands higher than the horn
            g = cfg.geometry
            height_m = g.radar_height_short_m if distance_m <= 3.0 else g.radar_height_m
        self.geo = Geometry(cfg, distance_m, height_m)
        self.t = 0.0
        self.calibration_loaded = calibration_loaded
        srng = np.random.default_rng(seed + 20011)  # own stream: ideal servos leave the rest unchanged
        self.servos = [Servo(n, cfg, srng) for n in SERVO_NAMES]
        self.ac_size_mm = cfg.sensor.ac_size_mm
        self.wind_scale = 1.0
        self.blocked_until = -1.0
        # weather has its own generator and draws nothing when clear, so a clear
        # run and a rainy run of the same seed share setup, wind and sensor noise
        self.weather = "clear"
        self.wrng = np.random.default_rng(seed + 10_007)
        self._drops = {k: None for k in PSDS}  # (until_t, scattered fraction, bias mm)
        self._shimmer = None
        self._setup()

    # --------------------------------------------------------------- weather
    def set_weather(self, name: str) -> None:
        if name not in WEATHERS:
            raise ValueError(f"weather must be one of {WEATHERS}, got {name!r}")
        self.weather = name
        self._drops = {k: None for k in PSDS}
        self._shimmer = None
        if name == "shimmer":
            wc, L = self.cfg.weather, self.geo.d
            w0 = self.cfg.laser.launch_radius_mm * 1e-3
            ap = self.cfg.sensor.b_aperture_mm * 1e-3
            wander_mm = math.sqrt(2.42 * wc.shimmer_cn2 * L ** 3 * w0 ** (-1 / 3)) * 1e3
            aoa_rad = math.sqrt(2.91 * wc.shimmer_cn2 * L * ap ** (-1 / 3))
            sig = {"A": wander_mm, "C": wander_mm, "B": aoa_rad * self.cfg.sensor.b_focal_mm}
            self._shimmer = {k: [OUProcess(sig[k], wc.shimmer_tau_s, self.wrng) for _ in range(2)] for k in PSDS}
            self.shimmer_sigma_mm = sig

    @property
    def rain_mm_h(self) -> float:
        wc = self.cfg.weather
        return {"rain_light": wc.rain_light_mm_h, "rain_heavy": wc.rain_heavy_mm_h}.get(self.weather, 0.0)

    @property
    def transmission(self) -> float:
        """Atmospheric power transmission of the path (same for the three beams)."""
        wc, L = self.cfg.weather, self.geo.d
        if self.rain_mm_h > 0:
            db = wc.rain_ext_db_km * self.rain_mm_h ** wc.rain_ext_exp * L / 1000.0
            return 10 ** (-db / 10)
        if self.weather == "fog":
            return math.exp(-3.91 / wc.fog_visibility_m * L)
        return 1.0

    def _step_weather(self, dt: float) -> None:
        if self.weather == "clear":
            return
        wc = self.cfg.weather
        rate = wc.rain_hit_rate_per_mm_h * self.rain_mm_h
        if rate > 0:
            for k in PSDS:
                d = self._drops[k]
                if d is not None and self.t >= d[0]:
                    self._drops[k] = d = None
                if d is None and self.wrng.random() < rate * dt:
                    s = self.wrng.uniform(wc.rain_scatter_min, wc.rain_scatter_max)
                    per = wc.rain_bias_b_mm if k == "B" else wc.rain_bias_ac_mm
                    self._drops[k] = (self.t + self.wrng.exponential(wc.rain_drop_stay_s), s,
                                      self.wrng.normal(0, per * s, 2))
        if self._shimmer is not None:
            self._shimmer_now = {k: np.array([p.step(dt) for p in ps]) for k, ps in self._shimmer.items()}

    # --------------------------------------------------------------- setup
    def _setup(self) -> None:
        g, rng = self.cfg.geometry, self.rng
        p, r = g.setup_pointing_deg, g.setup_roll_deg
        # manual setup: each side is aimed by hand at the other one (so along the
        # sloped line of sight when the heights differ), within +/- p
        self.base = np.degrees(self.geo.att0) + np.array(
            [rng.uniform(-p, p), rng.uniform(-p, p), rng.uniform(-r, r),
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
        self._step_weather(dt)

    # --------------------------------------------------------------- sensors
    @property
    def blocked(self) -> bool:
        return self.t < self.blocked_until

    def read_psds(self) -> dict[str, PsdSample]:
        beams = self.geo.beams(self.attitude(), self.trans, self.bias)
        noise = self.cfg.sensor.noise_um * 1e-3
        r99 = self.geo.spot_r99_mm
        sig = r99 / 3.035  # 1/e^2 radius / 2 = Gaussian sigma (r99 = 1.517 w, sigma = w/2)
        T = self.transmission
        sun = self.cfg.weather.sun_noise_factor if self.weather == "sun" else 1.0
        shim = getattr(self, "_shimmer_now", None) if self._shimmer is not None else None
        raw = {}
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
            drop = self._drops[k]
            atm = T  # share of the launched power that survives the path and the window
            if shim is not None:
                pos = pos + shim[k]
            if drop is not None:  # a drop on this window: scatters light, shifts the centroid
                atm *= 1.0 - drop[1]
                pos = pos + drop[2]
            power *= atm
            detect = power > 0.02
            if detect and not valid and k != "B":
                # part of the spot is off the active area: the PSD reports the
                # centroid of the part it sees, pulled towards the inside.
                lim = max(half - 0.5 * r99, 0.1)
                pos = np.clip(pos, -lim, lim)
            # position noise of a PSD goes as 1 / photocurrent; the sensor noise
            # draw stays on the main generator and is only scaled here
            scale = sun / max(atm, 0.05)
            meas = pos + self.rng.normal(0, noise, 2) * scale if noise > 0 else pos.copy()
            raw[k] = (meas, bool(valid and detect), bool(detect), float(power))
        top = max(v[3] for v in raw.values())
        gate = self.cfg.sensor.amplitude_gate
        out = {}
        for k, (meas, valid, detect, power) in raw.items():
            # amplitude gate against the strongest PSD: only meaningful for a spot
            # that is whole on its PSD (a partial spot is already not valid)
            dip = bool(valid and power < gate * top)
            out[k] = PsdSample(float(meas[0]), float(meas[1]), valid and not dip, detect, power, dip)
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
