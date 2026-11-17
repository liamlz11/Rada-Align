"""Live final-design session behind the demo page (simulation)."""

from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np

from .config import DEMO_SOURCES, TOLERANCE_DEG, PsdConfig, quiet
from .controller import STATES, PsdController
from .model import ERROR_NAMES, PSDS, SERVO_NAMES
from .world import World


def _f(x, nd=5):
    return None if x is None else round(float(x), nd)


def _vec(v, nd=5):
    return None if v is None else [round(float(a), nd) for a in v]


def _score(s):
    if s is None:
        return None
    out = {"mean": _vec(s["mean"]), "sigma": _vec(s["sigma"]), "score": _vec(s["score"]), "pass": bool(s["pass"])}
    if "u_bias" in s:
        out.update({"u_bias": _vec(s["u_bias"]), "score_with_bias": _vec(s["score_with_bias"]),
                    "pass_with_bias": bool(s["pass_with_bias"])})
    return out


class PsdSession:
    PUBLISH_HZ = 20.0

    def __init__(self, cfg: PsdConfig, seed: int = 0, distance_m: float = 12.0):
        self.base_cfg = cfg
        self.seed = seed
        self.distance_m = distance_m
        self.speed = 1.0
        self.ac_size_mm = cfg.sensor.ac_size_mm
        self.wind_scale = 1.0
        self.field_errors = False  # False: demo profile (config.DEMO_SOURCES); True: every assumed source
        self.height_m: float | None = None  # None = scenario default from config
        self.weather = "clear"
        self.paused = False  # the guided tour freezes the simulation while a caption is read
        self.pause_on_state = False  # tour: freeze at every controller state change
        self._lock = threading.Lock()
        self._messages: deque[dict] = deque(maxlen=100)
        self._seq = 0
        self._thread: threading.Thread | None = None
        self._running = False
        self._build()

    def _build(self) -> None:
        cfg = self.base_cfg
        if not self.field_errors:  # demo profile: wind, drift, laser and calibration residuals off
            cfg = quiet(cfg, DEMO_SOURCES)
        self.world = World(cfg, self.seed, self.distance_m, height_m=self.height_m)
        self.world.ac_size_mm = self.ac_size_mm
        self.world.wind_scale = self.wind_scale
        self.world.set_weather(self.weather)
        self.ctrl = PsdController(self.world)
        self.dt = self.ctrl.dt
        self._events_sent = 0
        self._next_pub = 0.0

    # --------------------------------------------------------------- commands
    def command(self, name: str, value=None) -> dict:
        with self._lock:
            w, c = self.world, self.ctrl
            if name == "start":
                c.start()
                if self.pause_on_state:  # IDLE -> CALIB happens here, not in a tick
                    self.paused = True
                    self._publish()  # the stepping loop is frozen: publish CALIB now
            elif name == "stop":
                c.stop()
            elif name == "new_setup":
                self.seed += 1
                self._build()
            elif name == "distance":
                self.distance_m = float(np.clip(float(value), 1.0, 20.0))
                self._build()
            elif name == "height":
                self.height_m = float(np.clip(float(value), -5.0, 5.0))
                self._build()
            elif name == "weather":
                w.set_weather(str(value))
                self.weather = str(value)
                c._log(f"field condition: {value} (path transmission {w.transmission:.1%})")
            elif name == "psd_size":
                self.ac_size_mm = float(value)
                w.ac_size_mm = self.ac_size_mm
                c._log(f"PSD-A/C active size set to {self.ac_size_mm:.0f} mm")
            elif name == "wind":
                self.wind_scale = float(np.clip(float(value), 0.0, 5.0))
                w.wind_scale = self.wind_scale
            elif name == "push":
                side, _, size = str(value).partition("/")  # "horn", "radar" or "radar/0.02"
                deg = float(size) if size else 0.08
                w.push_side(side, deg)
                c._log(f"disturbance: {side} tripod knocked by {deg:.3f} deg")
            elif name == "block":
                w.block_beam(float(value or 0.6))
                c._log(f"disturbance: beam blocked for {float(value or 0.6):.1f} s")
            elif name == "gap":
                c.radar_gap = bool(value)
                c._log("radar correction gap " + ("ALLOWED" if c.radar_gap else "closed"))
            elif name == "field_errors":
                self.field_errors = bool(value)
                self._build()
            elif name == "encoder_test":
                az, el = (float(v) for v in str(value).split(","))
                if not c.start_encoder_test(float(np.clip(az, -2, 2)), float(np.clip(el, -2, 2))):
                    raise ValueError("encoder test needs TRACK")
            elif name == "pause":
                self.paused = bool(value)
            elif name == "pause_on_state":
                self.pause_on_state = bool(value)
            elif name == "speed":
                self.speed = float(np.clip(float(value), 0.25, 8.0))
            else:
                raise KeyError(name)
            return {"ok": True}

    # --------------------------------------------------------------- stepping
    def step_once(self) -> dict | None:
        with self._lock:
            before = self.ctrl.state
            self.ctrl.tick()
            self.world.step(self.dt)
            if self.pause_on_state and self.ctrl.state != before:
                self.paused = True
                self._next_pub = self.world.t  # publish the new state at once
            if self.world.t >= self._next_pub:
                self._next_pub = self.world.t + 1.0 / self.PUBLISH_HZ
                return self._publish()
        return None

    def run_in_background(self) -> None:
        self._running = True

        def loop():
            next_t = time.perf_counter()
            while self._running:
                if self.paused:
                    time.sleep(0.05)
                    next_t = time.perf_counter()
                    continue
                n = max(1, round(self.speed))
                frac = self.speed / n
                for _ in range(n):
                    self.step_once()
                next_t += self.dt / frac
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.perf_counter()

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def since(self, seq: int) -> list[dict]:
        with self._lock:
            return [m for m in self._messages if m["seq"] > seq]

    def latest(self) -> dict | None:
        with self._lock:
            return self._messages[-1] if self._messages else None

    @property
    def seq(self) -> int:
        return self._seq

    def _height_mm(self, inc: dict) -> float:
        """R3: radar reference above the horn centre, from the measured range and
        the two inclinometer tilts, each corrected by the PSD elevation error."""
        e = self.ctrl.est if self.ctrl.est is not None else np.zeros(5)
        elev = 0.5 * ((inc["horn_tilt"] + e[1]) + (-inc["radar_tilt"] - e[3]))
        geo = self.world.geo  # horn centre and radar reference sit ahead of their pivots
        base = self.world.read_range() - geo.horn_lever[0] - geo.radar_lever[0]
        return base * np.tan(np.radians(elev)) * 1e3

    # --------------------------------------------------------------- message
    def _publish(self) -> dict:
        w, c = self.world, self.ctrl
        psd = getattr(c, "last_psd", None) or w.read_psds()
        true = w.true_errors()
        geo = w.geo
        inc = w.read_inclinometers()
        tw = c.track_window
        self._seq += 1
        new_events = c.events[self._events_sent:]
        self._events_sent = len(c.events)
        msg = {
            "seq": self._seq,
            "sim": True,
            "t": _f(w.t, 2),
            "speed": self.speed,
            "paused": self.paused,
            "state": c.state,
            "sub": c.sub,
            "states": list(STATES),
            "fault_reason": c.fault_reason,
            "setup_id": self.seed,
            "distance_m": geo.d,
            "tolerance_deg": TOLERANCE_DEG,
            "aligned": c.aligned,
            "aligned_with_bias": c.aligned_with_bias,
            "klt_bridge": c.klt_bridge,
            "radar_gap": c.radar_gap,
            "radar_braked": all(w.servos[i].braked for i in (3, 4, 5)),
            "blocked": w.blocked,
            "wind_scale": self.wind_scale,
            "field_errors": self.field_errors,
            "weather": w.weather,
            "transmission": _f(w.transmission, 4),
            "rain_mm_h": w.rain_mm_h,
            "alerts": list(c.alerts),
            "corrections": c.corrections,
            "enc_test": None if c.enc_test is None else {k: v for k, v in c.enc_test.items() if k != "buf"},
            "psd": {
                k: {"x": _f(psd[k].x_mm, 4), "y": _f(psd[k].y_mm, 4), "valid": psd[k].valid,
                    "detect": psd[k].detect, "power": _f(psd[k].power, 3), "dip": psd[k].dip,
                    "drop": w._drops[k] is not None} for k in PSDS
            },
            "psd_geom": {
                "ac_size_mm": w.ac_size_mm, "b_size_mm": w.cfg.sensor.b_size_mm,
                "b_focal_mm": w.cfg.sensor.b_focal_mm, "spot_r99_mm": _f(geo.spot_r99_mm, 3),
                "mm_per_deg_ac": _f(geo.d * 1e3 * np.pi / 180, 2),
                "mm_per_deg_b": _f(w.cfg.sensor.b_focal_mm * np.pi / 180, 4),
            },
            "est": _vec(c.est) if c.est is not None else None,
            "est_use": list(c.use),
            "truth": _vec(true),
            "error_names": list(ERROR_NAMES),
            "verify": None if c.window is None else {
                "valid_fraction": _f(c.window["valid_fraction"], 3),
                "reported": _score(c.window["reported"]), "truth": _score(c.window["truth"]),
                "pass": c.window["pass"]},
            "track": None if tw is None else {
                "reported": _score(tw["reported"]), "truth": _score(tw["truth"]), "full": tw["full"]},
            "before": c.before,
            "before_camera": c.before_camera,
            "after": c.after,
            "servos": [{"name": n, "pos": _f(s.pos, 4), "enc": _f(s.encoder, 4), "target": _f(s.target, 4),
                        "braked": s.braked, "moving": s.moving, "travel": s.travel}
                       for n, s in zip(SERVO_NAMES, w.servos)],
            "incl": {k: _f(v, 4) for k, v in inc.items()},
            "dh_mm": _f(self._height_mm(inc), 1),
            "dh_true_mm": _f(geo.height_mm(w.trans, w.attitude()), 1),
            "scan": {"info": c.scan_info, "path": [list(p) for p in c.scan_path[-1500::6]],
                     "pos": [_f(w.servos[0].encoder, 4), _f(w.servos[1].encoder, 4)]},
            "emitter": geo.emitter,
            "pose": {"emitter": geo.emitter, "att_deg": _vec(w.attitude_deg(), 4), "trans_m": _vec(w.trans, 4),
                     "height_m": geo.h, "horn_lever_m": list(geo.horn_lever), "radar_lever_m": list(geo.radar_lever),
                     "lasers_m": {k: list(v) for k, v in geo.lasers.items()}},
            "events": [list(e) for e in new_events],
        }
        self._messages.append(msg)
        return msg
