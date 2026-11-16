"""Final-design controller: CALIB SETUP ACQUIRE SCAN ALIGN VERIFY TRACK FAULT.

Reads only sensors (PSDs, inclinometers, rangefinder, camera, encoders) and
commands the six servos. Never uses radar data. docs/FINAL_DESIGN.md section 5.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .config import TOLERANCE_DEG, PsdConfig
from .estimator import PsdEstimator, readings_vector, window_score
from .model import Geometry, SERVO_NAMES
from .world import World

STATES = ("IDLE", "CALIB", "SETUP", "ACQUIRE", "SCAN", "ALIGN", "VERIFY", "TRACK", "FAULT")
HORN = (0, 1, 2)
RADAR = (3, 4, 5)


class PsdController:
    def __init__(self, world: World):
        self.w = world
        self.cfg: PsdConfig = world.cfg
        self.c = self.cfg.control
        self.dt = 1.0 / self.cfg.sensor.rate_hz
        # Emitter = the side with the lasers and the camera (it scans and acquires).
        self.emitter = world.geo.emitter
        self.E_SRV = (3, 4) if self.emitter == "radar" else (0, 1)
        self.E_ERR = (2, 3) if self.emitter == "radar" else (0, 1)
        self.E_NAME = self.emitter
        self.state = "IDLE"
        self.sub = ""
        self.fault_reason = ""
        self.events: list[tuple[float, str]] = []
        self.radar_gap = False  # operator/radar PC allows radar correction (Q21, Q22)
        self.est: np.ndarray | None = None
        self.use: tuple[str, ...] = ()
        self.before: dict | None = None
        self.before_camera: dict | None = None  # emitter error from the camera, before any correction (R5)
        self.after: dict | None = None
        self.window: dict | None = None
        self.track_window: dict | None = None
        self.alerts: list[str] = []
        self.corrections = 0
        self.klt_bridge = False
        self.scan_path: list[tuple[float, float]] = []
        self.scan_info: dict = {}
        self._reset_buffers()

    # ------------------------------------------------------------ helpers
    def _reset_buffers(self) -> None:
        self._t_state = 0.0
        self._buf: list = []
        self._iter = 0
        self._verify_attempts = 0
        self._integral = np.zeros(6)
        self._settle_t = None
        self._lost_t = 0.0
        self._track_est = deque()
        self._track_true = deque()
        self._track_ok = deque()
        self._track_eval_t = 0.0
        self._wp: list[tuple[float, float]] = []
        self._scan_expansions = 0
        self._radar_fix = False
        self._side_only = None

    def _log(self, text: str) -> None:
        self.events.append((round(self.w.t, 2), text))

    def _goto(self, state: str, sub: str = "") -> None:
        if state != self.state:
            self._log(f"{self.state} -> {state}" + (f" ({sub})" if sub else ""))
        self.state, self.sub = state, sub
        self._t_state = 0.0
        self._buf = []
        self._settle_t = None

    def _fault(self, reason: str) -> None:
        self.fault_reason = reason
        self._goto("FAULT")
        self._log(f"FAULT: {reason}")

    def _enc(self) -> np.ndarray:
        return np.array([s.encoder for s in self.w.servos])

    def _command(self, idx, delta_deg) -> bool:
        ok = True
        for i, dlt in zip(idx, delta_deg):
            s = self.w.servos[i]
            if not s.command(s.encoder + float(dlt)):
                ok = False
            if s.at_limit:
                self._fault(f"{SERVO_NAMES[i]} at travel limit ({s.travel:.1f} deg)")
                return False
        return ok

    def _Jsub(self, rows, srv) -> np.ndarray:
        return self.est_model.Jj[np.ix_(list(rows), list(srv))]

    def _settled(self) -> bool:
        if any(s.moving for s in self.w.servos):
            self._settle_t = None
            return False
        if self._settle_t is None:
            self._settle_t = self.w.t
        return self.w.t - self._settle_t >= self.c.settle_s

    def _brake_radar(self, on: bool) -> None:
        for i in RADAR:
            s = self.w.servos[i]
            if on:
                s.target = s.pos
            s.braked = on
        self._log("radar axes " + ("braked" if on else "released"))

    # ------------------------------------------------------------ commands
    def start(self) -> None:
        self._reset_buffers()
        self.fault_reason = ""
        self.alerts = []
        self.after = None
        self.before = None
        self.before_camera = None
        self.track_window = None
        self.window = None
        self.klt_bridge = False
        for i in RADAR:
            self.w.servos[i].braked = False
        self._goto("CALIB")

    def stop(self) -> None:
        for s in self.w.servos:
            s.target = s.pos
        self._goto("IDLE")

    # ------------------------------------------------------------ main tick
    def tick(self) -> None:
        self._t_state += self.dt
        psd = self.w.read_psds()
        self.last_psd = psd
        getattr(self, "_s_" + self.state.lower())(psd)

    def _s_idle(self, psd) -> None:
        pass

    def _s_fault(self, psd) -> None:
        pass

    def _s_calib(self, psd) -> None:
        if not self.w.calibration_loaded:
            self._fault("calibration file missing (lasers, PSD targets, PSD-B null)")
            return
        if self._t_state >= 0.3:
            d = self.w.read_range()
            # height difference from the two inclinometers (both sides are aimed
            # roughly at each other), not from any scenario value
            inc = self.w.read_inclinometers()
            g = self.cfg.geometry
            h = (d - g.horn_lever_m[0] - g.radar_lever_m[0]) * math.tan(
                math.radians(0.5 * (inc["horn_tilt"] - inc["radar_tilt"])))
            self.est_model = PsdEstimator(Geometry(self.cfg, d, h))
            self._log(f"calibration loaded; range {d:.2f} m, height difference about {h * 1e3:+.0f} mm")
            self._goto("SETUP", "level rolls")

    def _s_setup(self, psd) -> None:
        if self.sub == "level rolls":
            inc = self.w.read_inclinometers()
            self._command((2, 5), (-inc["horn_roll"], -inc["radar_roll"]))
            self.sub = "settle"
            return
        if self._settled():
            self._goto("ACQUIRE", "template match")

    def _s_acquire(self, psd) -> None:
        if self.sub == "template match":
            cam = self.w.camera_acquire()
            if cam is None:
                if self._t_state > 3.0:
                    self._fault("camera does not see the PSD unit (outside FOV)")
                return
            if self.before_camera is None:
                tru = self.w.true_errors()
                self.before_camera = {"side": self.E_NAME, "axes": list(self.E_ERR), "est": [float(v) for v in cam],
                                      "true": [float(tru[i]) for i in self.E_ERR]}
            self._command(self.E_SRV, -np.linalg.solve(self._Jsub(self.E_ERR, self.E_SRV), np.array(cam)))
            self._log(f"PSD unit found by template match, KLT locked; coarse {self.E_NAME} error {cam[0]:+.3f}/{cam[1]:+.3f} deg")
            self.sub = "coarse point"
            return
        if self._settled():
            self._start_scan()

    # ------------------------------------------------------------ SCAN
    def _start_scan(self, half_deg: float | None = None) -> None:
        geo = self.w.geo
        d_mm = self.w.read_range() * 1e3
        step_mm = max(self.w.ac_size_mm - 2 * geo.spot_r99_mm, 0.5)
        step = math.degrees(math.atan(step_mm / d_mm))
        half = half_deg or max(self.c.scan_sigmas * self.cfg.camera.acquire_sigma_deg, step)
        enc = self._enc()
        c_pan, c_tilt = enc[self.E_SRV[0]], enc[self.E_SRV[1]]
        n = int(math.ceil(2 * half / step)) + 1
        tilts = np.linspace(c_tilt - half, c_tilt + half, n)
        wp = []
        for i, tl in enumerate(tilts):
            row = [(c_pan - half, tl), (c_pan + half, tl)]
            wp += row if i % 2 == 0 else row[::-1]
        # start from the centre line outwards would be faster on average; the
        # serpentine keeps the picture simple and the time bound exact.
        self._wp = wp
        self.scan_info = {"centre": [float(c_pan), float(c_tilt)], "half_deg": float(half), "step_deg": float(step)}
        self.scan_path = []
        self._goto("SCAN", f"{self.E_NAME} raster +/-{half:.3f} deg, step {step:.4f} deg")

    def _s_scan(self, psd) -> None:
        enc = self._enc()
        p, t = self.E_SRV
        self.scan_path.append((float(enc[p]), float(enc[t])))
        if psd["A"].valid or psd["C"].valid:
            for i in self.E_SRV:
                s = self.w.servos[i]
                s.target = s.pos
            self._log(f"beam captured on PSD-{'A' if psd['A'].valid else 'C'} after {self._t_state:.1f} s of scan")
            self._goto("ALIGN", "settle")
            return
        if self.w.servos[p].moving or self.w.servos[t].moving:
            return
        if not self._wp:
            if self._scan_expansions < 2:
                self._scan_expansions += 1
                self._start_scan(self.scan_info["half_deg"] * 2)
                return
            self._fault("beam not found in the raster window")
            return
        pan, tilt = self._wp.pop(0)
        self.w.servos[p].command(pan)
        self.w.servos[t].command(tilt)

    # ------------------------------------------------------------ ALIGN
    def _measure(self, psd, need: tuple[str, ...]):
        ok = all(psd[k].valid for k in need)
        self._buf.append((readings_vector(psd), ok, self.w.true_errors(), psd["C"].valid))

    def _s_align(self, psd) -> None:
        if self.sub == "settle":
            if self._settled():
                self.sub = "measure"
                self._buf = []
            return
        if self.sub == "roll search":
            if psd["C"].valid:
                self._log("PSD-C captured by roll search")
                self.sub = "settle"
            elif self._settled():
                k = len(self._buf) + 1
                self._buf.append(None)
                offset = 0.2 * ((k + 1) // 2) * (1 if k % 2 else -1)
                if abs(offset) > 2.0:
                    self._fault("PSD-C not found by roll search")
                    return
                self.w.servos[2].command(self._roll0 + offset)
            return
        # measure
        self._measure(psd, ("A", "B"))
        if len(self._buf) * self.dt < self.c.align_window_s:
            return
        good = [b for b in self._buf if b[1]]
        if len(good) < 0.5 * len(self._buf):
            if not psd["A"].detect and not psd["C"].detect:
                self._log("beam lost during ALIGN")
                self._start_scan()
                return
            # spot partly off PSD-A/C: re-centre the emitter on the clipped centroid
            m = np.mean([b[0] for b in self._buf], axis=0)
            i, j = self.E_ERR
            rough = np.array([(m[0] + m[4]) / 2 / self.est_model.G[0, i], (m[1] + m[5]) / 2 / self.est_model.G[1, j]])
            self._command(self.E_SRV, -np.linalg.solve(self._Jsub(self.E_ERR, self.E_SRV), rough))
            self._log(f"spot at the PSD edge: {self.E_NAME} re-centred by {rough[0]:+.3f}/{rough[1]:+.3f} deg")
            self.sub = "settle"
            return
        use_c = np.mean([b[3] for b in good]) >= 0.8
        if not use_c and not any(b[3] for b in good) and not psd["C"].detect:
            self.sub = "roll search"
            self._buf = []
            self._roll0 = self.w.servos[2].encoder
            return
        self.use = ("A", "B", "C") if use_c else ("A", "B")
        m = np.mean([b[0] for b in good], axis=0)
        inc = self.w.read_inclinometers()
        e = self.est_model.estimate(m, self.use, inc)
        self.est = e
        if self.before is None:
            self.before = {"est": e.tolist(), "true": np.mean([b[2] for b in good], axis=0).tolist()}
            self._log("before: horn {:+.3f}/{:+.3f}, radar {:+.3f}/{:+.3f} deg".format(*e[:4]))
        roll_known = "C" in self.use or self.cfg.control.roll_from_inclinometers
        target = np.array([*e[:4], e[4] if roll_known else 0.0, inc["radar_roll"]])
        if self._side_only is not None:
            # re-centre during the test on one side only, then back to TRACK
            rows, srv, name = self._side_only
            if np.all(np.abs(target[list(rows)]) < self.c.deadband_deg):
                self._side_only = None
                if name == "radar":
                    self._brake_radar(True)
                self._track_est.clear(); self._track_true.clear(); self._track_ok.clear()
                self._goto("TRACK", "radar braked, horn PID")
                return
            self._iter += 1
            if self._iter > self.c.max_iterations:
                self._fault(f"no convergence in {name} re-alignment")
                return
            delta = -self.c.kp * np.linalg.solve(self._Jsub(rows, srv), target[list(rows)])
            self.corrections += 1
            if self._command(srv, delta):
                self._log(f"{name} re-align #{self._iter}: {delta[0]:+.3f}/{delta[1]:+.3f} deg")
            self.sub = "settle"
            return
        if np.all(np.abs(target[:4]) < self.c.deadband_deg) and abs(target[4]) < 0.1:
            self._goto("VERIFY", f"{self.c.verify_window_s:.0f} s, no correction")
            return
        self._iter += 1
        if self._iter > self.c.max_iterations:
            self._fault("no convergence in ALIGN")
            return
        self._integral += target
        u = self.c.kp * target + self.c.ki * self._integral
        delta = -np.linalg.solve(self.est_model.Jj, u)
        self.corrections += 1
        if self._command(range(6), delta):
            self._log(f"ALIGN #{self._iter}: horn {delta[0]:+.3f}/{delta[1]:+.3f}, radar {delta[3]:+.3f}/{delta[4]:+.3f} deg")
        self.sub = "settle"

    # ------------------------------------------------------------ VERIFY
    def _sample_errors(self, psd):
        use = self.use or ("A", "B")
        ok = all(psd[k].valid for k in use)
        e = self.est_model.estimate(readings_vector(psd), use, self.w.read_inclinometers()) if ok else None
        return ok, e

    def _s_verify(self, psd) -> None:
        ok, e = self._sample_errors(psd)
        self._buf.append((ok, e, self.w.true_errors()))
        if len(self._buf) * self.dt < self.c.verify_window_s:
            return
        est = np.array([b[1] for b in self._buf if b[0]])
        frac = len(est) / len(self._buf)
        true = np.array([b[2] for b in self._buf])
        ts = window_score(true)
        if len(est) > 1:
            rs = window_score(est, self.est_model.u_bias_deg(self.use or ("A", "B")))
            passed = rs["pass"] and frac >= self.c.min_valid_fraction
        else:
            rs, passed = None, False
        self.window = {"kind": "verify", "valid_fraction": frac, "reported": rs, "truth": ts, "pass": passed}
        if passed:
            self.after = {"reported": {k: v.tolist() for k, v in rs.items() if not k.startswith("pass")},
                          "pass_with_bias": rs["pass_with_bias"],
                          "truth": {k: v.tolist() for k, v in ts.items() if k != "pass"},
                          "truth_pass": ts["pass"], "valid_fraction": frac}
            self._log("VERIFY passed: score " + " ".join(f"{s:.4f}" for s in rs["score"])
                      + f" (truth {' '.join(f'{s:.4f}' for s in ts['score'])})")
            self._brake_radar(True)
            self._goto("TRACK", "radar braked, horn PID")
            return
        self._verify_attempts += 1
        why = "too few valid samples" if frac < self.c.min_valid_fraction else "criterion not met"
        self._log(f"VERIFY failed ({why}, valid {frac:.0%})")
        if self._verify_attempts >= self.c.max_verify_attempts:
            self._fault(f"VERIFY failed {self._verify_attempts} times ({why}): conditions too unstable")
            return
        self._goto("ALIGN", "settle")

    # ------------------------------------------------------------ TRACK
    def _s_track(self, psd) -> None:
        if not psd["A"].detect:
            self._lost_t += self.dt
            self.klt_bridge = True
            if self._lost_t > self.c.beam_lost_s:
                if self.emitter == "radar" and not self.radar_gap:
                    msg = "beam lost: re-scan needs the radar; waiting for a correction gap"
                    if msg not in self.alerts:
                        self.alerts.append(msg)
                        self._log("ALERT " + msg)
                    return
                self._log(f"beam lost > {self.c.beam_lost_s:.1f} s: back to SCAN with the {self.E_NAME}")
                self.alerts = [a for a in self.alerts if not a.startswith("beam lost")]
                self.klt_bridge = False
                self._lost_t = 0.0
                if self.emitter == "radar":
                    self._brake_radar(False)
                self._start_scan()
            return
        if self.klt_bridge:
            self._log(f"beam back after {self._lost_t:.2f} s (KLT bridged the {self.E_NAME} pointing)")
            self.alerts = [a for a in self.alerts if not a.startswith("beam lost")]
        self.klt_bridge = False
        self._lost_t = 0.0
        ok, e = self._sample_errors(psd)
        n_max = int(self.c.track_window_s / self.dt)
        self._track_true.append(self.w.true_errors())
        self._track_ok.append(ok)
        while len(self._track_ok) > n_max:
            self._track_ok.popleft()
        if ok:
            self._track_est.append(e)
            self.est = e
        recent = list(self._track_ok)[-int(1.0 / self.dt):]
        if not self._radar_fix and len(recent) >= int(1.0 / self.dt) and np.mean(recent) < 0.5:
            if self.emitter == "radar" and not self.radar_gap:
                msg = "radar moved: spot leaving the PSD; radar correction needs a gap"
                if msg not in self.alerts:
                    self.alerts.append(msg)
                    self._log("ALERT " + msg)
            else:
                self._log(f"spot leaving the PSD in TRACK: {self.E_NAME} re-aligns")
                self.alerts = [a for a in self.alerts if not a.startswith("radar moved")]
                self._track_ok.clear()
                self._iter = 0
                if self.emitter == "radar":
                    self._brake_radar(False)
                self._side_only = (self.E_ERR, self.E_SRV, self.E_NAME)
                self._goto("ALIGN", "settle")
                return
        elif len(recent) and np.mean(recent) >= 0.9:
            self.alerts = [a for a in self.alerts if not a.startswith("radar moved")]
        while len(self._track_est) > n_max:
            self._track_est.popleft()
        while len(self._track_true) > n_max:
            self._track_true.popleft()
        if self._radar_fix:
            if self._settled():
                self._brake_radar(True)
                self._radar_fix = False
                self._track_est.clear()
                self._track_true.clear()
            return
        if len(self._track_est) > 10:
            rs = window_score(np.array(self._track_est), self.est_model.u_bias_deg(self.use or ("A", "B")))
            ts = window_score(np.array(self._track_true))
            self.track_window = {"kind": "track", "reported": rs, "truth": ts,
                                 "full": len(self._track_true) >= n_max,
                                 "valid_fraction": float(np.mean(self._track_ok))}
        self._track_eval_t += self.dt
        if self._track_eval_t < 1.0 or len(self._track_est) < n_max // 2:
            return
        self._track_eval_t = 0.0
        mean = np.mean(self._track_est, axis=0)
        if max(abs(mean[0]), abs(mean[1])) > self.c.track_threshold_deg:
            J = self.est_model.Jj[0:2, 0:2]
            delta = -self.c.track_kp * np.linalg.solve(J, mean[:2])
            if self._command((0, 1), delta):
                self.corrections += 1
                self._log(f"TRACK horn correction {delta[0]:+.3f}/{delta[1]:+.3f} deg")
                self._track_est.clear()
                self._track_true.clear()
        if max(abs(mean[2]), abs(mean[3])) > self.c.radar_alert_deg:
            msg = f"radar drift {mean[2]:+.3f}/{mean[3]:+.3f} deg > {self.c.radar_alert_deg} deg"
            if msg not in self.alerts:
                self.alerts = [a for a in self.alerts if not a.startswith("radar drift")] + [msg]
                self._log("ALERT " + msg + (" -> correcting in allowed gap" if self.radar_gap else " (radar braked; waiting for a gap)"))
            if self.radar_gap:
                self._brake_radar(False)
                J = self.est_model.Jj[2:4, 3:5]
                delta = -np.linalg.solve(J, mean[2:4])
                self._command((3, 4), delta)
                self.corrections += 1
                self._radar_fix = True
                self._settle_t = None
                self.alerts = [a for a in self.alerts if not a.startswith("radar drift")]
        else:
            self.alerts = [a for a in self.alerts if not a.startswith("radar drift")]

    # ------------------------------------------------------------ status
    @property
    def aligned_with_bias(self) -> bool:
        """Also passes with the calibration uncertainty added (strict state)."""
        if not self.aligned:
            return False
        tw = self.track_window
        if tw is not None and tw["full"]:
            return bool(tw["reported"]["pass_with_bias"])
        return bool(self.after and self.after["pass_with_bias"])

    @property
    def aligned(self) -> bool:
        if self.state != "TRACK" or self.klt_bridge:
            return False
        tw = self.track_window
        if tw is None or not tw["full"]:
            return self.after is not None
        return tw["reported"]["pass"] and tw["valid_fraction"] >= self.c.min_valid_fraction
