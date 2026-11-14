"""Alignment state machine: IDLE -> ACQUIRE -> ALIGN -> VERIFY -> TRACK -> FAULT.

One call to `step()` is one tick (one camera sample). The controller only
talks to the `Hardware` interfaces and a clock, so the same code drives the
simulator and the real rig.

ALIGN   Quick looks (short window). Corrects X/Z until the radar error is
        inside the tracking band, then Pan/Tilt until the horn error is.
VERIFY  One full window scored with the pass criterion |mean| + 2*sigma <
        tolerance on all four axes. On a miss it trims every axis whose
        |mean| is above the deadband and verifies again.
TRACK   Rolling window. When |mean| drifts beyond the drift threshold it
        trims and verifies again; warns when vibration makes the criterion
        impossible.
FAULT   Motion stopped, with a reason.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Callable

from .config import Calibration, Config
from .estimator import ERROR_AXES, AxisStats, RunningMean, WindowEstimator, compute_errors_deg
from .geometry import intrinsics_from_config
from .interfaces import ActuatorError, Hardware

RADAR_AXES = ERROR_AXES[:2]
HORN_AXES = ERROR_AXES[2:]


class State(str, Enum):
    IDLE = "IDLE"
    ACQUIRE = "ACQUIRE"
    ALIGN = "ALIGN"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    FAULT = "FAULT"


@dataclass
class ControllerStatus:
    t_s: float
    state: str
    phase: str
    aligned: bool
    fault_reason: str | None
    warning: str | None
    distance_m: float | None
    errors: dict  # axis -> {last_deg, mean_deg, sigma_deg, score_deg, n}
    dx_mm: float | None  # horn center off the radar axis, to the radar's right
    dh_mm: float | None  # horn center above the radar axis
    actuators: dict  # axis -> position (mm or deg)
    before: dict | None  # axis -> mean_deg at the first measurement
    after: dict | None  # axis -> {mean_deg, sigma_deg, score_deg} at the passed verify
    iterations: int
    verify_attempts: int
    corrections: int  # re-alignments triggered while tracking
    events: list = field(default_factory=list)  # [t_s, text] since the last status

    def to_dict(self) -> dict:
        return asdict(self)


def _stats_dict(stats: dict[str, AxisStats]) -> dict:
    return {
        axis: {"mean_deg": s.mean, "sigma_deg": s.sigma, "score_deg": s.score, "n": s.n}
        for axis, s in stats.items()
    }


def _spiral(step: float, max_offset: float):
    """Square spiral of (pan, tilt) offsets, ring by ring."""
    ring = 1
    while ring * step <= max_offset + 1e-9:
        r = ring
        points = [(i, r) for i in range(-r, r + 1)] + [(r, j) for j in range(r - 1, -r - 1, -1)]
        points += [(i, -r) for i in range(r - 1, -r - 1, -1)] + [(-r, j) for j in range(-r + 1, r)]
        for i, j in points:
            yield i * step, j * step
        ring += 1


class Controller:
    def __init__(
        self,
        hw: Hardware,
        cfg: Config,
        calibration: Calibration,
        clock: Callable[[], float],
    ):
        self.hw = hw
        self.cfg = cfg
        self.calibration = calibration
        self._clock = clock
        self._intr = intrinsics_from_config(cfg.camera)
        self._tol = cfg.geometry.tolerance_deg
        self.state = State.IDLE
        self._reset()

    # ------------------------------------------------------------------ API
    def start(self) -> None:
        """Begin (or restart) an alignment from scratch."""
        self._reset()
        self._enter_acquire()

    def stop(self) -> None:
        self.hw.actuators.stop()
        self.state = State.IDLE
        self._phase = ""
        self._aligned = False

    def step(self) -> ControllerStatus:
        now = self._clock()
        sample = self._read(now)
        if self.state is State.ACQUIRE:
            self._step_acquire(now)
        elif self.state is State.ALIGN:
            self._step_align(now, sample)
        elif self.state is State.VERIFY:
            self._step_verify(now, sample)
        elif self.state is State.TRACK:
            self._step_track(now, sample)
        return self._status(now)

    # ------------------------------------------------------------- internals
    def _reset(self) -> None:
        self._phase = ""
        self._aligned = False
        self.fault_reason: str | None = None
        self._warning: str | None = None
        self._window = WindowEstimator()
        self._track = WindowEstimator(self.cfg.estimator.verify_window_s)
        self._distance = RunningMean()
        self._radar_roll = RunningMean()
        self._horn_roll = RunningMean()
        self._radar_valid = False
        self._horn_valid = False
        self._last_errors: dict[str, float] | None = None
        self._last_stats: dict[str, AxisStats] = {}
        self._before: dict | None = None
        self._after: dict | None = None
        self.verify_window: tuple[float, float] | None = None  # last passed verify
        self._iterations = 0
        self._verify_attempts = 0
        self._corrections = 0
        self._unstable_count = 0
        self._lost_count = 0
        self._settle_t0: float | None = None
        self._window_t0 = 0.0
        self._events: list = []

    def _event(self, now: float, text: str) -> None:
        self._events.append([now, text])

    def _read(self, now: float) -> dict[str, float] | None:
        radar = self.hw.radar_camera.measure_beacon()
        horn = self.hw.horn_camera.measure_beacon()
        self._radar_valid, self._horn_valid = radar.valid, horn.valid
        self._distance.add(self.hw.rangefinder.read_distance_m())
        self._radar_roll.add(self.hw.radar_inclinometer.read_tilt().roll_deg)
        self._horn_roll.add(self.hw.horn_inclinometer.read_tilt().roll_deg)
        if not (radar.valid and horn.valid):
            return None
        self._last_errors = compute_errors_deg(
            radar,
            horn,
            self._radar_roll.mean,
            self._horn_roll.mean,
            self._distance.mean,
            self.calibration,
            self._intr,
        )
        return self._last_errors

    def _fault(self, now: float, reason: str) -> None:
        self.hw.actuators.stop()
        self.state = State.FAULT
        self._phase = ""
        self._aligned = False
        self.fault_reason = reason
        self._event(now, f"FAULT: {reason}")

    def _settled(self, now: float) -> bool:
        if self.hw.actuators.is_moving():
            self._settle_t0 = None
            return False
        if self._settle_t0 is None:
            self._settle_t0 = now
        return now - self._settle_t0 >= self.cfg.estimator.settle_time_s

    def _begin(self, state: State, phase: str, now: float) -> None:
        self.state = state
        self._phase = phase
        self._settle_t0 = None
        self._lost_count = 0
        if phase == "measure":
            self._window.reset()
            self._window_t0 = now

    def _collect(self, now: float, sample, window_s: float) -> dict[str, AxisStats] | None:
        """Add a sample; return the statistics once the window is complete."""
        if sample is None:
            self._lost_count += 1
            if self._lost_count > self.cfg.controller.lost_frames:
                self._event(now, "beacon lost, re-acquiring")
                self._enter_acquire()
            return None
        self._lost_count = 0
        self._window.add(sample, now)
        if now - self._window_t0 < window_s - 1e-9 or self._window.n < 2:
            return None
        self._last_stats = self._window.stats()
        return self._last_stats

    def _command(self, now: float, axis: str, delta: float) -> bool:
        act = self.hw.actuators
        lo, hi = act.limits(axis)
        target = act.position(axis) + delta
        unit = "mm" if axis in ("x", "z") else "deg"
        if not lo <= target <= hi:
            self._fault(
                now,
                f"{axis} travel limit: needs {target:.2f} {unit}, limits [{lo:g}, {hi:g}] {unit}",
            )
            return False
        try:
            act.move_relative(axis, delta)
        except ActuatorError as exc:
            self._fault(now, f"actuator error: {exc}")
            return False
        return True

    def _correct(
        self, now: float, stats: dict[str, AxisStats], threshold: float, combined: bool = False
    ) -> bool:
        """Command one correction; return True if anything was commanded
        (or a FAULT was raised).

        Position (X/Z) has priority over pointing (Pan/Tilt). Moving the horn
        sideways by an angle eps (seen from the radar) also turns the line of
        sight seen from the horn by eps. With `combined` (fine trims) the
        Pan/Tilt correction is issued in the same step and includes that
        known coupling; otherwise Pan/Tilt waits for the next measurement.
        """
        gain = self.cfg.controller.gain
        d_mm = self._distance.mean * 1000.0
        radar_az = gain * stats["radar_az"].mean if abs(stats["radar_az"].mean) > threshold else 0.0
        radar_el = gain * stats["radar_el"].mean if abs(stats["radar_el"].mean) > threshold else 0.0
        moves = []
        # Radar sees the horn to its right -> horn moves to its own right
        # (they face each other). Radar sees it high -> horn moves down.
        if radar_az:
            moves.append(("x", d_mm * math.tan(math.radians(radar_az))))
        if radar_el:
            moves.append(("z", -d_mm * math.tan(math.radians(radar_el))))
        if combined or not moves:
            horn_az = stats["horn_az"].mean - radar_az  # after the X move
            horn_el = stats["horn_el"].mean + radar_el  # after the Z move
            if abs(horn_az) > threshold:
                moves.append(("pan", gain * horn_az))
            if abs(horn_el) > threshold:
                moves.append(("tilt", gain * horn_el))
        for axis, delta in moves:
            if not self._command(now, axis, delta):
                break
        return bool(moves)

    def _unstable_axes(self, stats: dict[str, AxisStats]) -> list[str]:
        """Axes where 2*sigma alone already reaches the tolerance."""
        return [a for a in ERROR_AXES if 2.0 * stats[a].sigma >= self._tol]

    def _unstable_text(self, stats: dict[str, AxisStats], axes: list[str]) -> str:
        worst = max(axes, key=lambda a: stats[a].sigma)
        return (
            f"2*sigma = {2 * stats[worst].sigma:.3f} deg on {worst} >= "
            f"{self._tol:g} deg tolerance, pass criterion impossible"
        )

    # --------------------------------------------------------------- ACQUIRE
    def _enter_acquire(self) -> None:
        now = self._clock()
        self.state = State.ACQUIRE
        self._phase = "search"
        self._aligned = False
        self._acquire_t0 = now
        self._acquire_ok = 0
        self._search = _spiral(self.cfg.controller.search_step_deg, self.cfg.controller.search_max_deg)
        act = self.hw.actuators
        self._search_origin = (act.position("pan"), act.position("tilt"))
        self._settle_t0 = None

    def _step_acquire(self, now: float) -> None:
        ctl = self.cfg.controller
        if self._radar_valid and self._horn_valid:
            self._acquire_ok += 1
            if self._acquire_ok >= ctl.acquire_frames:
                self.hw.actuators.stop()
                self._begin(State.ALIGN, "settle", now)
            return
        self._acquire_ok = 0
        if now - self._acquire_t0 > ctl.acquire_timeout_s:
            missing = [
                name
                for name, ok in (("radar camera", self._radar_valid), ("horn camera", self._horn_valid))
                if not ok
            ]
            self._fault(
                now,
                f"beacon not found after {ctl.acquire_timeout_s:g} s ({' and '.join(missing)})",
            )
            return
        # Only the horn can move, so only a blind horn camera is worth a search.
        if self._horn_valid or self.hw.actuators.is_moving():
            self._settle_t0 = None
            return
        if self._settle_t0 is None:
            self._settle_t0 = now
        if now - self._settle_t0 < ctl.search_dwell_s:
            return
        self._settle_t0 = None
        act = self.hw.actuators
        for pan_off, tilt_off in self._search:
            pan = self._search_origin[0] + pan_off
            tilt = self._search_origin[1] + tilt_off
            (pan_lo, pan_hi), (tilt_lo, tilt_hi) = act.limits("pan"), act.limits("tilt")
            if pan_lo <= pan <= pan_hi and tilt_lo <= tilt <= tilt_hi:
                try:
                    act.move_relative("pan", pan - act.position("pan"))
                    act.move_relative("tilt", tilt - act.position("tilt"))
                except ActuatorError as exc:
                    self._fault(now, f"actuator error: {exc}")
                return

    # ----------------------------------------------------------------- ALIGN
    def _step_align(self, now: float, sample) -> None:
        if self._phase in ("settle", "trim"):
            if self._settled(now):
                if self._phase == "trim":
                    self._begin(State.VERIFY, "measure", now)
                else:
                    self._begin(State.ALIGN, "measure", now)
            return
        stats = self._collect(now, sample, self.cfg.estimator.align_window_s)
        if stats is None:
            return
        if self._before is None:
            self._before = {a: stats[a].mean for a in ERROR_AXES}

        unstable = self._unstable_axes(stats)
        self._unstable_count = self._unstable_count + 1 if unstable else 0
        if self._unstable_count >= self.cfg.controller.unstable_windows:
            self._fault(now, "unstable: " + self._unstable_text(stats, unstable))
            return

        # Coarse stage ends inside the tracking band, so TRACK does not
        # immediately call the result a drift.
        if self._correct(now, stats, self.cfg.controller.track_drift_threshold_deg):
            if self.state is State.FAULT:
                return
            self._iterations += 1
            if self._iterations > self.cfg.controller.max_iterations:
                self._fault(
                    now, f"no convergence after {self.cfg.controller.max_iterations} corrections"
                )
                return
            self._begin(State.ALIGN, "settle", now)
        else:
            self._begin(State.VERIFY, "measure", now)

    # ---------------------------------------------------------------- VERIFY
    def _step_verify(self, now: float, sample) -> None:
        stats = self._collect(now, sample, self.cfg.estimator.verify_window_s)
        if stats is None:
            return
        self._verify_attempts += 1
        if all(stats[a].score < self._tol for a in ERROR_AXES):
            self._aligned = True
            self._after = _stats_dict(stats)
            self.verify_window = (self._window_t0, now)
            self._event(now, "ALIGNED")
            self.state = State.TRACK
            self._phase = "track"
            self._track.reset()
            self._warning = None
            self._lost_count = 0
            return

        unstable = self._unstable_axes(stats)
        if unstable:
            self._fault(now, "unstable: " + self._unstable_text(stats, unstable))
            return
        if self._verify_attempts >= self.cfg.controller.max_verify_attempts:
            worst = max(ERROR_AXES, key=lambda a: stats[a].score)
            self._fault(
                now,
                f"verify failed {self._verify_attempts} times: |mean| + 2*sigma = "
                f"{stats[worst].score:.3f} deg on {worst}",
            )
            return
        if self._correct(now, stats, self.cfg.controller.deadband_deg, combined=True):
            if self.state is State.FAULT:
                return
            self._begin(State.ALIGN, "trim", now)
        else:
            self._begin(State.VERIFY, "measure", now)

    # ----------------------------------------------------------------- TRACK
    def _step_track(self, now: float, sample) -> None:
        if sample is None:
            self._lost_count += 1
            if self._lost_count > self.cfg.controller.lost_frames:
                self._event(now, "beacon lost, re-acquiring")
                self._enter_acquire()
            return
        self._lost_count = 0
        self._track.add(sample, now)
        if self._track.span_s < 0.9 * self.cfg.estimator.verify_window_s:
            return
        stats = self._last_stats = self._track.stats()
        self._aligned = all(stats[a].score < self._tol for a in ERROR_AXES)
        unstable = self._unstable_axes(stats)
        if unstable:
            self._warning = "unstable conditions: " + self._unstable_text(stats, unstable)
            return  # do not chase vibration with the motors
        self._warning = None
        threshold = self.cfg.controller.track_drift_threshold_deg
        drifted = [a for a in ERROR_AXES if abs(stats[a].mean) > threshold]
        if drifted:
            worst = max(drifted, key=lambda a: abs(stats[a].mean))
            self._event(now, f"drift {stats[worst].mean:+.3f} deg on {worst}, re-aligning")
            self._corrections += 1
            self._iterations = 0
            self._verify_attempts = 0
            self._unstable_count = 0
            self._aligned = False
            self._correct(now, stats, self.cfg.controller.deadband_deg, combined=True)
            if self.state is not State.FAULT:
                self._begin(State.ALIGN, "trim", now)

    # ---------------------------------------------------------------- status
    def _status(self, now: float) -> ControllerStatus:
        errors = {}
        for axis in ERROR_AXES:
            entry = {"last_deg": self._last_errors[axis] if self._last_errors else None}
            if axis in self._last_stats:
                s = self._last_stats[axis]
                entry.update(mean_deg=s.mean, sigma_deg=s.sigma, score_deg=s.score, n=s.n)
            errors[axis] = entry
        d = self._distance.mean
        dx = dh = None
        if self._last_errors and d:
            dx = d * math.tan(math.radians(self._last_errors["radar_az"])) * 1000.0
            dh = d * math.tan(math.radians(self._last_errors["radar_el"])) * 1000.0
        act = self.hw.actuators
        events, self._events = self._events, []
        return ControllerStatus(
            t_s=now,
            state=self.state.value,
            phase=self._phase,
            aligned=self._aligned,
            fault_reason=self.fault_reason,
            warning=self._warning,
            distance_m=d,
            errors=errors,
            dx_mm=dx,
            dh_mm=dh,
            actuators={axis: act.position(axis) for axis in ("x", "z", "pan", "tilt")},
            before=self._before,
            after=self._after,
            iterations=self._iterations,
            verify_attempts=self._verify_attempts,
            corrections=self._corrections,
            events=events,
        )
