"""Run one simulated alignment and score it against the truth."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import Config
from .controller import Controller, ControllerStatus, State
from .estimator import ERROR_AXES
from .sensors import Simulation, build_simulation


@dataclass
class RunResult:
    seed: object
    distance_m: float
    final_state: str
    aligned: bool  # the system declared ALIGNED at least once
    fault_reason: str | None
    t_aligned_s: float | None  # time of the first ALIGNED
    duration_s: float
    iterations: int
    verify_attempts: int
    corrections: int
    before: dict | None  # system-reported means at the first measurement
    reported: dict | None  # system-reported stats of the passed verify window
    truth_verify: dict | None  # TRUE stats over that same window
    truth_track: dict | None  # TRUE stats over the tracking period
    history: dict | None = None


def _truth_stats(rows: np.ndarray) -> dict | None:
    """rows: one row per tick, one column per axis (true error, deg)."""
    if len(rows) < 2:
        return None
    out = {}
    for i, axis in enumerate(ERROR_AXES):
        mean = float(rows[:, i].mean())
        sigma = float(rows[:, i].std(ddof=1))
        out[axis] = {"mean_deg": mean, "sigma_deg": sigma, "score_deg": abs(mean) + 2.0 * sigma}
    return out


def run_single(
    cfg: Config,
    seed,
    distance_m: float | None = None,
    max_time_s: float | None = None,
    track_time_s: float | None = None,
    record_history: bool = False,
    on_tick: Callable[[ControllerStatus, dict, Simulation], None] | None = None,
    **scenario_overrides,
) -> RunResult:
    """Simulate start -> ALIGNED -> `track_time_s` of tracking (or FAULT/timeout)."""
    max_time_s = cfg.montecarlo.max_time_s if max_time_s is None else max_time_s
    track_time_s = cfg.montecarlo.track_time_s if track_time_s is None else track_time_s
    sim = build_simulation(cfg, seed, distance_m, **scenario_overrides)
    world = sim.world
    ctrl = Controller(sim.hardware, cfg, sim.calibration, clock=lambda: world.t)
    ctrl.start()
    dt = 1.0 / cfg.camera.rate_hz

    times, truths, states, measured = [], [], [], []
    t_aligned = None
    first_window = None
    status = None
    while world.t < max_time_s:
        world.advance(dt)
        status = ctrl.step()
        truth = world.true_errors_deg()
        times.append(world.t)
        truths.append([truth[a] for a in ERROR_AXES])
        states.append(status.state)
        if record_history:
            measured.append([status.errors[a]["last_deg"] for a in ERROR_AXES])
        if on_tick is not None:
            on_tick(status, truth, sim)
        if t_aligned is None and ctrl.verify_window is not None:
            first_window = ctrl.verify_window
            t_aligned = world.t
        if status.state == State.FAULT.value:
            break
        if t_aligned is not None and world.t - t_aligned >= track_time_s:
            break

    t = np.array(times)
    truth_rows = np.array(truths)
    truth_verify = truth_track = None
    if first_window is not None:
        in_window = (t > first_window[0] + 1e-9) & (t <= first_window[1] + 1e-9)
        truth_verify = _truth_stats(truth_rows[in_window])
        tracking = (t > t_aligned) & (np.array(states) == State.TRACK.value)
        truth_track = _truth_stats(truth_rows[tracking])

    fault_reason = status.fault_reason
    if status.state != State.FAULT.value and t_aligned is None:
        fault_reason = f"timeout: not aligned within {max_time_s:g} s"

    history = None
    if record_history:
        history = {
            "t_s": t,
            "truth_deg": truth_rows,
            "measured_deg": np.array(measured, dtype=float),
            "state": states,
        }
    return RunResult(
        seed=seed,
        distance_m=sim.scenario.distance_m,
        final_state=status.state,
        aligned=t_aligned is not None,
        fault_reason=fault_reason,
        t_aligned_s=t_aligned,
        duration_s=world.t,
        iterations=status.iterations,
        verify_attempts=status.verify_attempts,
        corrections=status.corrections,
        before=status.before,
        reported=status.after if t_aligned is not None else None,
        truth_verify=truth_verify,
        truth_track=truth_track,
        history=history,
    )
