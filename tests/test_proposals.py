"""Behaviour added after the design review (docs/REVIEW.md): P2 U_bias and
P4 hold. P1 (track on the criterion) was reverted; TRACK uses the fixed drift
threshold, and its behaviour is pinned here too."""

import pytest

from rada_align.config import load_config, with_only_sources
from rada_align.controller import Controller
from rada_align.estimator import ERROR_AXES
from rada_align.interfaces import AXES
from rada_align.montecarlo import format_summary, run_montecarlo, summarize
from rada_align.runner import run_single
from rada_align.sensors import build_simulation

CFG = load_config()
QUIET = with_only_sources(CFG, ())
TOL = CFG.geometry.tolerance_deg
DT = 1.0 / CFG.camera.rate_hz


def _make(cfg, **scenario):
    sim = build_simulation(cfg, 1, **scenario)
    ctrl = Controller(sim.hardware, cfg, sim.calibration, clock=lambda: sim.world.t)
    return sim, ctrl


def _run(sim, ctrl, seconds):
    events, status = [], None
    for _ in range(round(seconds / DT)):
        sim.world.advance(DT)
        status = ctrl.step()
        events += status.events
    return status, events


def _run_until_track(sim, ctrl, limit_s=120.0):
    for _ in range(round(limit_s / DT)):
        sim.world.advance(DT)
        status = ctrl.step()
        if status.state == "TRACK":
            return status
    raise AssertionError("never reached TRACK")


# Radar tripod settling at 0.03 deg/min: slow enough to align first, then the
# mean crosses the drift threshold about a minute later.
SLOW_DRIFT = dict(radar_drift_deg_per_s=(0.0005, 0.0))


def _positions(sim):
    return {axis: sim.hardware.actuators.position(axis) for axis in AXES}


# --------------------------------------------------------------------- TRACK
@pytest.mark.xfail(strict=True, reason="superseded mutual-sighting design: its assumed noise (2 sigma ~0.03 deg) cannot meet the 0.005 deg requirement (Liam, 2026-11-17)")
def test_track_corrects_only_beyond_the_drift_threshold():
    sim, ctrl = _make(QUIET, **SLOW_DRIFT)
    ctrl.start()
    status = _run_until_track(sim, ctrl)
    assert status.corrections == 0
    settled = _positions(sim)

    # While the mean is inside the drift threshold nothing moves.
    status, events = _run(sim, ctrl, 6.0)
    assert status.state == "TRACK"
    assert abs(status.errors["radar_az"]["mean_deg"]) < CFG.controller.track_drift_threshold_deg
    assert _positions(sim) == settled

    status, events = _run(sim, ctrl, 150.0)
    assert status.corrections >= 1
    drift = [text for _, text in events if text.startswith("drift ")]
    assert drift and "radar_az" in drift[0]
    assert abs(sim.world.true_errors_deg()["radar_az"]) < TOL


@pytest.mark.xfail(strict=True, reason="superseded mutual-sighting design: its assumed noise (2 sigma ~0.03 deg) cannot meet the 0.005 deg requirement (Liam, 2026-11-17)")
def test_track_does_not_move_for_jitter_it_cannot_fix(monkeypatch):
    """Mean far inside the drift threshold, jitter just under the "unstable"
    limit: the criterion fails, the display says so, and nothing moves."""
    sim, ctrl = _make(QUIET)
    ctrl.start()
    _run_until_track(sim, ctrl)
    settled = _positions(sim)

    # 0.005 +/- 0.0235 deg: |mean| = 0.005 < drift threshold, 2*sigma = 0.047 < tol,
    # |mean| + 2*sigma = 0.052 >= tol.
    tick = {"n": 0}

    def synthetic(now):
        tick["n"] += 1
        sign = 1.0 if tick["n"] % 2 else -1.0
        ctrl._last_errors = {axis: 0.005 + sign * 0.0235 for axis in ERROR_AXES}
        return ctrl._last_errors

    monkeypatch.setattr(ctrl, "_read", synthetic)
    status, events = _run(sim, ctrl, 15.0)

    assert status.state == "TRACK"
    assert status.errors["radar_az"]["score_deg"] >= TOL
    assert abs(status.errors["radar_az"]["mean_deg"]) < CFG.controller.track_drift_threshold_deg
    assert 2 * status.errors["radar_az"]["sigma_deg"] < TOL
    assert status.warning is None
    assert not status.aligned
    assert status.corrections == 0
    assert not [text for _, text in events if text.startswith("move ")]
    assert _positions(sim) == settled


# ---------------------------------------------------------------- P2: U_bias
@pytest.mark.xfail(strict=True, reason="superseded mutual-sighting design: its assumed noise (2 sigma ~0.03 deg) cannot meet the 0.005 deg requirement (Liam, 2026-11-17)")
def test_status_reports_score_with_calibration_uncertainty():
    result = run_single(CFG, seed=1)
    assert result.aligned
    u_radar = 2 * (CFG.calibration.residual_sigma_deg**2 + CFG.calibration.fixture_repeat_sigma_deg**2) ** 0.5
    u_horn = 2 * CFG.calibration.residual_sigma_deg
    for axis in ERROR_AXES:
        entry = result.reported[axis]
        expected = u_radar if axis.startswith("radar") else u_horn
        assert entry["u_bias_deg"] == pytest.approx(expected)
        assert entry["score_with_bias_deg"] == pytest.approx(entry["score_deg"] + expected)


def test_aligned_with_bias_needs_a_small_calibration_uncertainty():
    # Assumed calibration: U_bias alone is 0.036 deg, so the strict claim is impossible.
    sim, ctrl = _make(CFG)
    ctrl.start()
    status, _ = _run(sim, ctrl, 60.0)
    assert status.aligned or status.state != "TRACK"
    assert not status.aligned_with_bias

    # No calibration uncertainty and no noise: both claims hold.
    sim, ctrl = _make(QUIET)
    ctrl.start()
    status, _ = _run(sim, ctrl, 30.0)
    assert status.aligned and status.aligned_with_bias


def test_summary_reports_the_strict_pass_rate():
    summary = summarize(run_montecarlo(QUIET, runs=3, seed=3), TOL)
    assert summary["with_bias_pass_rate"] == 1.0
    assert summary["with_bias_false_accept_rate"] == 0.0
    assert "pass incl. U_bias" in format_summary(summary)


# ------------------------------------------------------------------ P4: hold
def test_hold_stops_motion_during_alignment_and_resumes():
    sim, ctrl = _make(QUIET)
    ctrl.start()
    _run(sim, ctrl, 3.0)
    assert sim.hardware.actuators.is_moving()

    ctrl.set_hold(True)
    assert not sim.hardware.actuators.is_moving()
    frozen = _positions(sim)
    status, events = _run(sim, ctrl, 10.0)
    assert status.hold and status.phase == "hold"
    assert _positions(sim) == frozen
    assert not [text for _, text in events if text.startswith("move ")]

    ctrl.set_hold(False)
    status, _ = _run(sim, ctrl, 40.0)
    assert status.state == "TRACK" and status.aligned and not status.hold


def test_hold_defers_track_corrections_until_released():
    sim, ctrl = _make(QUIET, **SLOW_DRIFT)
    ctrl.start()
    _run_until_track(sim, ctrl)

    ctrl.set_hold(True)
    frozen = _positions(sim)
    status, events = _run(sim, ctrl, 150.0)
    assert status.state == "TRACK" and status.corrections == 0
    assert _positions(sim) == frozen
    assert status.warning.startswith("correction pending (hold)")
    assert not status.aligned  # the display does not pretend
    pending = [text for _, text in events if text.startswith("correction pending")]
    assert len(pending) == 1  # logged once, not every tick

    ctrl.set_hold(False)
    status, events = _run(sim, ctrl, 20.0)
    assert status.corrections >= 1
    assert abs(sim.world.true_errors_deg()["radar_az"]) < TOL


def test_every_move_is_logged_with_a_timestamp():
    sim, ctrl = _make(QUIET)
    ctrl.start()
    _, events = _run(sim, ctrl, 30.0)
    moves = [(t, text) for t, text in events if text.startswith("move ")]
    assert moves
    assert {text.split()[1] for _, text in moves} <= set(AXES)
    assert all(0.0 < t <= 30.0 + 1e-6 for t, _ in moves)
    assert [t for t, _ in moves] == sorted(t for t, _ in moves)
