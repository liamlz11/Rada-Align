import json

import pytest

from rada_align.config import load_config, with_only_sources
from rada_align.controller import Controller, State, _spiral
from rada_align.estimator import ERROR_AXES
from rada_align.runner import run_single
from rada_align.sensors import build_simulation

CFG = load_config()
QUIET = with_only_sources(CFG, ())
TOL = CFG.geometry.tolerance_deg


@pytest.mark.parametrize("distance_m", [2.0, 12.0])
def test_zero_noise_run_converges(distance_m):
    result = run_single(QUIET, seed=1, distance_m=distance_m)
    assert result.fault_reason is None
    assert result.aligned and result.final_state == State.TRACK.value
    assert max(abs(v) for v in result.before.values()) > 0.1  # started misaligned
    for axis in ERROR_AXES:
        assert result.truth_verify[axis]["score_deg"] < TOL
        assert result.truth_track[axis]["score_deg"] < TOL
        assert abs(result.reported[axis]["mean_deg"] - result.truth_verify[axis]["mean_deg"]) < 1e-3


def test_extreme_wind_ends_in_fault_with_reason():
    windy = CFG.replace_path("disturbances.wind_jitter_deg", 10 * CFG.disturbances.wind_jitter_deg)
    windy = windy.replace_path("disturbances.wind_sway_deg", 10 * CFG.disturbances.wind_sway_deg)
    result = run_single(windy, seed=1)
    assert result.final_state == State.FAULT.value
    assert not result.aligned
    assert result.fault_reason.startswith("unstable: 2*sigma")
    assert "pass criterion impossible" in result.fault_reason


def test_travel_limit_faults_instead_of_moving_out_of_range():
    # 1 deg at 15 m needs 262 mm, the X slide has +/-250 mm.
    result = run_single(QUIET, seed=1, distance_m=15.0, radar_az_deg=1.0)
    assert result.final_state == State.FAULT.value
    assert result.fault_reason.startswith("x travel limit")


def test_spiral_search_finds_a_beacon_outside_the_field_of_view():
    sim = build_simulation(QUIET, 1, horn_az_deg=9.0)
    assert not sim.hardware.horn_camera.measure_beacon().valid
    result = run_single(QUIET, seed=1, horn_az_deg=9.0)
    assert result.aligned and result.fault_reason is None


def test_spiral_covers_rings_in_order():
    points = list(_spiral(5.0, 10.0))
    assert len(points) == 8 + 16
    assert all(max(abs(p), abs(t)) == 5.0 for p, t in points[:8])
    assert len(set(points)) == len(points)


def test_beacon_never_visible_faults_after_timeout():
    result = run_single(QUIET, seed=1, radar_az_deg=20.0)
    assert result.final_state == State.FAULT.value
    assert result.fault_reason.startswith("beacon not found")
    assert "radar camera" in result.fault_reason
    assert result.duration_s == pytest.approx(QUIET.controller.acquire_timeout_s, abs=1.0)


@pytest.mark.xfail(strict=True, reason="superseded mutual-sighting design: its assumed noise (2 sigma ~0.03 deg) cannot meet the 0.005 deg requirement (Liam, 2026-11-17)")
def test_track_corrects_drift():
    # Radar tripod settling at 0.12 deg/min: crosses the 0.03 deg drift threshold.
    result = run_single(
        QUIET, seed=1, track_time_s=90.0, radar_drift_deg_per_s=(0.002, 0.0)
    )
    assert result.aligned
    assert result.corrections >= 1
    assert result.final_state != State.FAULT.value


def test_status_is_json_serializable_and_stop_returns_to_idle():
    sim = build_simulation(QUIET, 1)
    ctrl = Controller(sim.hardware, QUIET, sim.calibration, clock=lambda: sim.world.t)
    assert ctrl.step().state == "IDLE"
    ctrl.start()
    for _ in range(30):
        sim.world.advance(0.1)
        status = ctrl.step()
    text = json.dumps(status.to_dict())
    assert json.loads(text)["state"] in ("ACQUIRE", "ALIGN")
    assert set(status.errors) == set(ERROR_AXES)
    ctrl.stop()
    assert ctrl.step().state == "IDLE"
    assert not sim.hardware.actuators.is_moving()
