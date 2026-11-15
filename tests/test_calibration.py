import math

import numpy as np
import pytest

from rada_align.calibration import (
    load_calibration,
    mirror_apparent_angle_deg,
    reversal_from_readings,
    reversal_study,
    save_calibration,
    simulate_reversal,
)
from rada_align.config import CalibrationMissing, load_config, with_only_sources
from rada_align.controller import Controller
from rada_align.sensors import build_simulation

CFG = load_config()
QUIET = with_only_sources(CFG, ())


def test_reversal_maths_separates_offset_from_target_direction():
    # delta = (+0.30, -0.20), target at (+0.50, +0.10) in the datum frame.
    normal = np.tile([0.50 + 0.30, 0.10 - 0.20], (40, 1))
    flipped = np.tile([-0.50 + 0.30, -0.10 - 0.20], (40, 1))
    r = reversal_from_readings(normal, flipped, seat_sigma_deg=0.0)
    assert (r.delta_az_deg, r.delta_el_deg) == pytest.approx((0.30, -0.20))
    assert (r.axis_az_deg, r.axis_el_deg) == pytest.approx((0.50, 0.10))
    assert r.u_delta_deg == pytest.approx(0.0, abs=1e-12)
    # The seat term is added even when the readings are perfect.
    seat = reversal_from_readings(normal, flipped, seat_sigma_deg=0.010)
    assert seat.u_delta_deg == pytest.approx(0.010 / math.sqrt(2))


@pytest.mark.parametrize("distance_m", [2.0, 12.0])
def test_noise_free_reversal_recovers_the_injected_delta(distance_m):
    result, truth = simulate_reversal(QUIET, seed=4, distance_m=distance_m, seconds_per_position=2.0, seat_sigma_deg=0.0)
    assert max(abs(d) for d in truth["delta_deg"]) > 0.05  # a real offset was injected
    assert result.delta_az_deg == pytest.approx(truth["delta_deg"][0], abs=0.001)
    assert result.delta_el_deg == pytest.approx(truth["delta_deg"][1], abs=0.001)


def test_seat_error_is_not_removed_by_the_reversal():
    result, truth = simulate_reversal(QUIET, seed=4, distance_m=12.0, seconds_per_position=2.0, seat_sigma_deg=0.010)
    expected = -(np.array(truth["seat_normal_deg"]) + np.array(truth["seat_flipped_deg"])) / 2.0
    error = np.array([result.delta_az_deg, result.delta_el_deg]) - np.array(truth["operating_delta_deg"])
    assert error == pytest.approx(expected, abs=0.001)
    assert result.u_delta_deg >= 0.010 / math.sqrt(2)


def test_simulated_reversal_error_matches_the_predicted_sigma():
    """All noise on, 60 s per position, 12 m: the achieved RMS error must agree
    with the uncertainty the procedure reports (batch means are known to be
    slightly optimistic, hence the 30% allowance, fixed before this test ran)."""
    study = reversal_study(CFG, runs=30, seed=0, distance_m=12.0, seconds_per_position=60.0, seat_sigma_deg=0.010)
    rms = max(study["rms_error_az_deg"], study["rms_error_el_deg"])
    assert rms < 1.3 * study["predicted_u_delta_deg"]
    assert rms > 0.5 * study["predicted_u_delta_deg"]
    assert abs(study["mean_error_az_deg"]) < 0.005 and abs(study["mean_error_el_deg"]) < 0.005


def test_flipped_position_needs_good_range_at_short_distance():
    """Flipped, the camera is on the other side of the axis: the matched-offset
    cancellation is gone and a 5 cm range error dominates at 2 m."""
    coarse = reversal_study(CFG, runs=20, seed=0, distance_m=2.0, seconds_per_position=10.0, seat_sigma_deg=0.0)
    tape = CFG.replace_path("rangefinder.bias_sigma_m", 0.002).replace_path("rangefinder.noise_m", 0.0)
    fine = reversal_study(tape, runs=20, seed=0, distance_m=2.0, seconds_per_position=10.0, seat_sigma_deg=0.0)
    assert coarse["rms_error_az_deg"] > 0.03
    assert fine["rms_error_az_deg"] < 0.01


def test_calibration_file_roundtrip_and_refusal(tmp_path):
    calibration = build_simulation(CFG, 1).calibration
    path = tmp_path / "calib" / "datum.yaml"
    save_calibration(path, calibration, meta={"method": "test"})
    assert load_calibration(path) == calibration

    with pytest.raises(CalibrationMissing, match="not found"):
        load_calibration(tmp_path / "nope.yaml")
    path.write_text("radar: {delta_az_deg: 0.1}\n", encoding="utf-8")
    with pytest.raises(CalibrationMissing, match="not valid"):
        load_calibration(path)


def test_controller_refuses_to_start_without_calibration():
    sim = build_simulation(CFG, 1)
    with pytest.raises(CalibrationMissing, match="refusing to start"):
        Controller(sim.hardware, CFG, None, clock=lambda: sim.world.t)


@pytest.mark.parametrize("distance_m", [2.0, 12.0])
def test_flange_mirror_shows_the_tilt_once_not_twice(distance_m):
    """Beacon next to the camera (not a collimated beam): a mirror tilt of
    theta moves the reflection by theta."""
    zero = mirror_apparent_angle_deg(0.0, distance_m, camera_x_m=0.10, beacon_x_m=-0.10)
    shift = mirror_apparent_angle_deg(0.05, distance_m, camera_x_m=0.10, beacon_x_m=-0.10) - zero
    assert abs(shift) == pytest.approx(0.05, rel=0.01)
