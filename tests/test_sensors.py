import dataclasses
import math

import numpy as np
import pytest

from rada_align import geometry as g
from rada_align.config import load_config, with_only_sources
from rada_align.disturbances import Backlash, OUProcess, WindModel, quantize
from rada_align.interfaces import ActuatorError
from rada_align.sensors import SimActuators, build_simulation, intrinsics_from_config

CFG = load_config()
QUIET = with_only_sources(CFG, ())
ALIGNED = dict(
    radar_az_deg=0.0, radar_el_deg=0.0, radar_roll_deg=0.0,
    horn_az_deg=0.0, horn_el_deg=0.0, horn_roll_deg=0.0,
    radar_delta_deg=(0.0, 0.0), horn_delta_deg=(0.0, 0.0),
)


def test_ou_process_has_requested_sigma_and_correlation():
    rng = np.random.default_rng(1)
    process = OUProcess(sigma=2.0, tau_s=1.0, rng=rng)
    samples = np.array([process.step(0.1) for _ in range(200_000)])
    assert samples.std() == pytest.approx(2.0, rel=0.03)
    lag = 10  # one correlation time
    corr = np.corrcoef(samples[:-lag], samples[lag:])[0, 1]
    assert corr == pytest.approx(math.exp(-1.0), abs=0.03)


def test_wind_model_combines_jitter_and_sway():
    wind = WindModel(0.3, 0.4, 2.0, np.random.default_rng(2))
    samples = np.array([wind.step(0.1) for _ in range(100_000)])
    assert samples.std(axis=0) == pytest.approx([0.5, 0.5], rel=0.08)
    quiet = WindModel(0.0, 0.0, 2.0, np.random.default_rng(2))
    assert quiet.step(0.1) == (0.0, 0.0)


def test_quantize():
    assert quantize(0.0101, 0.0025) == pytest.approx(0.01)
    assert quantize(0.0101, 0.0) == 0.0101


def test_backlash_swallows_small_reversals():
    lash = Backlash(width=0.02)
    assert lash.update(1.0) == pytest.approx(0.99)  # play taken up going forward
    assert lash.update(0.985) == pytest.approx(0.99)  # reversal inside the play: no motion
    assert lash.update(0.9) == pytest.approx(0.91)


def test_actuators_speed_quantization_and_limits():
    act = SimActuators(CFG.actuators, np.random.default_rng(3))
    act.move_relative("x", 10.0011)
    assert act.is_moving()
    act.advance(0.1)
    assert act.position("x") == pytest.approx(2.0)  # 20 mm/s * 0.1 s
    for _ in range(10):
        act.advance(0.1)
    assert not act.is_moving()
    assert act.position("x") == pytest.approx(10.0)  # rounded to the 0.0025 mm step
    assert abs(act.mechanical("x") - act.position("x")) <= CFG.actuators.x.backlash / 2 + 1e-12

    assert act.limits("x") == (-250.0, 250.0)
    with pytest.raises(ActuatorError, match="soft limit"):
        act.move_relative("x", 300.0)
    with pytest.raises(ActuatorError, match="unknown axis"):
        act.move_relative("yaw", 1.0)

    act.move_relative("pan", 5.0)
    act.advance(0.1)
    act.stop()
    assert not act.is_moving()
    assert act.position("pan") == pytest.approx(0.2)


def test_world_true_errors_have_the_documented_signs():
    sim = build_simulation(QUIET, seed=0, **{**ALIGNED, "radar_az_deg": 0.5, "radar_el_deg": 0.2})
    err = sim.world.true_errors_deg()
    # Radar looks right and up of the horn -> horn appears left and below.
    # Tangent-plane angles: az picks up a 1/cos(el) factor (3e-6 deg here).
    assert err["radar_az"] == pytest.approx(-0.5, abs=1e-4)
    assert err["radar_el"] == pytest.approx(-0.2, abs=1e-4)
    assert err["horn_az"] == pytest.approx(0.0, abs=1e-9)

    # +x moves the horn to its right = the radar's left -> radar az more negative.
    sim.hardware.actuators.move_relative("x", 100.0)
    sim.hardware.actuators.move_relative("pan", 1.0)
    for _ in range(100):
        sim.world.advance(0.1)
    after = sim.world.true_errors_deg()
    assert after["radar_az"] < err["radar_az"] - 0.4
    # Horn turned right by 1 deg and moved right: the radar is now to its left.
    assert after["horn_az"] < -1.0


def test_camera_pixel_matches_truth_and_parallax():
    intr = intrinsics_from_config(QUIET.camera)

    def view(cfg):
        sim = build_simulation(cfg, seed=0, **ALIGNED)
        m = sim.hardware.radar_camera.measure_beacon()
        assert m.valid
        az, el = g.pixel_to_angles_rad(m.u_px, m.v_px, intr)
        return math.degrees(az), math.degrees(el), sim.world.true_distance_m()

    # Unmatched: camera 10 cm right of the radar axis, horn beacon 10 cm above
    # the horn center -> the full parallax angle shows up in the pixels.
    unmatched = dataclasses.replace(QUIET.horn_head, beacon_offset_m=(0.0, 0.10))
    az, el, d = view(dataclasses.replace(QUIET, horn_head=unmatched))
    assert az == pytest.approx(math.degrees(math.atan2(-0.10, d)), abs=1e-6)
    assert el == pytest.approx(math.degrees(math.atan2(0.10, d)), abs=1e-6)

    # Rig rule in config.yaml: the beacon mirrors the opposite camera -> no parallax.
    az, el, _ = view(QUIET)
    assert az == pytest.approx(0.0, abs=1e-9)
    assert el == pytest.approx(0.0, abs=1e-9)


def test_camera_reports_invalid_outside_field_of_view():
    sim = build_simulation(QUIET, seed=0, **{**ALIGNED, "horn_az_deg": 9.0})
    assert not sim.hardware.horn_camera.measure_beacon().valid
    assert sim.hardware.radar_camera.measure_beacon().valid


def test_inclinometer_and_rangefinder_read_truth_without_noise():
    sim = build_simulation(QUIET, seed=0, **{**ALIGNED, "radar_roll_deg": 0.4, "radar_el_deg": 0.3})
    tilt = sim.hardware.radar_inclinometer.read_tilt()
    assert tilt.roll_deg == pytest.approx(0.4, abs=1e-9)
    assert tilt.pitch_deg == pytest.approx(0.3, abs=1e-9)
    assert sim.hardware.rangefinder.read_distance_m() == pytest.approx(
        QUIET.geometry.distance_m - 0.10, abs=1e-9  # aperture center is 10 cm ahead of the pivot
    )


def test_noise_sources_have_the_configured_size():
    sim = build_simulation(CFG, seed=5, **ALIGNED)
    truth = sim.world.true_distance_m()
    ranges = np.array([sim.hardware.rangefinder.read_distance_m() for _ in range(2000)])
    assert ranges.std() == pytest.approx(CFG.rangefinder.noise_m, rel=0.1)
    assert abs(ranges.mean() - truth) < 4 * CFG.rangefinder.bias_sigma_m

    quiet_wind = with_only_sources(CFG, ("centroid",))
    sim = build_simulation(quiet_wind, seed=5, **ALIGNED)
    us = np.array([sim.hardware.radar_camera.measure_beacon().u_px for _ in range(2000)])
    assert us.std() == pytest.approx(CFG.camera.centroid_noise_px, rel=0.1)


def test_same_seed_gives_same_setup_when_sources_are_switched_off():
    full = build_simulation(CFG, seed=7).scenario
    only_wind = build_simulation(with_only_sources(CFG, ("wind",)), seed=7).scenario
    assert only_wind.radar_az_deg == full.radar_az_deg
    assert only_wind.horn_el_deg == full.horn_el_deg
    assert only_wind.radar_drift_deg_per_s == (0.0, 0.0)
