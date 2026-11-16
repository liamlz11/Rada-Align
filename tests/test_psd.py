"""Final design (Raster Scan + PSD + KLT): model, sensors, controller, demo server."""

import math

import numpy as np
import pytest
from fastapi.testclient import TestClient

from rada_align.psd.config import load_psd_config, quiet
from rada_align.psd.controller import PsdController
from rada_align.psd.model import DEG, Geometry
from rada_align.psd.server import create_app
from rada_align.psd.session import PsdSession
from rada_align.psd.world import World

CFG = load_psd_config()
QUIET = quiet(CFG)


def run(world, ctrl, seconds, stop_state=None):
    end = world.t + seconds
    while world.t < end:
        ctrl.tick()
        world.step(ctrl.dt)
        if stop_state and ctrl.state == stop_state:
            break
    return ctrl.state


@pytest.mark.parametrize("d", [2.0, 12.0])
def test_readings_zero_when_aligned_and_rank_5(d):
    geo = Geometry(CFG, d)
    assert np.abs(geo.readings_vector(np.zeros(6), np.zeros(4))).max() < 1e-9
    lin = geo.jacobians()
    sv = np.linalg.svd(lin["Jm"], compute_uv=False)
    assert int((sv > sv[0] * 1e-9).sum()) == 5
    X, *_ = np.linalg.lstsq(lin["Jm"].T, lin["Je"].T, rcond=None)
    assert np.abs(lin["Jm"].T @ X - lin["Je"].T).max() < 1e-6  # all 5 errors observable


def test_lasers_on_radar_a_reads_radar_error_b_ignores_translation():
    geo = Geometry(CFG, 12.0)
    assert geo.emitter == "radar"
    shifted = geo.beams(np.zeros(6), np.array([0.005, 0.0, 0.0, 0.0]))
    assert np.abs(shifted["B"]).max() < 1e-9  # a sideways shift does not change the beam angle
    assert abs(shifted["A"][0]) > 4.0  # 5 mm sideways shows on A
    G = geo.jacobians()["G"]
    assert G[0, 2] == pytest.approx((geo.d - 0.2) * 1e3 * DEG, rel=1e-3)  # A: radar error, 11.8 m path
    assert abs(G[0, 0]) < 1e-6  # A does not see the horn error
    assert G[2, 0] == pytest.approx(-G[2, 2])  # B = f (horn - radar)


def test_spot_off_the_psd_is_invalid_and_partial_is_detected_only():
    w = World(QUIET, 0, 12.0)
    w.base[:] = 0
    w.trans[:] = 0
    half = w.ac_size_mm / 2
    mm_per_deg = (w.geo.d - 0.2) * 1e3 * DEG
    w.base[3] = (half - 0.5 * w.geo.spot_r99_mm) / mm_per_deg  # radar pan: spot straddles the edge
    s = w.read_psds()["A"]
    assert s.detect and not s.valid
    w.base[3] = 1.0  # far off
    s = w.read_psds()["A"]
    assert not s.detect and not s.valid


@pytest.mark.parametrize("d,seed,emitter", [(2.0, 0, "radar"), (12.0, 1, "radar"), (15.0, 2, "radar"), (12.0, 1, "horn")])
def test_zero_noise_run_reaches_track_with_small_true_error(d, seed, emitter):
    w = World(QUIET.replace_path("geometry.emitter", emitter), seed, d)
    c = PsdController(w)
    c.start()
    assert run(w, c, 60, "TRACK") == "TRACK"
    assert np.abs(w.true_errors()[:4]).max() < 0.005
    assert c.after["truth_pass"]


def test_radar_is_braked_in_track_and_moves_only_in_a_gap():
    w = World(QUIET.replace_path("wind.drift_deg_per_min", 0.0), 3, 12.0)
    c = PsdController(w)
    c.start()
    run(w, c, 60, "TRACK")
    radar_before = [w.servos[i].pos for i in (3, 4, 5)]
    w.push_side("radar", 0.08)
    run(w, c, 8)
    assert [w.servos[i].pos for i in (3, 4, 5)] == radar_before
    assert c.alerts  # radar drift, spot leaving, or beam lost: radar is braked, so only an alert
    c.radar_gap = True
    run(w, c, 15)
    assert [w.servos[i].pos for i in (3, 4, 5)] != radar_before
    assert np.abs(w.true_errors()[2:4]).max() < 0.03


def test_short_block_is_bridged_long_loss_waits_for_gap_then_rescans():
    w = World(QUIET, 4, 12.0)
    c = PsdController(w)
    c.start()
    run(w, c, 60, "TRACK")
    w.block_beam(0.5)
    run(w, c, 0.3)
    assert c.state == "TRACK" and c.klt_bridge
    run(w, c, 1.0)
    assert c.state == "TRACK" and not c.klt_bridge
    w.block_beam(3.0)
    run(w, c, 1.5)
    assert c.state == "TRACK" and any(a.startswith("beam lost") for a in c.alerts)  # radar braked: wait
    c.radar_gap = True
    run(w, c, 0.2)
    assert c.state == "SCAN" and not w.servos[3].braked


def test_missing_calibration_faults():
    w = World(QUIET, 0, 12.0, calibration_loaded=False)
    c = PsdController(w)
    c.start()
    run(w, c, 1.0)
    assert c.state == "FAULT" and "calibration" in c.fault_reason


def test_scan_finds_the_beam_from_a_camera_bias():
    cfg = QUIET.replace_path("camera.acquire_sigma_deg", 0.05)
    w = World(cfg, 5, 12.0)
    w.camera_bias[:] = [0.06, -0.05]  # spot well outside the PSD after ACQUIRE
    c = PsdController(w)
    c.start()
    assert run(w, c, 60, "TRACK") == "TRACK"
    assert any("beam captured" in e[1] for e in c.events)


def test_demo_server_streams_and_accepts_commands():
    session = PsdSession(QUIET, seed=1, distance_m=2.0)
    with TestClient(create_app(session, autostart=False)) as client:
        assert "three" in client.get("/").text.lower()
        assert client.post("/api/command", json={"name": "start"}).json()["ok"]
        msg = None
        for _ in range(int(20 / session.dt)):
            msg = session.step_once() or msg
        assert msg["sim"] is True and msg["state"] in msg["states"]
        assert set(msg["psd"]) == {"A", "B", "C"}
        assert client.post("/api/command", json={"name": "bogus"}).status_code == 400


def test_u_bias_matches_sampled_calibration_residuals():
    """U_bias (k=2) from linear propagation equals 2x the spread of the estimate
    error caused by calibration residuals drawn the way the world draws them."""
    from rada_align.psd.estimator import PsdEstimator

    cfg = quiet(CFG, ("laser", "calib"))
    est = PsdEstimator(Geometry(cfg, 12.0))
    use = ("A", "B", "C")
    u = est.u_bias_deg(use)
    z6, z4 = np.zeros(6), np.zeros(4)
    r0 = est.geo.readings_vector(z6, z4)
    errs = []
    for seed in range(400):
        w = World(cfg, seed, 12.0)
        errs.append(est.estimator4_for(use) @ (est.geo.readings_vector(z6, z4, w.bias) - r0))  # roll from the inclinometers
    spread = 2 * np.std(errs, axis=0)
    assert np.allclose(spread, u, rtol=0.15)
    assert np.all(PsdEstimator(Geometry(QUIET, 12.0)).u_bias_deg(use) == 0)
