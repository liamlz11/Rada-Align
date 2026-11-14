import pytest

from rada_align.config import load_config, with_only_sources
from rada_align.estimator import ERROR_AXES, RunningMean, WindowEstimator, compute_errors_deg
from rada_align.geometry import intrinsics_from_config
from rada_align.sensors import build_simulation

QUIET = with_only_sources(load_config(), ())


def test_window_statistics_and_pass_score():
    est = WindowEstimator()
    for i, value in enumerate([0.01, 0.03, 0.02, 0.02]):
        est.add({"a": value, "b": -value}, t_s=i * 0.1)
    stats = est.stats()
    assert stats["a"].n == 4
    assert stats["a"].mean == pytest.approx(0.02)
    assert stats["a"].sigma == pytest.approx(0.0081650, abs=1e-6)  # ddof = 1
    assert stats["b"].score == pytest.approx(0.02 + 2 * 0.0081650, abs=1e-6)
    est.reset()
    assert est.n == 0 and est.stats() == {}


def test_rolling_window_drops_old_samples():
    est = WindowEstimator(window_s=1.0)
    for i in range(30):
        est.add({"a": float(i)}, t_s=i * 0.1)
    assert est.n == 11  # samples from t = 1.9 .. 2.9
    assert est.span_s == pytest.approx(1.0)
    assert est.stats()["a"].mean == pytest.approx(24.0)


def test_running_mean():
    mean = RunningMean()
    assert mean.mean is None
    mean.add(1.0)
    mean.add(2.0)
    assert mean.mean == pytest.approx(1.5)


def _measure(sim):
    hw = sim.hardware
    return compute_errors_deg(
        hw.radar_camera.measure_beacon(),
        hw.horn_camera.measure_beacon(),
        hw.radar_inclinometer.read_tilt().roll_deg,
        hw.horn_inclinometer.read_tilt().roll_deg,
        hw.rangefinder.read_distance_m(),
        sim.calibration,
        intrinsics_from_config(QUIET.camera),
    )


@pytest.mark.parametrize("distance_m, model_residual_deg", [(12.0, 0.001), (2.0, 0.01)])
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_noise_free_measurement_matches_truth(distance_m, model_residual_deg, seed):
    """Random +/-1 deg setup with roll, camera mount offsets and parallax.

    Away from the null a second-order term remains: the other head is tilted,
    so its beacon sits a few mm nearer or farther than its reference point
    (net offset * depth shift / d^2). Worst case over 200 setups: 0.0005 deg
    at 12 m, 0.0077 deg at 2 m. It vanishes at the null (next test).
    """
    sim = build_simulation(QUIET, seed, distance_m)
    measured, truth = _measure(sim), sim.world.true_errors_deg()
    assert max(abs(truth[a]) for a in ERROR_AXES) > 0.1  # a real misalignment
    for axis in ERROR_AXES:
        assert measured[axis] == pytest.approx(truth[axis], abs=model_residual_deg)


@pytest.mark.parametrize("distance_m", [2.0, 12.0])
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_noise_free_measurement_is_exact_at_the_null(distance_m, seed):
    """Aligned heads, still with random roll and camera mount offsets."""
    sim = build_simulation(
        QUIET, seed, distance_m,
        radar_az_deg=0.0, radar_el_deg=0.0, horn_az_deg=0.0, horn_el_deg=0.0,
    )
    measured, truth = _measure(sim), sim.world.true_errors_deg()
    for axis in ERROR_AXES:
        assert measured[axis] == pytest.approx(truth[axis], abs=1e-9)
