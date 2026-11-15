import cv2
import numpy as np
import pytest

from rada_align.config import load_config
from rada_align.geometry import Intrinsics, intrinsics_from_config
from rada_align.vision.camera import SyntheticSource, VisionCamera
from rada_align.vision.detect import render_spot
from rada_align.vision.intrinsics import (
    LensCalibration,
    calibrate_from_corners,
    chessboard_object_points,
    load_lens_calibration,
    save_lens_calibration,
    to_nominal,
)

CFG = load_config()
NOMINAL = intrinsics_from_config(CFG.camera)


class ListSource:
    def __init__(self, frames):
        self.frames = list(frames)
        self.closed = False

    def read(self):
        return self.frames.pop(0) if self.frames else None

    def close(self):
        self.closed = True


def test_camera_reports_centroid_sigma_and_validity():
    camera = VisionCamera(SyntheticSource(u=320.3, v=240.7, noise=2.0, seed=5), CFG.vision, NOMINAL)
    ms = [camera.measure_beacon() for _ in range(40)]
    assert all(m.valid for m in ms)
    assert ms[0].sigma_px == 0.0  # no scatter estimate from one sample
    assert np.mean([m.u_px for m in ms]) == pytest.approx(320.3, abs=0.02)
    assert np.mean([m.v_px for m in ms]) == pytest.approx(240.7, abs=0.02)
    assert 0.003 < ms[-1].sigma_px < 0.05
    assert ms[1].timestamp_s >= ms[0].timestamp_s
    camera.close()


def test_camera_tracks_in_a_window_and_recovers_after_losing_the_beacon():
    shape = (480, 640)
    blank = render_spot(shape, 0, 0, amplitude=0.0)
    far = render_spot(shape, 500.0, 100.0)  # outside the 80 px window around the first spot
    frames = [render_spot(shape, 200.0, 300.0)] * 2 + [far] * 3 + [far] + [blank]
    camera = VisionCamera(ListSource(frames), CFG.vision, NOMINAL)
    assert camera.measure_beacon().valid and camera.measure_beacon().valid
    # The beacon jumped out of the tracking window: three misses, then full frame.
    assert [camera.measure_beacon().valid for _ in range(3)] == [False, False, False]
    m = camera.measure_beacon()
    assert m.valid and m.u_px == pytest.approx(500.0, abs=0.02)
    assert not camera.measure_beacon().valid and camera.last_detection.reason == "not_found"
    assert not camera.measure_beacon().valid and camera.last_detection.reason == "no_frame"


def test_frame_differencing_switches_the_led_and_needs_a_callback():
    vision = CFG.replace_path("vision.frame_differencing", 1).vision
    with pytest.raises(ValueError, match="led"):
        VisionCamera(SyntheticSource(), vision, NOMINAL)

    source = SyntheticSource(u=100.4, v=200.6, noise=2.0, seed=6, background=120.0, amplitude=100.0)
    calls = []

    def led(on):
        calls.append(on)
        source.led_on = on

    camera = VisionCamera(source, vision, NOMINAL, led=led)
    m = camera.measure_beacon()
    assert calls == [False, True]
    assert m.valid and m.u_px == pytest.approx(100.4, abs=0.1) and m.v_px == pytest.approx(200.6, abs=0.1)


def test_pixels_are_mapped_to_the_nominal_camera():
    """A real lens (own focal lengths, principal point, distortion) must give
    the pixel the ideal camera of config.yaml would have seen."""
    matrix = np.array([[1500.0, 0.0, 650.0], [0.0, 1480.0, 350.0], [0.0, 0.0, 1.0]])
    dist = np.array([-0.25, 0.08, 0.001, -0.0005, 0.0])
    lens = LensCalibration(matrix, dist, (1280, 720), 0.1)
    nominal = Intrinsics(f_px=16129.0, cx_px=2028.0, cy_px=1520.0, width_px=4056, height_px=3040)
    for x, y in ((0.0, 0.0), (0.15, -0.08), (-0.2, 0.12)):  # tangent-plane direction
        point = np.array([[x, y, 1.0]])
        (u, v), = cv2.projectPoints(point, np.zeros(3), np.zeros(3), matrix, dist)[0].reshape(-1, 2)
        nu, nv = to_nominal(u, v, lens, nominal)
        assert nu == pytest.approx(nominal.cx_px + nominal.f_px * x, abs=0.05)
        assert nv == pytest.approx(nominal.cy_px + nominal.f_px * y, abs=0.05)
    assert to_nominal(12.5, 7.5, None, nominal) == (12.5, 7.5)


def test_chessboard_calibration_recovers_the_lens_and_roundtrips(tmp_path):
    matrix = np.array([[1400.0, 0.0, 640.0], [0.0, 1400.0, 360.0], [0.0, 0.0, 1.0]])
    dist = np.array([-0.2, 0.05, 0.0, 0.0, 0.0])
    grid = chessboard_object_points((9, 6), 25.0)
    assert grid.shape == (54, 3) and grid[1, 0] == 25.0
    views, rng = [], np.random.default_rng(3)
    for _ in range(12):
        rvec = rng.uniform(-0.4, 0.4, 3)
        tvec = np.array([rng.uniform(-150, 50), rng.uniform(-100, 20), rng.uniform(500, 900)])
        views.append(cv2.projectPoints(grid, rvec, tvec, matrix, dist)[0].reshape(-1, 2))
    lens = calibrate_from_corners([grid] * len(views), views, (1280, 720))
    assert lens.rms_px < 0.05
    assert lens.camera_matrix[0, 0] == pytest.approx(1400.0, rel=0.01)
    assert lens.camera_matrix[0, 2] == pytest.approx(640.0, abs=5.0)
    assert lens.dist_coeffs[0] == pytest.approx(-0.2, abs=0.02)

    path = tmp_path / "calib" / "camera.yaml"
    save_lens_calibration(path, lens, {"note": "test"})
    back = load_lens_calibration(path)
    assert np.allclose(back.camera_matrix, lens.camera_matrix) and back.image_size == (1280, 720)
