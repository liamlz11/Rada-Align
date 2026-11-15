"""Synthetic-image tests. Noise level and the 0.05 px tolerance were fixed
before the first run (docs/TASKS.md task 4)."""

import numpy as np
import pytest

from rada_align.config import load_config
from rada_align.vision.detect import DetectParams, detect_beacon, render_spot

PARAMS = DetectParams.from_config(load_config().vision)
SHAPE = (240, 320)
NOISE = 2.0  # grey levels rms, on a spot of amplitude 150 over background 20


@pytest.mark.parametrize("u, v", [(160.0, 120.0), (160.5, 120.5), (101.23, 77.81), (250.37, 30.06), (40.9, 200.4)])
def test_gaussian_spot_recovered_within_0_05_px_with_noise(u, v):
    rng = np.random.default_rng(11)
    worst = 0.0
    for _ in range(20):
        d = detect_beacon(render_spot(SHAPE, u, v, noise=NOISE, rng=rng), PARAMS)
        assert d.valid and d.reason == "ok"
        worst = max(worst, abs(d.u_px - u), abs(d.v_px - v))
    assert worst < 0.05


def test_noise_free_spot_is_recovered_almost_exactly():
    d = detect_beacon(render_spot(SHAPE, 123.4, 56.7), PARAMS)
    assert d.valid
    assert d.u_px == pytest.approx(123.4, abs=0.01) and d.v_px == pytest.approx(56.7, abs=0.01)
    assert d.circularity > 0.9 and d.area_px > 20


def test_two_spots_are_rejected():
    image = render_spot(SHAPE, 100.0, 100.0, extra=[(200.0, 140.0, 150.0)], noise=NOISE, rng=np.random.default_rng(1))
    d = detect_beacon(image, PARAMS)
    assert not d.valid and d.reason == "multiple" and d.candidates == 2


def test_saturated_spot_is_flagged():
    image = render_spot(SHAPE, 160.3, 120.7, amplitude=600.0, noise=NOISE, rng=np.random.default_rng(2))
    assert image.max() == 255
    d = detect_beacon(image, PARAMS)
    assert not d.valid and d.reason == "saturated"
    assert abs(d.u_px - 160.3) < 1.0  # position still reported, flagged as unusable


def test_blank_and_noise_only_frames_find_nothing():
    rng = np.random.default_rng(3)
    for _ in range(10):
        image = render_spot(SHAPE, 0, 0, amplitude=0.0, noise=NOISE, rng=rng)
        assert detect_beacon(image, PARAMS).reason == "not_found"


def test_elongated_and_tiny_blobs_are_not_beacons():
    streak = np.full(SHAPE, 20, np.uint8)
    streak[100:103, 50:120] = 180  # a reflection line
    assert detect_beacon(streak, PARAMS).reason == "not_found"
    hot_pixel = np.full(SHAPE, 20, np.uint8)
    hot_pixel[50, 60] = 200
    assert detect_beacon(hot_pixel, PARAMS).reason == "not_found"


def test_roi_selects_one_of_two_spots_and_keeps_full_image_coordinates():
    image = render_spot(SHAPE, 100.25, 100.75, extra=[(200.0, 140.0, 150.0)])
    d = detect_beacon(image, PARAMS, roi=(60, 60, 140, 140))
    assert d.valid
    assert d.u_px == pytest.approx(100.25, abs=0.02) and d.v_px == pytest.approx(100.75, abs=0.02)
    assert detect_beacon(image, PARAMS, roi=(0, 0, 40, 40)).reason == "not_found"


def test_frame_differencing_removes_a_bright_static_background():
    rng = np.random.default_rng(4)
    sun = [(230.0, 60.0, 200.0)]  # bright spot that is there with the LED on or off
    on = render_spot(SHAPE, 90.4, 150.6, extra=sun, noise=NOISE, rng=rng)
    off = render_spot(SHAPE, 0, 0, amplitude=0.0, extra=sun, noise=NOISE, rng=rng)
    assert detect_beacon(on, PARAMS).reason == "multiple"
    d = detect_beacon(on, PARAMS, off_image=off)
    assert d.valid
    assert abs(d.u_px - 90.4) < 0.08 and abs(d.v_px - 150.6) < 0.08  # two noisy frames: sqrt(2) more noise


def test_colour_frames_are_accepted():
    gray = render_spot(SHAPE, 150.5, 99.5)
    d = detect_beacon(np.dstack([gray, gray, gray]), PARAMS)
    assert d.valid and d.u_px == pytest.approx(150.5, abs=0.02)
