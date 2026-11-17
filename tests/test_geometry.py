import math

import numpy as np
import pytest

from rada_align import geometry as g
from rada_align.config import config_from_dict, load_config


def test_10_47_mm_at_12_m_is_0_050_deg():
    assert g.offset_to_angle_deg(0.01047, 12.0) == pytest.approx(0.050, abs=0.001)


def test_angle_offset_roundtrip():
    offset = g.angle_to_offset_m(math.radians(0.05), 12.0)
    assert offset * 1000 == pytest.approx(10.472, abs=0.001)
    assert g.offset_to_angle_deg(offset, 12.0) == pytest.approx(0.05, abs=1e-9)


def test_body_rotation_points_boresight_right_and_up():
    forward = g.body_rotation(math.radians(2.0), math.radians(3.0)) @ np.array([0.0, 1.0, 0.0])
    assert forward[0] > 0 and forward[2] > 0
    az, el = g.tangent_angles_rad(forward)
    assert math.degrees(az) == pytest.approx(2.0, abs=1e-9)
    assert math.atan2(forward[2], math.hypot(forward[0], forward[1])) == pytest.approx(
        math.radians(3.0), abs=1e-12
    )


def test_pixel_angle_roundtrip_and_image_convention():
    intr = g.Intrinsics(16000.0, 2028.0, 1520.0, 4056, 3040)
    u, v = g.angles_to_pixel(math.radians(0.3), math.radians(0.2), intr)
    assert u > intr.cx_px  # target to the right -> larger u
    assert v < intr.cy_px  # target above -> smaller v (v grows down)
    az, el = g.pixel_to_angles_rad(u, v, intr)
    assert math.degrees(az) == pytest.approx(0.3, abs=1e-9)
    assert math.degrees(el) == pytest.approx(0.2, abs=1e-9)


@pytest.mark.parametrize("distance_m", [2.0, 12.0])
@pytest.mark.parametrize("true_error_deg", [0.0, 0.05, -1.0])
def test_parallax_correction_recovers_true_angle(distance_m, true_error_deg):
    """Camera 10 cm right of the axis, beacon 4 cm left of the far reference
    point (as this head sees it), camera mount misaligned by 0.3 deg."""
    camera_offset, beacon_seen, delta = 0.10, -0.04, math.radians(0.3)
    reference_x = distance_m * math.tan(math.radians(true_error_deg))
    pixel_angle = math.atan2(reference_x + beacon_seen - camera_offset, distance_m) + delta

    uncorrected = math.degrees(pixel_angle - delta)
    assert abs(uncorrected - true_error_deg) > 0.5  # parallax is large

    eps, _ = g.boresight_error_rad(
        pixel_angle, 0.0, delta, 0.0, (beacon_seen - camera_offset, 0.0), distance_m
    )
    assert math.degrees(eps) == pytest.approx(true_error_deg, abs=1e-9)


def test_net_offset_mirrors_the_other_head():
    # Other head's beacon 10 cm to ITS right appears 10 cm to OUR left.
    assert g.net_offset_m((0.0, 0.0), (0.10, 0.0)) == pytest.approx((-0.10, 0.0))
    # Camera 10 cm right, far beacon 10 cm to its own left: same physical side.
    assert g.net_offset_m((0.10, 0.0), (-0.10, 0.0)) == pytest.approx((0.0, 0.0))
    assert g.net_offset_m((0.10, 0.0), (0.0, 0.10)) == pytest.approx((-0.10, 0.10))


def test_net_offset_roll_rotates_the_baseline():
    roll = math.radians(0.5)
    x, z = g.net_offset_m((0.0, 0.0), (0.0, 0.10), other_roll_rad=roll)
    assert abs(x) * 1000 == pytest.approx(0.8727, abs=0.001)  # 100 mm * sin(0.5 deg)
    assert z == pytest.approx(0.10 * math.cos(roll))


@pytest.mark.parametrize(
    "distance_m, net_m, expected_deg",
    [(12.0, 0.10, 0.0020), (2.0, 0.10, 0.0697), (2.0, 0.01, 0.0070), (2.0, 0.0, 0.0)],
)
def test_range_error_sensitivity_depends_on_net_offset(distance_m, net_m, expected_deg):
    """A 5 cm range error matters at 2 m unless camera and beacon offsets match."""
    pixel_angle = math.atan2(net_m, distance_m)  # reference point exactly on axis
    eps, _ = g.boresight_error_rad(pixel_angle, 0.0, 0.0, 0.0, (net_m, 0.0), distance_m + 0.05)
    assert abs(math.degrees(eps)) == pytest.approx(expected_deg, abs=0.0002)


def test_config_loads_and_is_strict():
    cfg = load_config()
    assert cfg.geometry.tolerance_deg == 0.005  # R7 tightened by the team, 2026-11-17
    assert cfg.actuators.pan.step == pytest.approx(1.8 / 16 / 100)
    changed = cfg.replace_path("camera.centroid_noise_px", 0.0)
    assert changed.camera.centroid_noise_px == 0.0
    assert cfg.camera.centroid_noise_px == 0.3

    import yaml
    from rada_align.config import DEFAULT_CONFIG_PATH

    raw = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    raw["camera"]["typo_key"] = 1
    with pytest.raises(ValueError, match="unknown keys"):
        config_from_dict(raw)
    del raw["camera"]["typo_key"]
    del raw["geometry"]["distance_m"]
    with pytest.raises(ValueError, match="missing keys"):
        config_from_dict(raw)
