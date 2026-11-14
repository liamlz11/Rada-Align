"""Frames, line of sight, boresight errors and parallax correction.

Units inside this module: radians and metres (every name carries its unit).

World frame: x right, y forward (radar toward horn), z up; origin at the
radar reference point. A body frame has the same meaning for its own head:
x right, y along its boresight, z up. The horn faces the radar, so its body
frame is turned 180 deg about z before its own pointing errors are applied.

Boresight errors are tangent-plane angles of a direction v in a body frame:
az = atan2(v.x, v.y) (positive to the head's right), el = atan2(v.z, v.y)
(positive up). A pinhole camera measures exactly these angles.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole model. Image convention: u grows to the right, v grows down."""

    f_px: float
    cx_px: float
    cy_px: float
    width_px: int
    height_px: int


def intrinsics_from_config(cam) -> Intrinsics:
    """Ideal intrinsics from a CameraCfg (principal point at the image center)."""
    return Intrinsics(
        f_px=cam.focal_length_mm * 1e-3 / (cam.pixel_size_um * 1e-6),
        cx_px=cam.width_px / 2.0,
        cy_px=cam.height_px / 2.0,
        width_px=cam.width_px,
        height_px=cam.height_px,
    )


def rot_x(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


FACE_BACK = rot_z(math.pi)  # turns a head to face the opposite head


def body_rotation(az_rad: float, el_rad: float, roll_rad: float = 0.0) -> np.ndarray:
    """Body-to-parent rotation: yaw right by az, pitch up by el, then roll."""
    return rot_z(-az_rad) @ rot_x(el_rad) @ rot_y(roll_rad)


def tangent_angles_rad(v_body: np.ndarray) -> tuple[float, float]:
    """(az, el) of a direction given in a body frame."""
    return math.atan2(v_body[0], v_body[1]), math.atan2(v_body[2], v_body[1])


def offset_to_angle_deg(offset_m: float, distance_m: float) -> float:
    """Angle subtended by a lateral offset at a distance."""
    return math.degrees(math.atan2(offset_m, distance_m))


def angle_to_offset_m(angle_rad: float, distance_m: float) -> float:
    """Lateral offset (dx or dh) that an angular error means at a distance."""
    return distance_m * math.tan(angle_rad)


def pixel_to_angles_rad(u_px: float, v_px: float, intr: Intrinsics) -> tuple[float, float]:
    return (
        math.atan((u_px - intr.cx_px) / intr.f_px),
        math.atan((intr.cy_px - v_px) / intr.f_px),
    )


def angles_to_pixel(az_rad: float, el_rad: float, intr: Intrinsics) -> tuple[float, float]:
    return (
        intr.cx_px + intr.f_px * math.tan(az_rad),
        intr.cy_px - intr.f_px * math.tan(el_rad),
    )


def net_offset_m(
    camera_offset_m: tuple[float, float],
    beacon_offset_m: tuple[float, float],
    own_roll_rad: float = 0.0,
    other_roll_rad: float = 0.0,
) -> tuple[float, float]:
    """Parallax baseline [right, up] in the observing head's body frame.

    The camera sits at camera_offset from its own reference point and looks
    at a beacon that sits at beacon_offset from the other head's reference
    point. The other head faces back, so its "right" is this head's "left".
    Roll of either head rotates its offset vector.
    """
    beacon = np.array([beacon_offset_m[0], 0.0, beacon_offset_m[1]])
    seen = rot_y(own_roll_rad).T @ FACE_BACK @ rot_y(other_roll_rad) @ beacon
    return seen[0] - camera_offset_m[0], seen[2] - camera_offset_m[1]


def mount_rotation(delta_az_rad: float, delta_el_rad: float) -> np.ndarray:
    """Camera-to-body rotation of a camera with calibration offset delta.

    delta is the pixel angle the camera reads for a target that sits exactly
    on the head's reference axis, so the camera itself is turned by -delta.
    """
    return body_rotation(-delta_az_rad, -delta_el_rad)


def boresight_error_rad(
    pixel_az_rad: float,
    pixel_el_rad: float,
    delta_az_rad: float,
    delta_el_rad: float,
    net_offset_xz_m: tuple[float, float],
    distance_m: float,
) -> tuple[float, float]:
    """(az, el) of the other head's reference point from this head's axis.

    The camera ray is rotated into the body frame with the calibrated mount
    offset, then the parallax baseline is removed. For small angles this is
    eps = pixel_angle - delta_calib - s / d (CLAUDE.md); written this way it
    stays exact when the parallax angle is degrees (tabletop distance).
    """
    ray_cam = np.array([math.tan(pixel_az_rad), 1.0, math.tan(pixel_el_rad)])
    ray = mount_rotation(delta_az_rad, delta_el_rad) @ ray_cam
    return (
        math.atan(ray[0] / ray[1] - net_offset_xz_m[0] / distance_m),
        math.atan(ray[2] / ray[1] - net_offset_xz_m[1] / distance_m),
    )
