"""Lens calibration (chessboard) and mapping of pixels to the nominal camera.

Design decision: the controller works with ONE ideal pinhole model per
camera type (rada_align.geometry.Intrinsics from config.yaml). A real lens
has its own focal lengths, principal point and distortion, so the driver
converts every measured pixel into the pixel the ideal camera would have
seen. Nothing downstream needs to know about the real lens.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from ..geometry import Intrinsics


@dataclass(frozen=True)
class LensCalibration:
    camera_matrix: np.ndarray  # 3 x 3
    dist_coeffs: np.ndarray  # k1, k2, p1, p2, k3
    image_size: tuple[int, int]  # width, height
    rms_px: float  # reprojection error of the calibration


def save_lens_calibration(path: str | Path, lens: LensCalibration, meta: dict | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "meta": meta or {},
        "camera_matrix": np.asarray(lens.camera_matrix, float).tolist(),
        "dist_coeffs": np.asarray(lens.dist_coeffs, float).ravel().tolist(),
        "image_size": [int(lens.image_size[0]), int(lens.image_size[1])],
        "rms_px": float(lens.rms_px),
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def load_lens_calibration(path: str | Path) -> LensCalibration:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return LensCalibration(
        camera_matrix=np.array(data["camera_matrix"], dtype=float).reshape(3, 3),
        dist_coeffs=np.array(data["dist_coeffs"], dtype=float),
        image_size=(int(data["image_size"][0]), int(data["image_size"][1])),
        rms_px=float(data["rms_px"]),
    )


def to_nominal(u_px: float, v_px: float, lens: LensCalibration | None, nominal: Intrinsics) -> tuple[float, float]:
    """Pixel in the real image -> pixel the ideal `nominal` camera would see."""
    if lens is None:
        return u_px, v_px
    point = np.array([[[u_px, v_px]]], dtype=np.float64)
    x, y = cv2.undistortPoints(point, lens.camera_matrix, lens.dist_coeffs)[0, 0]
    return nominal.cx_px + nominal.f_px * float(x), nominal.cy_px + nominal.f_px * float(y)


def calibrate_from_corners(
    object_points: list[np.ndarray], image_points: list[np.ndarray], image_size: tuple[int, int]
) -> LensCalibration:
    """cv2.calibrateCamera on already-detected corners (one array per view)."""
    obj = [np.asarray(o, dtype=np.float32).reshape(-1, 1, 3) for o in object_points]
    img = [np.asarray(i, dtype=np.float32).reshape(-1, 1, 2) for i in image_points]
    rms, matrix, dist, _, _ = cv2.calibrateCamera(obj, img, image_size, None, None)
    return LensCalibration(matrix, dist.ravel(), image_size, float(rms))


def chessboard_object_points(pattern: tuple[int, int], square_mm: float) -> np.ndarray:
    """Inner-corner grid (cols, rows) on the z = 0 plane, in millimetres."""
    cols, rows = pattern
    grid = np.zeros((cols * rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return grid


def calibrate_from_images(
    images: list[np.ndarray], pattern: tuple[int, int], square_mm: float
) -> tuple[LensCalibration, int]:
    """Chessboard calibration. Returns the calibration and the number of
    images in which the board was found. Needs at least 5 usable views."""
    grid = chessboard_object_points(pattern, square_mm)
    object_points, image_points, size = [], [], None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for image in images:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, pattern)
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(grid)
        image_points.append(corners)
    if len(image_points) < 5:
        raise ValueError(f"chessboard found in only {len(image_points)} of {len(images)} images; need at least 5")
    return calibrate_from_corners(object_points, image_points, size), len(image_points)
