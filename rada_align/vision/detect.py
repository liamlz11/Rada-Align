"""Find the beacon in one image and measure its centroid to subpixel accuracy.

Pure image processing: no camera, no hardware. Pixel convention as in
rada_align.interfaces: u to the right, v down, pixel centers at integers.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectParams:
    saturation_level: float
    threshold_sigma: float
    min_contrast: float
    min_area_px: int
    max_area_px: int
    min_circularity: float

    @classmethod
    def from_config(cls, vision) -> "DetectParams":
        return cls(
            saturation_level=vision.saturation_level,
            threshold_sigma=vision.threshold_sigma,
            min_contrast=vision.min_contrast,
            min_area_px=vision.min_area_px,
            max_area_px=vision.max_area_px,
            min_circularity=vision.min_circularity,
        )


@dataclass(frozen=True)
class Detection:
    u_px: float
    v_px: float
    valid: bool
    reason: str  # "ok", "not_found", "multiple", "saturated"
    area_px: int = 0
    peak: float = 0.0
    circularity: float = 0.0
    candidates: int = 0


def to_gray(image: np.ndarray) -> np.ndarray:
    """Float32 grey image from an 8-bit grey or BGR frame."""
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image.astype(np.float32)


def _circularity(weights: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> float:
    """Minor / major axis of the blob's intensity ellipse (1 = round)."""
    total = weights.sum()
    if total <= 0:
        return 0.0
    mx, my = (weights * xs).sum() / total, (weights * ys).sum() / total
    dx, dy = xs - mx, ys - my
    # 1/12 = variance of one pixel's own extent, so a 1-pixel-wide line is not "infinitely thin"
    cxx = (weights * dx * dx).sum() / total + 1.0 / 12.0
    cyy = (weights * dy * dy).sum() / total + 1.0 / 12.0
    cxy = (weights * dx * dy).sum() / total
    spread = np.sqrt(((cxx - cyy) / 2.0) ** 2 + cxy**2)
    major, minor = (cxx + cyy) / 2.0 + spread, (cxx + cyy) / 2.0 - spread
    return float(np.sqrt(max(minor, 0.0) / major)) if major > 0 else 0.0


def _refine(sig: np.ndarray, background: float, x: float, y: float, sigma: float) -> tuple[float, float]:
    """Iterated Gaussian-weighted centroid around (x, y), in `sig` coordinates.

    The first estimate uses the thresholded blob, whose outline depends on the
    noise. Here every pixel in a fixed window counts, weighted by a Gaussian
    the size of the spot and centred on the current estimate; at convergence
    the estimate is the centre of symmetry of the spot. This is close to the
    best a centroid can do in white noise.
    """
    sigma = max(sigma, 1.0)
    half = int(np.ceil(3.0 * sigma)) + 1
    h, w = sig.shape
    for _ in range(12):
        x0, x1 = int(round(x)) - half, int(round(x)) + half + 1
        y0, y1 = int(round(y)) - half, int(round(y)) + half + 1
        if x0 < 0 or y0 < 0 or x1 > w or y1 > h:
            break  # too close to the edge for a symmetric window: keep the estimate
        ys, xs = np.mgrid[y0:y1, x0:x1]
        weights = np.exp(-((xs - x) ** 2 + (ys - y) ** 2) / (2.0 * sigma**2)) * (sig[y0:y1, x0:x1] - background)
        total = weights.sum()
        if total <= 0:
            break
        # A matched Gaussian weight pulls the centroid halfway back to the
        # window centre, so step twice the measured offset.
        nx = x + 2.0 * ((weights * xs).sum() / total - x)
        ny = y + 2.0 * ((weights * ys).sum() / total - y)
        moved = abs(nx - x) + abs(ny - y)
        x, y = nx, ny
        if moved < 1e-4:
            break
    return x, y


def detect_beacon(
    image: np.ndarray,
    params: DetectParams,
    roi: tuple[int, int, int, int] | None = None,
    off_image: np.ndarray | None = None,
) -> Detection:
    """Detect one beacon.

    `roi` = (u0, v0, u1, v1), half-open, in full-image pixels. `off_image` is
    a frame taken with the LED off; if given, it is subtracted first so that
    sun and static background cancel (saturation is still judged on the
    LED-on frame).

    Rejections (valid=False): nothing above threshold, more than one blob that
    passes the size and roundness filters, or a saturated blob. A saturated
    blob still returns its centroid, flagged, because a clipped spot is biased.
    """
    gray = to_gray(image)
    signal = gray - to_gray(off_image) if off_image is not None else gray
    h, w = signal.shape
    u0, v0, u1, v1 = (0, 0, w, h) if roi is None else roi
    u0, v0, u1, v1 = max(0, u0), max(0, v0), min(w, u1), min(h, v1)
    if u1 - u0 < 3 or v1 - v0 < 3:
        return Detection(0.0, 0.0, False, "not_found")
    sig = signal[v0:v1, u0:u1]
    raw = gray[v0:v1, u0:u1]

    background = float(np.median(sig))
    noise = 1.4826 * float(np.median(np.abs(sig - background)))
    threshold = background + max(params.threshold_sigma * noise, params.min_contrast)
    mask = (sig > threshold).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    blobs = []
    for label in range(1, count):
        x, y, bw, bh, area = stats[label]
        if not params.min_area_px <= area <= params.max_area_px:
            continue
        # One pixel of margin: the spot's faint skirt carries centroid information.
        x0, y0 = max(0, x - 1), max(0, y - 1)
        x1, y1 = min(sig.shape[1], x + bw + 1), min(sig.shape[0], y + bh + 1)
        window = np.clip(sig[y0:y1, x0:x1] - background, 0.0, None)
        own = cv2.dilate((labels[y0:y1, x0:x1] == label).astype(np.uint8), np.ones((3, 3), np.uint8))
        weights = window * own
        ys, xs = np.mgrid[y0:y1, x0:x1]
        circ = _circularity(weights, xs, ys)
        if circ < params.min_circularity:
            continue
        total = weights.sum()
        cx, cy = (weights * xs).sum() / total, (weights * ys).sum() / total
        spot_sigma = float(np.sqrt(((weights * ((xs - cx) ** 2 + (ys - cy) ** 2)).sum() / total) / 2.0))
        cx, cy = _refine(sig, background, float(cx), float(cy), spot_sigma)
        blobs.append({
            "u": u0 + cx,
            "v": v0 + cy,
            "area": int(area),
            "peak": float(raw[y0:y1, x0:x1][own > 0].max()),
            "circ": circ,
        })

    if not blobs:
        return Detection(0.0, 0.0, False, "not_found")
    if len(blobs) > 1:
        return Detection(0.0, 0.0, False, "multiple", candidates=len(blobs))
    b = blobs[0]
    saturated = b["peak"] >= params.saturation_level
    return Detection(
        b["u"], b["v"], not saturated, "saturated" if saturated else "ok",
        area_px=b["area"], peak=b["peak"], circularity=b["circ"], candidates=1,
    )


def render_spot(
    shape: tuple[int, int],
    u: float,
    v: float,
    sigma_px: float = 2.5,
    amplitude: float = 150.0,
    background: float = 20.0,
    noise: float = 0.0,
    rng: np.random.Generator | None = None,
    extra: list[tuple[float, float, float]] | None = None,
) -> np.ndarray:
    """Synthetic 8-bit frame with a Gaussian spot (for tests and dry runs).

    `extra` adds more spots as (u, v, amplitude). Values are clipped to 0..255
    like a real sensor.
    """
    h, w = shape
    ys, xs = np.mgrid[0:h, 0:w]
    image = np.full(shape, background, dtype=np.float64)
    for su, sv, amp in [(u, v, amplitude), *(extra or [])]:
        image += amp * np.exp(-((xs - su) ** 2 + (ys - sv) ** 2) / (2.0 * sigma_px**2))
    if noise > 0:
        image += (rng or np.random.default_rng()).standard_normal(shape) * noise
    return np.clip(np.rint(image), 0, 255).astype(np.uint8)
