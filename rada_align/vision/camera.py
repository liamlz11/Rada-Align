"""Camera driver: frames in, BeaconMeasurement out.

`VisionCamera` implements the Camera interface on top of any frame source.
`OpenCVSource` is the real one (USB webcam through cv2.VideoCapture). It must
run from native Windows Python: WSL2 does not see USB cameras by default.
"""

from __future__ import annotations

import sys
import time
from collections import deque
from typing import Callable, Protocol

import cv2
import numpy as np

from ..config import VisionCfg
from ..geometry import Intrinsics
from ..interfaces import BeaconMeasurement, Camera
from .detect import DetectParams, Detection, detect_beacon, render_spot
from .intrinsics import LensCalibration, to_nominal


class FrameSource(Protocol):
    def read(self) -> np.ndarray | None:
        """Return the next frame, or None if no frame is available."""

    def close(self) -> None: ...


class OpenCVSource:
    """USB camera with fixed exposure where the driver allows it."""

    def __init__(self, device_index: int, width: int, height: int, exposure: float | None = None):
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        self._cap = cv2.VideoCapture(device_index, backend)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {device_index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.exposure_fixed = False
        if exposure is not None:
            # DirectShow: 0.25 = manual. Other backends use other codes, so the
            # result is read back and reported instead of assumed.
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self._cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
            self.exposure_fixed = abs(self._cap.get(cv2.CAP_PROP_EXPOSURE) - exposure) < 1e-6
        self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def read(self) -> np.ndarray | None:
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        self._cap.release()


class SyntheticSource:
    """Static Gaussian spot with sensor noise. For tests and dry runs only."""

    def __init__(self, shape=(480, 640), u=320.3, v=240.7, noise=2.0, seed=0, **spot):
        self.shape, self.u, self.v, self.noise, self._spot = shape, u, v, noise, spot
        self._rng = np.random.default_rng(seed)
        self.led_on = True

    def read(self) -> np.ndarray | None:
        amplitude = self._spot.get("amplitude", 150.0) if self.led_on else 0.0
        spot = {**self._spot, "amplitude": amplitude}
        return render_spot(self.shape, self.u, self.v, noise=self.noise, rng=self._rng, **spot)

    def close(self) -> None:
        pass


class VisionCamera(Camera):
    """Beacon centroid from a frame source, in nominal-camera pixels.

    Tracks the beacon in a window around its last position, and reports the
    scatter of the last `sigma_window` valid centroids as sigma_px.

    Frame differencing (LED on minus LED off) needs something that switches
    the LED: pass `led` (called with True/False). None of the four hardware
    interfaces provides LED control yet; the ESP32 firmware (task 6) will.
    """

    MISSES_BEFORE_FULL_FRAME = 3

    def __init__(
        self,
        source: FrameSource,
        vision: VisionCfg,
        nominal: Intrinsics,
        lens: LensCalibration | None = None,
        led: Callable[[bool], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        if vision.frame_differencing and led is None:
            raise ValueError("frame_differencing needs an `led` callback to switch the beacon")
        self._source = source
        self._vision = vision
        self._params = DetectParams.from_config(vision)
        self._nominal = nominal
        self._lens = lens
        self._led = led
        self._clock = clock
        self._last: tuple[float, float] | None = None  # raw pixels, for the ROI
        self._misses = 0
        self._recent: deque[tuple[float, float]] = deque(maxlen=max(2, vision.sigma_window))
        self.last_detection: Detection | None = None

    def _roi(self, shape) -> tuple[int, int, int, int] | None:
        half = self._vision.roi_half_px
        if self._last is None or half <= 0:
            return None
        u, v = self._last
        return int(u) - half, int(v) - half, int(u) + half + 1, int(v) + half + 1

    def measure_beacon(self) -> BeaconMeasurement:
        off = None
        if self._vision.frame_differencing:
            self._led(False)
            off = self._source.read()
            self._led(True)
        frame = self._source.read()
        now = self._clock()
        if frame is None or (self._vision.frame_differencing and off is None):
            self.last_detection = Detection(0.0, 0.0, False, "no_frame")
            return BeaconMeasurement(0.0, 0.0, 0.0, False, now)

        detection = detect_beacon(frame, self._params, self._roi(frame.shape), off)
        self.last_detection = detection
        if detection.reason == "not_found":
            self._misses += 1
            if self._misses >= self.MISSES_BEFORE_FULL_FRAME:
                self._last = None  # lost: search the whole frame next time
        else:
            self._misses = 0
        if detection.reason in ("ok", "saturated"):
            self._last = (detection.u_px, detection.v_px)

        u, v = to_nominal(detection.u_px, detection.v_px, self._lens, self._nominal)
        if detection.valid:
            self._recent.append((u, v))
        sigma = 0.0
        if len(self._recent) >= 2:
            sigma = float(np.array(self._recent).std(axis=0, ddof=1).max())
        return BeaconMeasurement(u, v, sigma, detection.valid, now)

    def close(self) -> None:
        self._source.close()
