"""Abstract hardware interfaces.

Simulated drivers (rada_align.sensors) and real drivers (rada_align.vision,
rada_align.hw) implement these, so the estimator and controller never know
which one they are talking to.

Units at this boundary: pixels for image coordinates (u right, v down),
degrees for angles, millimetres for linear axes, metres for range, seconds
for time.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

LINEAR_AXES = ("x", "z")  # millimetres
ROTARY_AXES = ("pan", "tilt")  # degrees
AXES = LINEAR_AXES + ROTARY_AXES


class ActuatorError(RuntimeError):
    """A motion command was rejected (soft limit, e-stop, driver error)."""


@dataclass(frozen=True)
class BeaconMeasurement:
    """Centroid of the opposite beacon in one camera.

    Pixels are those of the NOMINAL camera of config.yaml (ideal pinhole,
    principal point at the image centre). A real driver converts its own
    lens (focal lengths, principal point, distortion) to that model, so the
    controller uses one set of intrinsics for every camera.
    """

    u_px: float
    v_px: float
    sigma_px: float  # centroid uncertainty of this sample
    valid: bool  # False: not found, saturated, or multiple blobs
    timestamp_s: float


@dataclass(frozen=True)
class TiltReading:
    """Dual-axis inclinometer reading relative to gravity."""

    pitch_deg: float  # boresight above the horizon
    roll_deg: float  # about the boresight, positive = right side down
    timestamp_s: float


class Camera(ABC):
    @abstractmethod
    def measure_beacon(self) -> BeaconMeasurement:
        """Return the current beacon centroid; valid=False if not usable."""


class Inclinometer(ABC):
    @abstractmethod
    def read_tilt(self) -> TiltReading:
        """Return pitch and roll relative to gravity."""


class Rangefinder(ABC):
    @abstractmethod
    def read_distance_m(self) -> float:
        """Return the distance between the two heads in metres."""


class Actuators(ABC):
    """Horn mount: x and z in millimetres, pan and tilt in degrees.

    Sign convention, seen from the horn looking at the radar: +x moves the
    horn to its right, +z moves it up, +pan turns it to its right, +tilt
    turns it up.
    """

    @abstractmethod
    def move_relative(self, axis: str, delta: float) -> None:
        """Start a relative move (non-blocking). Raises ActuatorError if rejected."""

    @abstractmethod
    def position(self, axis: str) -> float:
        """Return the position the driver believes the axis is at."""

    @abstractmethod
    def limits(self, axis: str) -> tuple[float, float]:
        """Return the soft limits (min, max) of one axis."""

    @abstractmethod
    def is_moving(self) -> bool:
        """Return True while any axis is in motion."""

    @abstractmethod
    def home(self, axis: str) -> None:
        """Run the homing sequence on one axis."""

    @abstractmethod
    def stop(self) -> None:
        """Stop all axes immediately."""


@dataclass
class Hardware:
    """Everything the controller talks to. Simulated and real parts can mix."""

    radar_camera: Camera
    horn_camera: Camera
    radar_inclinometer: Inclinometer
    horn_inclinometer: Inclinometer
    rangefinder: Rangefinder
    actuators: Actuators
