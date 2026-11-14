"""Abstract hardware interfaces.

Simulated drivers (rada_align.sensors) and real drivers (rada_align.vision,
rada_align.hw) implement these, so the estimator and controller never know
which one they are talking to.

PROVISIONAL (task 0 skeleton): signatures are finalized in the task 2 plan.

Units at this boundary: pixels for image coordinates, degrees for angles,
millimetres for linear axes, metres for range, seconds for time.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

LINEAR_AXES = ("x", "z")  # millimetres
ROTARY_AXES = ("pan", "tilt")  # degrees
AXES = LINEAR_AXES + ROTARY_AXES


@dataclass(frozen=True)
class BeaconMeasurement:
    """Centroid of the opposite beacon in one camera."""

    u_px: float
    v_px: float
    sigma_px: float  # centroid scatter over the averaging window
    valid: bool  # False: not found, saturated, or multiple blobs
    timestamp_s: float


@dataclass(frozen=True)
class TiltReading:
    """Dual-axis inclinometer reading relative to gravity."""

    pitch_deg: float
    roll_deg: float
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
    """Horn mount: x and z in millimetres, pan and tilt in degrees."""

    @abstractmethod
    def move_relative(self, axis: str, delta: float) -> None:
        """Command a relative move on one axis (mm or deg)."""

    @abstractmethod
    def position(self, axis: str) -> float:
        """Return the commanded position of one axis (mm or deg)."""

    @abstractmethod
    def is_moving(self) -> bool:
        """Return True while any axis is in motion."""

    @abstractmethod
    def home(self, axis: str) -> None:
        """Run the homing sequence on one axis."""

    @abstractmethod
    def stop(self) -> None:
        """Stop all axes immediately."""
