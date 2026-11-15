"""Camera-to-datum calibration: reversal method, calibration file, checks.

Reversal: the head is read in two orientations 180 deg apart about the datum
axis. In the head's own frame the target direction flips sign and the camera
offset `delta` does not, so

    delta = (reading_normal + reading_flipped) / 2
    axis  = (reading_normal - reading_flipped) / 2     (where the target is)

What the reversal cannot see: the head seats slightly differently each time.
With seat errors f_n (normal) and f_f (flipped), and the head left in the
normal position for operation, the offset that matters is delta + f_n while
the estimate is delta + (f_n - f_f) / 2: an error of -(f_n + f_f) / 2, i.e.
sigma_seat / sqrt(2). That term goes into the reported uncertainty.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from . import geometry as g
from .config import Calibration, CalibrationMissing, Config, HeadCalibration
from .estimator import compute_errors_deg
from .sensors import build_simulation


# ------------------------------------------------------------------ file I/O
def save_calibration(path: str | Path, calibration: Calibration, meta: dict | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "meta": meta or {},
        "radar": _plain(dataclasses.asdict(calibration.radar)),
        "horn": _plain(dataclasses.asdict(calibration.horn)),
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _plain(d: dict) -> dict:
    return {k: [float(x) for x in v] if isinstance(v, (tuple, list)) else float(v) for k, v in d.items()}


def load_calibration(path: str | Path) -> Calibration:
    """Read calib/datum.yaml. Raises CalibrationMissing with the reason."""
    path = Path(path)
    if not path.is_file():
        raise CalibrationMissing(f"calibration file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        heads = {}
        for side in ("radar", "horn"):
            head = data[side]
            heads[side] = HeadCalibration(
                delta_az_deg=float(head["delta_az_deg"]),
                delta_el_deg=float(head["delta_el_deg"]),
                u_delta_deg=float(head["u_delta_deg"]),
                camera_offset_m=tuple(float(x) for x in head["camera_offset_m"]),
                beacon_offset_m=tuple(float(x) for x in head["beacon_offset_m"]),
            )
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise CalibrationMissing(f"calibration file {path} is not valid: {exc!r}") from exc
    for side, head in heads.items():
        if not (math.isfinite(head.u_delta_deg) and head.u_delta_deg >= 0):
            raise CalibrationMissing(f"calibration file {path}: {side}.u_delta_deg must be >= 0")
    return Calibration(**heads)


# ------------------------------------------------------------ reversal maths
@dataclass(frozen=True)
class ReversalResult:
    delta_az_deg: float
    delta_el_deg: float
    axis_az_deg: float  # where the target is relative to the datum axis
    axis_el_deg: float
    u_stat_deg: float  # statistical uncertainty of delta (worse axis), 1 sigma
    u_delta_deg: float  # u_stat combined with the seat term, 1 sigma
    n_normal: int
    n_flipped: int


def _mean_and_sigma(samples: np.ndarray, blocks: int = 8) -> tuple[np.ndarray, np.ndarray]:
    """Mean per axis and its standard error from batch means.

    Wind sway is correlated over seconds, so sigma / sqrt(n) of the raw
    samples is too optimistic. Batch means are still optimistic when a batch
    is shorter than the correlation time; the simulation study checks this.
    """
    samples = np.asarray(samples, dtype=float)
    mean = samples.mean(axis=0)
    usable = (len(samples) // blocks) * blocks
    if usable < 2 * blocks:
        return mean, samples.std(axis=0, ddof=1)  # too few samples: no averaging credit
    batch = samples[:usable].reshape(blocks, -1, samples.shape[1]).mean(axis=1)
    return mean, batch.std(axis=0, ddof=1) / math.sqrt(blocks)


def reversal_from_readings(
    normal: np.ndarray, flipped: np.ndarray, seat_sigma_deg: float
) -> ReversalResult:
    """`normal`, `flipped`: N x 2 arrays of (az, el) readings in the head's own
    frame, parallax-corrected, taken with delta assumed zero."""
    mean_n, se_n = _mean_and_sigma(normal)
    mean_f, se_f = _mean_and_sigma(flipped)
    delta = (mean_n + mean_f) / 2.0
    axis = (mean_n - mean_f) / 2.0
    u_stat = float((np.sqrt(se_n**2 + se_f**2) / 2.0).max())
    return ReversalResult(
        delta_az_deg=float(delta[0]),
        delta_el_deg=float(delta[1]),
        axis_az_deg=float(axis[0]),
        axis_el_deg=float(axis[1]),
        u_stat_deg=u_stat,
        u_delta_deg=math.hypot(u_stat, seat_sigma_deg / math.sqrt(2.0)),
        n_normal=len(normal),
        n_flipped=len(flipped),
    )


# -------------------------------------------------------- reversal simulation
def _read_radar_camera(sim, calibration: Calibration, cfg: Config, seconds: float) -> np.ndarray:
    intr = g.intrinsics_from_config(cfg.camera)
    dt = 1.0 / cfg.camera.rate_hz
    hw = sim.hardware
    rows = []
    distance = roll_r = roll_h = 0.0
    for i in range(round(seconds / dt)):
        sim.world.advance(dt)
        # running means of the slow quantities, as the controller does
        distance += (hw.rangefinder.read_distance_m() - distance) / (i + 1)
        roll_r += (hw.radar_inclinometer.read_tilt().roll_deg - roll_r) / (i + 1)
        roll_h += (hw.horn_inclinometer.read_tilt().roll_deg - roll_h) / (i + 1)
        errors = compute_errors_deg(
            hw.radar_camera.measure_beacon(), hw.horn_camera.measure_beacon(),
            roll_r, roll_h, distance, calibration, intr,
        )
        rows.append((errors["radar_az"], errors["radar_el"]))
    return np.array(rows)


def simulate_reversal(
    cfg: Config,
    seed,
    distance_m: float | None = None,
    seconds_per_position: float = 60.0,
    seat_sigma_deg: float | None = None,
) -> tuple[ReversalResult, dict]:
    """Run the reversal on the simulated radar head.

    Returns the result and the truth: `operating_delta_deg` is the offset
    that is really in effect afterwards (head left in the normal position).
    """
    seat = cfg.calibration.fixture_repeat_sigma_deg if seat_sigma_deg is None else seat_sigma_deg
    # The fixture term is modelled here explicitly, per seating.
    base = cfg.replace_path("calibration.fixture_repeat_sigma_deg", 0.0)
    seed = tuple(int(x) for x in np.atleast_1d(seed))
    rng = np.random.default_rng(np.random.SeedSequence(seed + (99,)).spawn(1)[0])
    seat_flipped, seat_normal = rng.standard_normal((2, 2)) * seat

    first = build_simulation(base, seed, distance_m)
    scenario = first.scenario
    delta = np.array(scenario.radar_delta_deg)
    zero = dataclasses.replace(
        first.calibration,
        radar=dataclasses.replace(first.calibration.radar, delta_az_deg=0.0, delta_el_deg=0.0),
    )
    pinned = dataclasses.asdict(scenario)
    pinned.pop("distance_m")

    def position(tag: int, roll_deg: float, head_delta: np.ndarray):
        overrides = dict(pinned, radar_roll_deg=roll_deg, radar_delta_deg=tuple(head_delta))
        sim = build_simulation(base, seed + (tag,), scenario.distance_m, calibration=zero, **overrides)
        return _read_radar_camera(sim, zero, base, seconds_per_position)

    # Flipped first, then normal: the head stays in the normal seat afterwards.
    # A seat error is fixed to the datum, so in the flipped head frame it has
    # the opposite sign.
    flipped = position(1, scenario.radar_roll_deg + 180.0, delta - seat_flipped)
    normal = position(2, scenario.radar_roll_deg, delta + seat_normal)

    result = reversal_from_readings(normal, flipped, seat)
    truth = {
        "delta_deg": tuple(delta),
        "operating_delta_deg": tuple(delta + seat_normal),
        "seat_normal_deg": tuple(seat_normal),
        "seat_flipped_deg": tuple(seat_flipped),
    }
    return result, truth


def reversal_study(
    cfg: Config, runs: int, seed: int, distance_m: float, seconds_per_position: float,
    seat_sigma_deg: float,
) -> dict:
    """Monte Carlo of the reversal: achieved error against the predicted one."""
    errors, predicted, stat = [], [], []
    for i in range(runs):
        result, truth = simulate_reversal(cfg, (seed, i), distance_m, seconds_per_position, seat_sigma_deg)
        op = truth["operating_delta_deg"]
        errors.append((result.delta_az_deg - op[0], result.delta_el_deg - op[1]))
        predicted.append(result.u_delta_deg)
        stat.append(result.u_stat_deg)
    errors = np.array(errors)
    return {
        "runs": runs,
        "distance_m": distance_m,
        "seconds_per_position": seconds_per_position,
        "seat_sigma_deg": seat_sigma_deg,
        "rms_error_az_deg": float(np.sqrt((errors[:, 0] ** 2).mean())),
        "rms_error_el_deg": float(np.sqrt((errors[:, 1] ** 2).mean())),
        "mean_error_az_deg": float(errors[:, 0].mean()),
        "mean_error_el_deg": float(errors[:, 1].mean()),
        "predicted_u_delta_deg": float(np.mean(predicted)),
        "predicted_u_stat_deg": float(np.mean(stat)),
    }


# ------------------------------------------------------------- flange mirror
def mirror_apparent_angle_deg(
    tilt_deg: float, distance_m: float, camera_x_m: float = 0.0, beacon_x_m: float = 0.0
) -> float:
    """Azimuth at which a camera sees its own side's beacon in a flat mirror.

    Camera and beacon sit on the same head at lateral positions camera_x and
    beacon_x; the mirror is `distance_m` away with its normal turned by
    `tilt_deg` from the line back to the head. Exact reflection geometry.
    """
    tilt = math.radians(tilt_deg)
    normal = np.array([math.sin(tilt), -math.cos(tilt)])  # (x, y), facing the head
    point = np.array([0.0, distance_m])  # a point on the mirror
    beacon = np.array([beacon_x_m, 0.0])
    image = beacon - 2.0 * np.dot(beacon - point, normal) * normal  # virtual image
    ray = image - np.array([camera_x_m, 0.0])
    return math.degrees(math.atan2(ray[0], ray[1]))
