"""Monte Carlo over random setups, and a one-source-at-a-time error budget.

Everything here is scored against the TRUTH of the simulated world, not
against what the system reports about itself.
"""

from __future__ import annotations

from collections import Counter
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .config import ERROR_SOURCES, Config, with_only_sources
from .estimator import ERROR_AXES
from .runner import RunResult, run_single

# docs/CHALLENGE.md, preliminary error budget (deg, per axis). ESTIMATES.
PRELIMINARY_BUDGET_DEG = {
    "calibration": 0.015,
    "wind": 0.015,
    "fixture": 0.010,
    "inclinometer": 0.005,
    "actuator": 0.005,
    "turbulence": 0.003,
    "centroid": 0.002,
    "rangefinder": 0.002,
    "drift": None,  # no row of its own (part of "wind and tripod settling")
}


def _run_one(args) -> RunResult:
    cfg, seed, distance_m = args
    return run_single(cfg, seed, distance_m)


def run_montecarlo(
    cfg: Config, runs: int, seed: int, distance_m: float | None = None, jobs: int = 1
) -> list[RunResult]:
    """`runs` independent setups; run i always uses the seed (seed, i)."""
    tasks = [(cfg, (seed, i), distance_m) for i in range(runs)]
    if jobs <= 1:
        return [_run_one(task) for task in tasks]
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(_run_one, tasks, chunksize=max(1, runs // (jobs * 4))))


def _fault_category(reason: str) -> str:
    return reason.split(":")[0].split(" after ")[0].split(" failed ")[0]


def summarize(results: list[RunResult], tolerance_deg: float) -> dict:
    """Per-axis statistics over runs, pass rates and the STOP-gate check."""
    n = len(results)
    aligned = [r for r in results if r.aligned and r.truth_verify is not None]
    axes = {}
    for axis in ERROR_AXES:
        residual = np.array([abs(r.truth_verify[axis]["mean_deg"]) for r in aligned])
        score = np.array([r.truth_verify[axis]["score_deg"] for r in aligned])
        reported = np.array([abs(r.reported[axis]["mean_deg"]) for r in aligned])
        if len(aligned):
            axes[axis] = {
                "true_residual_mean_deg": float(residual.mean()),
                "true_residual_p95_deg": float(np.percentile(residual, 95)),
                "true_residual_max_deg": float(residual.max()),
                "true_score_p95_deg": float(np.percentile(score, 95)),
                "true_score_max_deg": float(score.max()),
                "reported_residual_p95_deg": float(np.percentile(reported, 95)),
            }
        else:
            axes[axis] = None

    def truly_passes(r: RunResult) -> bool:
        return all(r.truth_verify[a]["score_deg"] < tolerance_deg for a in ERROR_AXES)

    true_pass = [r for r in aligned if truly_passes(r)]
    # Stricter claim: the system's own score plus U_bias is inside the tolerance.
    with_bias = [
        r for r in aligned
        if all(r.reported[a]["score_with_bias_deg"] < tolerance_deg for a in ERROR_AXES)
    ]
    with_bias_wrong = [r for r in with_bias if not truly_passes(r)]
    times = np.array([r.t_aligned_s for r in aligned])
    faults = Counter(_fault_category(r.fault_reason) for r in results if not r.aligned)
    return {
        "runs": n,
        "distance_m": results[0].distance_m if results else None,
        "tolerance_deg": tolerance_deg,
        "axes": axes,
        "reported_pass_rate": len(aligned) / n,
        "true_pass_rate": len(true_pass) / n,
        "false_accept_rate": (len(aligned) - len(true_pass)) / len(aligned) if aligned else None,
        "with_bias_pass_rate": len(with_bias) / n,
        "with_bias_false_accept_rate": len(with_bias_wrong) / len(with_bias) if with_bias else None,
        "not_aligned": dict(faults),
        "time_to_align_s": {
            "mean": float(times.mean()),
            "p95": float(np.percentile(times, 95)),
            "max": float(times.max()),
        }
        if len(aligned)
        else None,
        "track_corrections_per_run": float(np.mean([r.corrections for r in aligned])) if aligned else None,
        "stop_gate_p95_exceeds_tolerance": any(
            a is not None and a["true_residual_p95_deg"] > tolerance_deg for a in axes.values()
        ),
    }


def _rms_true_error(results: list[RunResult]) -> dict:
    """RMS over runs of the true pointing error in the verify window.

    Per run and axis the true error has a mean (bias) and a sigma (motion);
    both count: rms^2 = mean over runs of (mean^2 + sigma^2).
    """
    aligned = [r for r in results if r.aligned and r.truth_verify is not None]
    out = {"aligned": len(aligned), "runs": len(results)}
    for axis in ERROR_AXES:
        if aligned:
            sq = [r.truth_verify[axis]["mean_deg"] ** 2 + r.truth_verify[axis]["sigma_deg"] ** 2 for r in aligned]
            out[axis] = float(np.sqrt(np.mean(sq)))
        else:
            out[axis] = None
    values = [out[a] for a in ERROR_AXES if out[a] is not None]
    out["worst_axis_deg"] = max(values) if values else None
    return out


def sensitivity(
    cfg: Config, runs: int, seed: int, distance_m: float | None = None, jobs: int = 1
) -> dict:
    """Measured error budget: one error source switched on at a time.

    "none" is the control residual with every source off (deadband, lever-arm
    coupling); "all" has everything on. The same seeds are used for every
    row, so each row sees the same initial setups.
    """
    rows = {}
    for name in ["none", *ERROR_SOURCES, "all"]:
        if name == "all":
            variant = cfg
        else:
            variant = with_only_sources(cfg, () if name == "none" else (name,))
        rows[name] = _rms_true_error(run_montecarlo(variant, runs, seed, distance_m, jobs))
        rows[name]["preliminary_deg"] = PRELIMINARY_BUDGET_DEG.get(name)
    base = rows["none"]["worst_axis_deg"] or 0.0
    for name, row in rows.items():
        worst = row["worst_axis_deg"]
        row["above_baseline_deg"] = (
            float(np.sqrt(max(worst**2 - base**2, 0.0))) if worst is not None and name != "none" else None
        )
    return rows


# ------------------------------------------------------------------ printing
def format_summary(summary: dict) -> str:
    tol = summary["tolerance_deg"]
    lines = [
        f"Monte Carlo: {summary['runs']} runs at {summary['distance_m']:g} m, tolerance {tol:g} deg",
        "TRUE residual = |mean true error| over the verify window; score = |mean| + 2*sigma (deg)",
        f"{'axis':10s} {'mean':>8s} {'p95':>8s} {'max':>8s} {'score p95':>10s} {'score max':>10s} {'reported p95':>13s}",
    ]
    for axis, a in summary["axes"].items():
        if a is None:
            lines.append(f"{axis:10s} (no aligned runs)")
            continue
        lines.append(
            f"{axis:10s} {a['true_residual_mean_deg']:8.4f} {a['true_residual_p95_deg']:8.4f} "
            f"{a['true_residual_max_deg']:8.4f} {a['true_score_p95_deg']:10.4f} "
            f"{a['true_score_max_deg']:10.4f} {a['reported_residual_p95_deg']:13.4f}"
        )
    lines.append(f"reported pass rate (system said ALIGNED): {summary['reported_pass_rate']:.1%}")
    lines.append(f"true pass rate (truth meets the criterion): {summary['true_pass_rate']:.1%}")
    if summary["false_accept_rate"] is not None:
        lines.append(f"false accepts (ALIGNED but truth fails):    {summary['false_accept_rate']:.1%}")
    line = f"pass incl. U_bias (score + 2*u_delta < tol):   {summary['with_bias_pass_rate']:.1%}"
    if summary["with_bias_false_accept_rate"] is not None:
        line += f", of which truth fails {summary['with_bias_false_accept_rate']:.1%}"
    lines.append(line)
    if summary["time_to_align_s"]:
        t = summary["time_to_align_s"]
        lines.append(f"time to ALIGNED: mean {t['mean']:.1f} s, p95 {t['p95']:.1f} s, max {t['max']:.1f} s")
        lines.append(f"re-alignments during tracking, per run: {summary['track_corrections_per_run']:.2f}")
    for reason, count in sorted(summary["not_aligned"].items(), key=lambda kv: -kv[1]):
        lines.append(f"not aligned: {count:4d} x {reason}")
    gate = summary["stop_gate_p95_exceeds_tolerance"]
    lines.append(f"STOP gate (p95 true residual > {tol:g} deg on any axis): {'TRIGGERED' if gate else 'not triggered'}")
    return "\n".join(lines)


def format_budget(rows: dict) -> str:
    lines = [
        "Error budget: RMS true error in the verify window, one source at a time (deg)",
        f"{'source':13s} {'prelim':>7s} {'worst axis':>10s} {'above base':>10s}  "
        + " ".join(f"{a:>9s}" for a in ERROR_AXES)
        + "  aligned",
    ]
    for name, row in rows.items():
        prelim = "   -   " if row["preliminary_deg"] is None else f"{row['preliminary_deg']:7.3f}"
        if row["worst_axis_deg"] is None:
            lines.append(f"{name:13s} {prelim} (no aligned runs)  {row['aligned']}/{row['runs']}")
            continue
        above = "     -    " if row["above_baseline_deg"] is None else f"{row['above_baseline_deg']:10.4f}"
        lines.append(
            f"{name:13s} {prelim} {row['worst_axis_deg']:10.4f} {above}  "
            + " ".join(f"{row[a]:9.4f}" for a in ERROR_AXES)
            + f"  {row['aligned']}/{row['runs']}"
        )
    return "\n".join(lines)
