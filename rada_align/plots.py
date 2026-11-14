"""Plots for the simulator: convergence, residual histogram, error budget."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .estimator import ERROR_AXES
from .runner import RunResult


def plot_convergence(result: RunResult, tolerance_deg: float, path: Path) -> None:
    """True and measured error of one run over time (needs record_history)."""
    h = result.history
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for ax, group, title in ((axes[0], (0, 1), "Radar error"), (axes[1], (2, 3), "Horn error")):
        for i in group:
            ax.plot(h["t_s"], h["truth_deg"][:, i], label=f"{ERROR_AXES[i]} true")
            ax.plot(h["t_s"], h["measured_deg"][:, i], ".", ms=2, label=f"{ERROR_AXES[i]} measured")
        ax.axhspan(-tolerance_deg, tolerance_deg, color="green", alpha=0.15, label=f"+/-{tolerance_deg:g} deg")
        ax.set_yscale("symlog", linthresh=0.1)
        ax.set_ylabel("deg")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=3, loc="upper right")
    if result.t_aligned_s is not None:
        for ax in axes:
            ax.axvline(result.t_aligned_s, color="k", ls="--", lw=0.8)
    axes[1].set_xlabel("time (s)")
    fig.suptitle(f"Convergence, one simulated run at {result.distance_m:g} m (SIMULATION)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_residual_histogram(results: list[RunResult], tolerance_deg: float, path: Path) -> None:
    aligned = [r for r in results if r.aligned and r.truth_verify is not None]
    fig, axes = plt.subplots(2, 2, figsize=(9, 6))
    for ax, axis in zip(axes.ravel(), ERROR_AXES):
        values = [r.truth_verify[axis]["mean_deg"] for r in aligned]
        ax.hist(values, bins=40, color="tab:blue")
        for x in (-tolerance_deg, tolerance_deg):
            ax.axvline(x, color="red", ls="--")
        ax.set_title(f"{axis}: true residual, {len(values)} aligned runs")
        ax.set_xlabel("deg")
    fig.suptitle("True residual error after alignment (SIMULATION, assumed parameters)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_error_budget(rows: dict, path: Path) -> None:
    names = [n for n in rows if rows[n]["worst_axis_deg"] is not None]
    measured = [rows[n]["worst_axis_deg"] for n in names]
    prelim = [rows[n]["preliminary_deg"] or 0.0 for n in names]
    x = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.bar(x - 0.2, prelim, 0.4, label="preliminary estimate (CHALLENGE.md)")
    ax.bar(x + 0.2, measured, 0.4, label="simulated: RMS true error, worst axis")
    ax.set_xticks(x, names, rotation=30, ha="right")
    ax.set_ylabel("deg")
    ax.set_title("Error budget, one source at a time (SIMULATION, assumed parameters)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
