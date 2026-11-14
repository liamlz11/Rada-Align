"""Simulator CLI.

    python -m rada_align.sim --runs 500 --distance 12
    python -m rada_align.sim --stream --distance 12 --seed 3

The first form prints the Monte Carlo table and the measured error budget
and saves plots and montecarlo.json to ./out. `--stream` runs one alignment
and prints one JSON line per tick (schema: docs/messages.md).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

from .config import load_config
from .montecarlo import format_budget, format_summary, run_montecarlo, sensitivity, summarize
from .runner import run_single


def _parse(argv):
    p = argparse.ArgumentParser(prog="python -m rada_align.sim", description=__doc__.split("\n")[0])
    p.add_argument("--config", default=None, help="path to config.yaml (default: repo root)")
    p.add_argument("--distance", type=float, default=None, help="distance in metres (default: config)")
    p.add_argument("--runs", type=int, default=None, help="Monte Carlo runs (default: config)")
    p.add_argument("--seed", type=int, default=None, help="base random seed (default: config)")
    p.add_argument("--sens-runs", type=int, default=None, help="runs per error-budget row")
    p.add_argument("--no-sensitivity", action="store_true", help="skip the error-budget analysis")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--out", default="out", help="output directory")
    p.add_argument("--jobs", type=int, default=1, help="parallel worker processes")
    p.add_argument("--stream", action="store_true", help="one run, live state as JSON lines")
    p.add_argument("--realtime", action="store_true", help="with --stream: pace output in real time")
    p.add_argument("--track-time", type=float, default=None, help="seconds of tracking after ALIGNED")
    return p.parse_args(argv)


def _stream(cfg, args) -> int:
    dt = 1.0 / cfg.camera.rate_hz

    def emit(status, truth, sim):
        message = status.to_dict()
        message["sim"] = True
        message["truth"] = truth
        sys.stdout.write(json.dumps(message) + "\n")
        sys.stdout.flush()
        if args.realtime:
            time.sleep(dt)

    seed = cfg.montecarlo.seed if args.seed is None else args.seed
    run_single(cfg, seed, args.distance, track_time_s=args.track_time, on_tick=emit)
    return 0


def main(argv=None) -> int:
    args = _parse(argv)
    cfg = load_config(args.config)
    if args.stream:
        return _stream(cfg, args)

    mc = cfg.montecarlo
    runs = mc.runs if args.runs is None else args.runs
    seed = mc.seed if args.seed is None else args.seed
    sens_runs = mc.sensitivity_runs if args.sens_runs is None else args.sens_runs
    tol = cfg.geometry.tolerance_deg
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print("SIMULATION with ASSUMED parameters (config.yaml). Not measured hardware performance.")
    t0 = time.time()
    results = run_montecarlo(cfg, runs, seed, args.distance, args.jobs)
    summary = summarize(results, tol)
    print(format_summary(summary))
    print(f"(Monte Carlo wall time {time.time() - t0:.0f} s)")

    budget = None
    if not args.no_sensitivity:
        t0 = time.time()
        budget = sensitivity(cfg, sens_runs, seed, args.distance, args.jobs)
        print()
        print(format_budget(budget))
        print(f"({sens_runs} runs per row, wall time {time.time() - t0:.0f} s)")

    report = {"summary": summary, "budget": budget, "config": asdict(cfg), "seed": seed}
    (out / "montecarlo.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    if not args.no_plots:
        from . import plots

        example = run_single(cfg, (seed, 0), args.distance, record_history=True)
        plots.plot_convergence(example, tol, out / "convergence.png")
        plots.plot_residual_histogram(results, tol, out / "residual_histogram.png")
        if budget is not None:
            plots.plot_error_budget(budget, out / "error_budget.png")
    print(f"\nsaved to {out.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
