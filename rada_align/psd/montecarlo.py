"""Monte Carlo of the final design, scored against truth.

    python -m rada_align.psd.montecarlo --runs 200 --distance 12

Reported pass = the controller's VERIFY passed. True pass = the same window
scored on the true errors. False accept = reported pass but true fail.
"""

from __future__ import annotations

import argparse

import numpy as np

from .config import WEATHERS, load_psd_config, quiet
from .controller import PsdController
from .world import World


def one_run(cfg, seed, distance_m, max_s=120.0, weather="clear") -> dict:
    w = World(cfg, seed, distance_m)
    w.set_weather(weather)
    c = PsdController(w)
    c.start()
    while w.t < max_s and c.state not in ("TRACK", "FAULT"):
        c.tick()
        w.step(c.dt)
    out = {"state": c.state, "t": w.t, "fault": c.fault_reason, "iters": c._iter}
    if c.after:
        out["reported_pass"] = True
        out["with_bias_pass"] = bool(c.after["pass_with_bias"])
        out["u_bias"] = c.after["reported"]["u_bias"]
        out["true_pass"] = bool(c.after["truth_pass"])
        out["true_score"] = c.after["truth"]["score"]
    else:
        out["reported_pass"] = False
        out["with_bias_pass"] = False
        out["true_pass"] = False
    return out


def run(cfg, runs, distance_m, seed0=0, weather="clear") -> dict:
    res = [one_run(cfg, seed0 + i, distance_m, weather=weather) for i in range(runs)]
    rep = np.array([r["reported_pass"] for r in res])
    tru = np.array([r["true_pass"] for r in res])
    wb = np.array([r["with_bias_pass"] for r in res])
    ub = np.array([r["u_bias"] for r in res if "u_bias" in r])
    scores = np.array([r["true_score"] for r in res if "true_score" in r])
    times = np.array([r["t"] for r in res if r["state"] == "TRACK"])
    faults = {}
    for r in res:
        if r["state"] == "FAULT":
            faults[r["fault"]] = faults.get(r["fault"], 0) + 1
    return {
        "runs": runs, "distance_m": distance_m, "weather": weather,
        "reported_pass": float(rep.mean()), "true_pass": float(tru.mean()),
        "false_accept": float((rep & ~tru).mean()),
        "with_bias_pass": float(wb.mean()),
        "with_bias_false_accept": float((wb & ~tru).mean()),
        "u_bias_deg": None if len(ub) == 0 else ub.mean(axis=0).round(4).tolist(),
        "p95_true_score": None if len(scores) == 0 else np.percentile(scores, 95, axis=0).round(4).tolist(),
        "median_time_to_track_s": None if len(times) == 0 else float(np.median(times)),
        "faults": faults,
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m rada_align.psd.montecarlo")
    p.add_argument("--runs", type=int, default=200)
    p.add_argument("--distance", type=float, default=12.0)
    p.add_argument("--keep", nargs="*", default=None,
                   help="error sources to keep (default: all on); e.g. --keep wind")
    p.add_argument("--weather", choices=WEATHERS, default="clear",
                   help="field condition (same seeds as clear: paired comparison)")
    args = p.parse_args(argv)
    cfg = load_psd_config()
    if args.keep is not None:
        cfg = quiet(cfg, tuple(args.keep))
    r = run(cfg, args.runs, args.distance, weather=args.weather)
    print(f"final design Monte Carlo, {r['runs']} runs at {r['distance_m']} m, weather {r['weather']} (SIMULATION, assumed parameters)")
    print(f"  reported pass {r['reported_pass']:.1%} | TRUE pass {r['true_pass']:.1%} | false accepts {r['false_accept']:.1%}")
    print(f"  pass incl. U_bias {r['with_bias_pass']:.1%} | of all runs, incl. U_bias but truth fails {r['with_bias_false_accept']:.1%}")
    print(f"  U_bias (k=2) per axis (eh_az, eh_el, er_az, er_el): {r['u_bias_deg']} deg")
    print(f"  p95 true score per axis (eh_az, eh_el, er_az, er_el): {r['p95_true_score']} deg")
    print(f"  median time to TRACK {r['median_time_to_track_s']} s; faults {r['faults']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
