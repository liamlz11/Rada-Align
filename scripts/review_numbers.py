"""Reproduces every number quoted in docs/REVIEW.md.

    python scripts/review_numbers.py

What-ifs change a copy of the config in memory; config.yaml is not modified.
"""
import math

from rada_align.config import load_config
from rada_align.montecarlo import run_montecarlo, summarize

deg = math.degrees


def arithmetic():
    print("== arithmetic")
    for d in (12.0, 2.0):
        print(f"d={d} m")
        for mm in (0.3, 1.0, 2.0):
            print(f"  offset knowledge error {mm} mm -> {deg(math.atan(mm / 1000 / d)):.4f} deg")
        print(f"  5 mm LED subtends {deg(math.atan(0.005 / d)):.4f} deg; 5% centroid shift = 0.25 mm -> {deg(math.atan(0.00025 / d)):.4f} deg")
        print(f"  tolerance 0.05 deg = {d * math.tan(math.radians(0.05)) * 1000:.2f} mm")
    for gap_um, base_mm in ((10, 100), (30, 100), (50, 100), (10, 50)):
        print(f"fixture seat gap {gap_um} um over {base_mm} mm base -> {deg(math.atan(gap_um * 1e-6 / (base_mm * 1e-3))):.4f} deg")
    alpha, dT, length, width = 23e-6, 5.0, 0.100, 0.050
    dl = alpha * dT * length
    print(f"thermal: aluminium, {dT} K across a {width*1000:.0f} mm wide, {length*1000:.0f} mm long bracket -> {dl*1e6:.1f} um -> {deg(math.atan(dl / width)):.4f} deg")
    print(f"sensor shift 1 um behind a 25 mm lens -> {deg(math.atan(1e-6 / 25e-3)):.4f} deg")
    for bw in (10.0, 20.0, 30.0):
        print(f"horn 3 dB beamwidth {bw} deg: pointing loss at 0.05 deg = {12 * (0.05 / bw) ** 2:.6f} dB, at 1 deg = {12 * (1 / bw) ** 2:.4f} dB")
    sigma_wind = 0.015
    print(f"wind: per-sample sigma {sigma_wind} -> 2*sigma = {2 * sigma_wind:.3f} deg = {2 * sigma_wind / 0.05:.0%} of tolerance; left for |mean| = {0.05 - 2 * sigma_wind:.3f} deg")
    bias = math.hypot(0.015, 0.010)
    print(f"bias rows RSS (calibration, fixture) = {bias:.4f} deg; 2*bias = {2 * bias:.4f}")
    # reversal: averaging time T per position, OU sway tau, jitter white at rate
    tau, rate, sway, jit = 2.0, 10.0, 0.0106, 0.0106
    for T in (10.0, 60.0):
        x = T / tau
        f = 2 / x * (1 - (1 - math.exp(-x)) / x)
        s_mean = math.sqrt(sway**2 * f + jit**2 / (T * rate))
        print(f"reversal: {T:.0f} s per position -> mean sigma {s_mean:.4f} deg, delta sigma (half difference) {s_mean / math.sqrt(2):.4f} deg")
    print(f"track rule: drift threshold 0.03 + 2*sigma 0.03 = {0.03 + 2 * sigma_wind:.3f} deg allowed before a correction")
    # Mirror on the horn flange, seen by the RADAR-side camera (reflection of
    # the radar-side beacon). Mirror tilt theta moves the virtual image by
    # 2*d*theta at a distance of 2*d, so the camera sees an angle of theta.
    print(f"flange mirror seen from the radar head: 0.3 px resolves {deg(math.atan(0.3 / 16129)):.5f} deg of flange tilt (factor 1, not 2)")
    for d in (12.0, 2.0):
        print(f"  d={d} m: reflection point moves {d * math.tan(math.radians(0.05)) * 1000:.1f} mm at 0.05 deg tilt, {d * math.tan(math.radians(1.0)) * 1000:.0f} mm at 1 deg")
    # Reversal with an independent seat error at each of the two seatings:
    # the estimate of the operating offset is wrong by (f1 + f2) / 2.
    for seat in (0.010, 0.005):
        for T, meas in ((10.0, 0.0043), (60.0, 0.0019)):
            print(f"reversal residual: seat {seat} deg per seating, {T:.0f} s per position -> seat part {seat / math.sqrt(2):.4f}, total {math.hypot(seat / math.sqrt(2), meas):.4f} deg")
    # In the flipped position the camera is on the other side of the axis, so
    # the matched-offset cancellation is lost: baseline 2*s = 20 cm.
    for d, dd in ((12.0, 0.05), (2.0, 0.05), (2.0, 0.002)):
        err = deg(abs(math.atan(0.20 / d) - math.atan(0.20 / (d + dd))))
        print(f"reversal, flipped position: d={d} m, range error {dd * 1000:.0f} mm -> {err:.4f} deg on that reading, {err / 2:.4f} deg on delta")


def whatif(label, cfg, runs=500):
    s = summarize(run_montecarlo(cfg, runs, 0, 12.0, jobs=8), 0.05)
    axes = [a for a in s["axes"].values() if a]
    p95 = max(a["true_residual_p95_deg"] for a in axes) if axes else float("nan")
    score = max(a["true_score_p95_deg"] for a in axes) if axes else float("nan")
    fa = s["false_accept_rate"]
    print(f"{label:36s} reported {s['reported_pass_rate']:6.1%}  true {s['true_pass_rate']:6.1%}  "
          f"false-accept {'   n/a' if fa is None else format(fa, '6.1%')}  resid p95 {p95:.4f}  "
          f"score p95 {score:.4f}  not aligned {s['not_aligned']}")


if __name__ == "__main__":
    arithmetic()
    base = load_config()
    print("== what-ifs, 12 m, 500 runs, seed 0, scored against truth with 0.05 deg")
    whatif("baseline (config.yaml)", base)
    c = base.replace_path("calibration.residual_sigma_deg", 0.005).replace_path("calibration.fixture_repeat_sigma_deg", 0.005)
    whatif("calibration 0.005, fixture 0.005", c)
    w = base.replace_path("disturbances.wind_jitter_deg", 0.0053).replace_path("disturbances.wind_sway_deg", 0.0053)
    whatif("wind halved (0.0075 total)", w)
    cw = c.replace_path("disturbances.wind_jitter_deg", 0.0053).replace_path("disturbances.wind_sway_deg", 0.0053)
    whatif("both", cw)
    whatif("verify window 10 s", base.replace_path("estimator.verify_window_s", 10.0))
    z = base.replace_path("calibration.residual_sigma_deg", 0.0).replace_path("calibration.fixture_repeat_sigma_deg", 0.0)
    whatif("no calibration/fixture bias at all", z)
    w2 = base.replace_path("disturbances.wind_jitter_deg", 0.0159).replace_path("disturbances.wind_sway_deg", 0.0159)
    whatif("wind x1.5 (0.0225 total)", w2)
    w3 = base.replace_path("disturbances.wind_jitter_deg", 0.0212).replace_path("disturbances.wind_sway_deg", 0.0212)
    whatif("wind x2 (0.030 total)", w3)
    c3 = base.replace_path("calibration.residual_sigma_deg", 0.03)
    whatif("calibration 0.030", c3)
    f3 = base.replace_path("calibration.fixture_repeat_sigma_deg", 0.03)
    whatif("fixture 0.030", f3)
