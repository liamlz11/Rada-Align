"""Simulated reversal calibration: achieved accuracy vs prediction.

    python scripts/reversal_study.py [runs]        (default 100)

SIMULATION with the ASSUMED parameters of config.yaml. Rows marked "tape"
replace the rangefinder with a 2 mm range knowledge (config copy in memory).
"""
import math
import sys

from rada_align.calibration import mirror_apparent_angle_deg, reversal_study
from rada_align.config import load_config

if __name__ == "__main__":
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    cfg = load_config()
    tape = cfg.replace_path("rangefinder.bias_sigma_m", 0.002).replace_path("rangefinder.noise_m", 0.0)
    print(f"Reversal calibration, {runs} simulated setups per row (deg). error = estimated delta - operating delta")
    print(f"{'d m':>4s} {'range':>10s} {'seat':>6s} {'T s':>4s} | {'rms az':>7s} {'rms el':>7s} {'mean az':>8s} {'mean el':>8s} | {'pred u':>7s} {'pred stat':>9s} {'seat/sqrt2':>10s}")
    for d in (12.0, 2.0):
        for label, c in (("5 cm", cfg), ("tape 2 mm", tape)):
            for seat in (0.010, 0.005):
                for seconds in (10.0, 60.0):
                    r = reversal_study(c, runs, 0, d, seconds, seat)
                    print(f"{d:4.0f} {label:>10s} {seat:6.3f} {seconds:4.0f} | {r['rms_error_az_deg']:7.4f} {r['rms_error_el_deg']:7.4f} "
                          f"{r['mean_error_az_deg']:+8.4f} {r['mean_error_el_deg']:+8.4f} | {r['predicted_u_delta_deg']:7.4f} "
                          f"{r['predicted_u_stat_deg']:9.4f} {seat / math.sqrt(2):10.4f}")
    print()
    print("Flange mirror: camera and beacon on the radar head, mirror on the horn flange.")
    print("apparent angle of the reflected beacon minus its angle at zero tilt, per degree of mirror tilt:")
    for d in (12.0, 2.0):
        for cam, beacon in ((0.0, 0.0), (0.10, -0.10), (0.10, 0.0)):
            zero = mirror_apparent_angle_deg(0.0, d, cam, beacon)
            for tilt in (0.05, 1.0):
                shift = mirror_apparent_angle_deg(tilt, d, cam, beacon) - zero
                print(f"  d={d:4.0f} m camera x={cam:+.2f} beacon x={beacon:+.2f}: tilt {tilt:4.2f} deg -> shift {shift:+.5f} deg "
                      f"(factor {shift / tilt:.4f}); angle at zero tilt {zero:+.4f} deg")
