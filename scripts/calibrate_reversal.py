"""Reversal calibration of the radar-side head, step by step.

    python scripts/calibrate_reversal.py --sim --distance 2 --seconds 60

Writes the calibration file the controller needs. With --sim the readings
come from the simulator and the file is written to calib/datum.sim.yaml, so a
simulated file can never be mistaken for a real one. The real-camera path
needs the vision driver (task 4) and is not available yet.
"""
import argparse
import dataclasses
import sys
import time
from pathlib import Path

from rada_align.calibration import save_calibration, simulate_reversal
from rada_align.config import Calibration, HeadCalibration, load_config

STEPS = """
Reversal calibration (radar-side head)
  1. Mount the head on the datum in the FLIPPED position (rotated 180 deg about
     the reference axis). Tighten as you will in the test.
  2. Do not touch either tripod. The system records {seconds:.0f} s.
  3. Rotate the head back to the NORMAL position and tighten. This is the
     position it stays in: do not re-seat it after this step.
  4. The system records {seconds:.0f} s again.
  At tabletop distance the range must be known to about 2 mm (tape measure).
"""


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--sim", action="store_true", help="take the readings from the simulator")
    p.add_argument("--config", default=None)
    p.add_argument("--distance", type=float, default=None, help="metres (default: config)")
    p.add_argument("--seconds", type=float, default=60.0, help="recording time per position")
    p.add_argument("--seat-sigma", type=float, default=None,
                   help="seat repeatability of the fixture, deg (default: config fixture value)")
    p.add_argument("--seed", type=int, default=0, help="--sim only")
    p.add_argument("--out", default=None, help="output file")
    args = p.parse_args(argv)

    if not args.sim:
        print("The real-camera path needs the vision driver (task 4). Use --sim for now.")
        return 2

    cfg = load_config(args.config)
    seat = cfg.calibration.fixture_repeat_sigma_deg if args.seat_sigma is None else args.seat_sigma
    print(STEPS.format(seconds=args.seconds))
    print("SIMULATION: readings come from the simulator with the assumed parameters.\n")
    result, truth = simulate_reversal(cfg, args.seed, args.distance, args.seconds, seat)

    print(f"samples: flipped {result.n_flipped}, normal {result.n_normal}")
    print(f"delta_az = {result.delta_az_deg:+.4f} deg    delta_el = {result.delta_el_deg:+.4f} deg")
    print(f"target seen at az {result.axis_az_deg:+.4f} deg, el {result.axis_el_deg:+.4f} deg from the datum axis")
    print(f"uncertainty: statistical {result.u_stat_deg:.4f} deg, seat {seat:.4f}/sqrt(2) deg "
          f"-> u_delta = {result.u_delta_deg:.4f} deg (1 sigma), U_bias = {2 * result.u_delta_deg:.4f} deg")
    op = truth["operating_delta_deg"]
    print(f"simulator truth: operating delta ({op[0]:+.4f}, {op[1]:+.4f}) deg -> "
          f"error ({result.delta_az_deg - op[0]:+.4f}, {result.delta_el_deg - op[1]:+.4f}) deg")

    calibration = Calibration(
        radar=HeadCalibration(
            delta_az_deg=result.delta_az_deg,
            delta_el_deg=result.delta_el_deg,
            u_delta_deg=result.u_delta_deg,
            camera_offset_m=cfg.radar_head.camera_offset_m,
            beacon_offset_m=cfg.radar_head.beacon_offset_m,
        ),
        # The horn head has no calibration procedure yet (docs/CALIBRATION.md).
        horn=HeadCalibration(
            delta_az_deg=0.0,
            delta_el_deg=0.0,
            u_delta_deg=cfg.calibration.residual_sigma_deg,
            camera_offset_m=cfg.horn_head.camera_offset_m,
            beacon_offset_m=cfg.horn_head.beacon_offset_m,
        ),
    )
    out = Path(args.out) if args.out else Path("calib") / "datum.sim.yaml"
    meta = {
        "method": "reversal",
        "source": "SIMULATION",
        "created_unix_s": time.time(),
        "seconds_per_position": args.seconds,
        "seat_sigma_deg": seat,
        "horn_head": "NOT CALIBRATED: delta 0 and u_delta are assumptions",
        "result": dataclasses.asdict(result),
    }
    save_calibration(out, calibration, meta)
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
