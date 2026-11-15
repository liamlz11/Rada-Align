"""Centroid stability of a static beacon: sigma in pixels and in degrees.

    python scripts/measure_stability.py --device 0 --seconds 30
    python scripts/measure_stability.py --fake --seconds 30

Run it from native Windows Python (WSL2 does not see USB cameras). The real
camera is opened only when --device is given explicitly. --fake uses a
synthetic spot with sensor noise and proves the pipeline, not the hardware.
Degrees use the focal length in calib/camera.yaml if present, otherwise the
nominal one from config.yaml (then marked as nominal).
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from rada_align.config import load_config
from rada_align.geometry import intrinsics_from_config
from rada_align.vision.camera import OpenCVSource, SyntheticSource, VisionCamera
from rada_align.vision.intrinsics import load_lens_calibration


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--device", type=int, default=None, help="camera index; opens the real camera")
    p.add_argument("--fake", action="store_true", help="synthetic spot instead of a camera")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--config", default=None)
    p.add_argument("--lens", default="calib/camera.yaml")
    p.add_argument("--out", default="out")
    args = p.parse_args(argv)
    if args.fake == (args.device is not None):
        p.error("give exactly one of --device N or --fake")

    cfg = load_config(args.config)
    lens_path = Path(args.lens)
    lens = load_lens_calibration(lens_path) if lens_path.is_file() and not args.fake else None
    if args.fake:
        source, rate = SyntheticSource(), cfg.camera.rate_hz
        nominal = intrinsics_from_config(cfg.camera)
    else:
        source = OpenCVSource(args.device, args.width, args.height, cfg.vision.exposure)
        print(f"camera {args.device}: {source.width} x {source.height}, exposure fixed: {source.exposure_fixed}")
        nominal = intrinsics_from_config(cfg.camera)
        rate = None
    f_px = float(lens.camera_matrix[0, 0]) if lens is not None else nominal.f_px
    # Without a lens file pixels are reported raw, so sigma in px is exact and
    # only the conversion to degrees depends on the focal length used.
    camera = VisionCamera(source, cfg.vision, nominal, lens=None)

    rows, reasons = [], {}
    t0 = time.monotonic()
    n_target = round(args.seconds * rate) if rate else None
    while (len(rows) + sum(reasons.values()) < n_target) if n_target else (time.monotonic() - t0 < args.seconds):
        m = camera.measure_beacon()
        if m.valid:
            rows.append((m.timestamp_s - t0, m.u_px, m.v_px))
        else:
            reason = camera.last_detection.reason
            reasons[reason] = reasons.get(reason, 0) + 1
    camera.close()

    total = len(rows) + sum(reasons.values())
    print(f"frames: {total}, valid: {len(rows)}, rejected: {reasons or 'none'}")
    if len(rows) < 10:
        print("too few valid frames for a sigma")
        return 1
    data = np.array(rows)
    su, sv = data[:, 1].std(ddof=1), data[:, 2].std(ddof=1)
    deg = lambda px: math.degrees(math.atan(px / f_px))
    source_name = "calib/camera.yaml" if lens is not None else "NOMINAL from config.yaml (ASSUMPTION)"
    print(f"centroid mean: u {data[:, 1].mean():.3f} px, v {data[:, 2].mean():.3f} px")
    print(f"centroid sigma: u {su:.4f} px, v {sv:.4f} px")
    print(f"focal length {f_px:.0f} px ({source_name}) -> sigma u {deg(su):.5f} deg, v {deg(sv):.5f} deg")
    if args.fake:
        print("SYNTHETIC source: this is the algorithm's noise floor, not a camera measurement.")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(data[:, 0], data[:, 1] - data[:, 1].mean(), label="u")
    axes[0].plot(data[:, 0], data[:, 2] - data[:, 2].mean(), label="v")
    axes[0].set_xlabel("time (s)"); axes[0].set_ylabel("centroid - mean (px)"); axes[0].legend(); axes[0].grid(alpha=0.3)
    axes[1].plot(data[:, 1] - data[:, 1].mean(), data[:, 2] - data[:, 2].mean(), ".", ms=3)
    axes[1].set_xlabel("u (px)"); axes[1].set_ylabel("v (px)"); axes[1].set_aspect("equal"); axes[1].grid(alpha=0.3)
    fig.suptitle(f"Static beacon centroid, sigma u {su:.4f} px, v {sv:.4f} px" + (" (SYNTHETIC)" if args.fake else ""))
    fig.tight_layout()
    name = "stability_fake.png" if args.fake else "stability.png"
    fig.savefig(out / name, dpi=120)
    print(f"plot: {(out / name).resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
