"""Lens calibration from chessboard photos -> calib/camera.yaml.

    python scripts/calibrate_camera.py --images calib/chessboard --cols 9 --rows 6 --square-mm 25

Take 15-20 photos of a flat chessboard with the alignment camera at the
resolution used for alignment: different distances, tilts and positions,
including the image corners. --cols/--rows count INNER corners.
"""
import argparse
import sys
from pathlib import Path

import cv2

from rada_align.vision.intrinsics import calibrate_from_images, save_lens_calibration


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--images", required=True, help="folder with chessboard photos")
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square-mm", type=float, required=True)
    p.add_argument("--out", default="calib/camera.yaml")
    args = p.parse_args(argv)

    paths = sorted(q for q in Path(args.images).iterdir() if q.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
    images = [img for img in (cv2.imread(str(q)) for q in paths) if img is not None]
    if not images:
        print(f"no images found in {args.images}")
        return 1
    try:
        lens, used = calibrate_from_images(images, (args.cols, args.rows), args.square_mm)
    except ValueError as exc:
        print(exc)
        return 1
    fx, fy = lens.camera_matrix[0, 0], lens.camera_matrix[1, 1]
    cx, cy = lens.camera_matrix[0, 2], lens.camera_matrix[1, 2]
    print(f"board found in {used} of {len(images)} images, image size {lens.image_size}")
    print(f"fx {fx:.1f} px, fy {fy:.1f} px, cx {cx:.1f}, cy {cy:.1f}, reprojection rms {lens.rms_px:.3f} px")
    if lens.rms_px > 0.5:
        print("WARNING: rms above 0.5 px; retake blurred or badly lit photos")
    save_lens_calibration(args.out, lens, {"images": len(images), "used": used,
                                           "pattern": [args.cols, args.rows], "square_mm": args.square_mm})
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
