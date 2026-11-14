"""Angular resolution of candidate rig parts, for docs/RIG.md.

    python scripts/rig_resolution.py [distance_m ...]      (default: 2 12)

The parts below are generic CANDIDATES, not parts we own. Each entry is an
ASSUMPTION about a typical part; replace it with the real part's datasheet
value once we know what we have.
"""
import math
import sys

TOLERANCE_DEG = 0.05
DEADBAND_DEG = 0.01  # the loop needs steps clearly finer than this


def verdict(step_deg: float) -> str:
    if step_deg <= DEADBAND_DEG / 2:
        return "OK"
    if step_deg <= DEADBAND_DEG:
        return "marginal"
    return "too coarse"


def rotary():
    print("Pan / tilt (independent of distance)")
    print(f"  {'drive':58s} {'deg/step':>10s}  verdict")
    full = 1.8  # ASSUMPTION: standard 200 step/rev stepper
    rows = [
        ("hobby servo (CHALLENGE.md: ~1 deg)", 1.0),
        ("stepper direct, full step", full),
        ("stepper direct, 1/16 microstep", full / 16),
        ("stepper direct, 1/256 microstep (nominal, not repeatable)", full / 256),
        ("28BYJ-48 geared stepper, 4096 half-steps/rev [unverified]", 360 / 4096),
        ("stepper + 5:1 belt, 1/16", full / 16 / 5),
        ("stepper + 100:1 worm, 1/16", full / 16 / 100),
        ("stepper + 90:1 worm stage, 1/8", full / 8 / 90),
    ]
    # Tangent arm: a screw pushes the end of a lever. Cheapest fine rotation.
    for pitch_mm, arm_mm, micro in ((0.5, 100, 1), (0.5, 100, 16), (0.8, 150, 1), (8.0, 150, 16)):
        step_mm = pitch_mm / 200 / micro
        rows.append((
            f"tangent arm: {pitch_mm} mm/rev screw on a {arm_mm} mm arm, 1/{micro}",
            math.degrees(math.atan(step_mm / arm_mm)),
        ))
    for name, step in rows:
        print(f"  {name:58s} {step:10.5f}  {verdict(step)}")
    for arm_mm in (100, 150):
        travel = arm_mm * math.tan(math.radians(2.0))
        print(f"  tangent arm {arm_mm} mm: +/-2 deg needs +/-{travel:.1f} mm of screw travel")


def linear(d: float):
    print(f"X / Z at {d:g} m  (tolerance {TOLERANCE_DEG} deg = {d * math.tan(math.radians(TOLERANCE_DEG)) * 1000:.2f} mm, "
          f"+/-1 deg setup error needs +/-{d * math.tan(math.radians(1.0)) * 1000:.0f} mm travel)")
    print(f"  {'drive':58s} {'mm/step':>10s} {'deg/step':>10s}  verdict")
    rows = [
        ("lead screw 8 mm/rev, full step", 8 / 200),
        ("lead screw 8 mm/rev, 1/16", 8 / 200 / 16),
        ("lead screw 2 mm/rev, full step", 2 / 200),
        ("GT2 belt, 20-tooth pulley (40 mm/rev), full step", 40 / 200),
        ("GT2 belt, 20-tooth pulley (40 mm/rev), 1/16", 40 / 200 / 16),
        ("hand micrometer stage, 10 um graduation (manual)", 0.010),
    ]
    for name, step_mm in rows:
        step_deg = math.degrees(math.atan(step_mm / 1000 / d))
        print(f"  {name:58s} {step_mm:10.4f} {step_deg:10.5f}  {verdict(step_deg)}")


def cameras(d: float):
    print(f"Cameras at {d:g} m")
    print(f"  {'camera':46s} {'deg/px':>8s} {'FOV deg':>12s} {'mm/px':>7s} {'px for 0.005 deg':>17s}")
    rows = [
        # name, width_px, height_px, focal length in px
        ("12 MP 1.55 um sensor + 25 mm lens", 4056, 3040, 25e-3 / 1.55e-6),
        ("12 MP 1.55 um sensor + 16 mm lens", 4056, 3040, 16e-3 / 1.55e-6),
        ("12 MP 1.55 um sensor + 6 mm lens", 4056, 3040, 6e-3 / 1.55e-6),
        ("1080p webcam, 70 deg horizontal FOV", 1920, 1080, 960 / math.tan(math.radians(35))),
        ("720p webcam, 60 deg horizontal FOV", 1280, 720, 640 / math.tan(math.radians(30))),
    ]
    for name, w, h, f_px in rows:
        deg_px = math.degrees(math.atan(1 / f_px))
        fov = f"{2 * math.degrees(math.atan(w / 2 / f_px)):.1f}x{2 * math.degrees(math.atan(h / 2 / f_px)):.1f}"
        print(f"  {name:46s} {deg_px:8.4f} {fov:>12s} {d / f_px * 1000:7.3f} {0.005 / deg_px:17.2f}")


def sensors():
    print("Sensors (datasheet values found by web search, see docs/RIG.md)")
    g_noise = 22.5e-6  # ADXL355 noise density, g/sqrt(Hz)
    for bw in (1.0, 10.0):
        print(f"  ADXL355 22.5 ug/sqrt(Hz) in {bw:g} Hz bandwidth -> {math.degrees(g_noise * math.sqrt(bw)):.5f} deg rms tilt noise")
    print(f"  SCL3300 angle output 182 LSB/deg -> {1 / 182:.5f} deg per LSB; offset error up to +/-0.57 deg over temperature")
    for roll_err in (0.57, 0.1):
        for net_mm in (100.0, 2.0):
            for d in (2.0, 12.0):
                err = math.degrees(math.atan(net_mm / 1000 * math.sin(math.radians(roll_err)) / d))
                print(f"  roll error {roll_err} deg x net camera/beacon offset {net_mm:g} mm at {d:g} m -> {err:.5f} deg")
    for d, acc_mm, net_mm in ((2.0, 60.0, 2.0), (2.0, 60.0, 100.0), (12.0, 100.0, 2.0), (12.0, 100.0, 100.0)):
        err = math.degrees(abs(math.atan(net_mm / 1000 / d) - math.atan(net_mm / 1000 / (d + acc_mm / 1000))))
        print(f"  range error {acc_mm:g} mm, net offset {net_mm:g} mm at {d:g} m -> {err:.5f} deg")
    for half_angle in (3.0, 10.0, 60.0):
        print(f"  beacon LED half angle +/-{half_angle:g} deg: full intensity only if the head points within that of the other head")


if __name__ == "__main__":
    distances = [float(a) for a in sys.argv[1:]] or [2.0, 12.0]
    rotary()
    for d in distances:
        print()
        linear(d)
        cameras(d)
    print()
    sensors()
