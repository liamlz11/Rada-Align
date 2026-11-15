# Camera-to-datum calibration

The radar-side camera measures angles from its own optical axis. The requirement
is relative to the radar's mechanical reference axis. `delta` is the link: the
pixel angle the camera reads for a target that sits exactly on the reference
axis. Any error in `delta` is a constant pointing error that the system cannot
see afterwards (docs/REVIEW.md, W1), so the calibration also has to say how well
it knows `delta`: `u_delta_deg`, shown on the dashboard as `U_bias = 2 * u_delta`.

All accuracy numbers below are SIMULATION with the assumed parameters of
config.yaml (`python scripts/reversal_study.py 60`, 60 setups per row). Nothing
here has been measured on hardware.

## 1. Reversal method (every setup)

Needs a datum that lets the head be mounted in two orientations 180 deg apart
about the reference axis (open question 11). If the datum is a flat face only,
this method does not apply.

### Procedure

1. Set up radar and horn as for the test. Beacons on, both cameras see them.
2. Mount the radar-side head FLIPPED (rotated 180 deg about the reference axis).
   Tighten it the way it will be tightened in the test.
3. Hands off both tripods. Record 60 s.
4. Rotate the head back to NORMAL and tighten. It stays in this seat for the
   test: do not re-seat it after this step.
5. Record 60 s.
6. `delta = (normal + flipped) / 2`, both readings in the head's own frame.
   `(normal - flipped) / 2` is where the target is, as a by-product.
7. The script writes `calib/datum.yaml`; the controller refuses to start
   without a calibration.

```
python scripts/calibrate_reversal.py --sim --distance 2 --seconds 60
```

`--sim` is the only mode today (writes `calib/datum.sim.yaml`). The real-camera
mode needs the vision driver from task 4.

### What limits it

| Limit | Size | What to do |
| --- | --- | --- |
| Seat repeatability. Each seating has its own error; the estimate is off by the mean of the two, `sigma_seat / sqrt(2)`. The reversal cannot see it. | 0.0071 deg for a 0.010 deg seat, 0.0035 deg for 0.005 deg | Three-point (kinematic) seat; measure the seat by repeating the reversal 5 times |
| Wind during the recording | 60 s per position: ~0.002 deg. 10 s: ~0.005 deg, and the uncertainty the script reports is then too optimistic | Record 60 s, not 10 s |
| Range, in the flipped position only. The camera is then on the other side of the axis and the matched-offset cancellation is lost (baseline 20 cm instead of ~0). | 12 m with +/-5 cm: negligible. 2 m with +/-5 cm: 0.075 deg. 2 m with 2 mm: 0.006-0.008 deg total | At tabletop distance measure the range with a tape to 2 mm |
| The radar tripod must not move between the two recordings | not modelled | Handle the head gently; repeat if the tripod was bumped |

### Accuracy reached in simulation (RMS error of `delta`, deg)

| Distance | Range knowledge | Seat sigma | Time per position | RMS az | RMS el | Reported `u_delta` |
| --- | --- | --- | --- | --- | --- | --- |
| 12 m | +/-5 cm | 0.010 | 10 s | 0.0089 | 0.0092 | 0.0074 |
| 12 m | +/-5 cm | 0.010 | 60 s | 0.0078 | 0.0074 | 0.0074 |
| 12 m | +/-5 cm | 0.005 | 60 s | 0.0046 | 0.0042 | 0.0041 |
| 2 m | +/-5 cm | 0.010 | 60 s | 0.0752 | 0.0073 | 0.0074 |
| 2 m | tape 2 mm | 0.010 | 60 s | 0.0084 | 0.0073 | 0.0074 |
| 2 m | tape 2 mm | 0.005 | 60 s | 0.0059 | 0.0042 | 0.0041 |

Reading the table:
- At 60 s the reported uncertainty matches the achieved error within about 10%.
  At 10 s it is 20-50% too optimistic (wind is correlated over seconds and ten
  seconds is too short to see that).
- The 2 m row with +/-5 cm range is wrong by 0.075 deg in azimuth while the
  script still reports 0.0074 deg: the reported uncertainty does NOT include the
  range term. On the table, use a tape measure.
- These match the arithmetic in docs/REVIEW.md (0.0083 and 0.0040 deg).

### What it buys (Monte Carlo, 500 runs at 12 m)

| Calibration | Reported pass | True pass | False accepts | Pass incl. U_bias |
| --- | --- | --- | --- | --- |
| Assumed in config.yaml (0.015 + 0.010 fixture) | 100.0% | 35.2% | 64.8% | 0.0% |
| Reversal, flat clamp, 60 s (0.0078) | 100.0% | 72.0% | 28.0% | 6.0% |
| Reversal, kinematic seat, 60 s (0.0044) | 100.0% | 86.4% | 13.6% | 33.0% |

The horn-side calibration is set to the same value in these rows, which is an
assumption (section 3). Wind is unchanged at the assumed 0.015 deg, and it is
what keeps the true pass rate below 100% and the strict pass rate low.

## 2. Ground-truth method (independent check, not a calibration)

Purpose: evidence that the measured angle is right, from something that is not
our own camera model.

1. Put the horn mock-up on a ruler or micrometer stage at a known distance `d`
   (tape, 2 mm).
2. Align. Then move the stage sideways by a known `x`: 1, 2, 5, 10 mm.
3. The radar error must change by `atan(x / d)`: at 2 m, 1 mm is 0.0286 deg and
   the 1.75 mm tolerance is 0.05 deg.
4. Repeat 5 times per step, both directions (backlash), then the same vertically.
5. Pass: measured minus expected below 0.01 deg (validation test 2, task 9).

This checks scale and linearity. It does not check `delta`: a constant offset
moves both the start and the end point.

## 3. Horn head (not solved)

Nothing ties the horn's camera to the horn's boresight yet. Candidate: a flat
mirror on the horn's aperture flange, seen by the radar-side camera as a
reflection of the radar-side beacon.

Verified so far, geometry only (`scripts/reversal_study.py`, exact reflection):
a flange tilt of theta moves the reflection by theta (factor 1.000 at 12 m and
2 m with matched offsets; 0.9994 at 2 m with unmatched ones). It is not 2 theta,
because the beacon is beside the camera and not a collimated beam.

Not verified: the mirror in the full simulator, how flat the mirror sits on the
flange, and whether the beacon is bright enough over twice the distance. Until
then the horn-side `u_delta` is an assumption (0.015 deg in config.yaml).

## 4. Offsets

Camera and beacon offsets from each head's reference point are entered in
config.yaml. The rig rule is matched offsets; what matters is the mismatch.
Measure it with a caliper after the heads are built: 1 mm of unknown mismatch is
0.0048 deg at 12 m and 0.029 deg at 2 m.
