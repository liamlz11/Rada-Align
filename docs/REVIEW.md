# Design review (red team)

Written from the seat of a senior radar test-range engineer judging the
hackathon. Only weak points are listed; anything not mentioned is fine.

Where the numbers come from:
- **[sim]** `rada_align` simulator, 500 runs at 12 m, seed 0, scored against the
  simulated truth. All component values are ASSUMPTIONS from config.yaml.
- **[arith]** plain arithmetic.
- Both are reproduced by `python scripts/review_numbers.py`.
- No DRS RADA figure is used anywhere. Radar datum, aperture size, band and horn
  beamwidth are unknown to us; where the answer depends on them it says so and
  points to docs/OPEN_QUESTIONS.md (Q1-Q10).

## 1. The five weakest points

### W1. The system cannot see its own largest error (datum transfer)

**Mechanism.** The radar-side camera measures angles relative to its own optical
axis. The requirement is relative to the radar's mechanical reference axis. The
link between the two is one number per axis, `delta`, from calibration, plus
how repeatably the head seats on the datum. Any error in that link is a constant
offset. Averaging does not reduce it, and the system reports "aligned" with full
confidence while the horn sits off-axis.

**Size.**
- Assumed today: calibration residual 0.015 deg and fixture repeatability
  0.010 deg, RSS 0.018 deg [arith].
- With those values the system declares ALIGNED in 100.0% of runs, but only
  35.2% truly meet `|mean| + 2*sigma < 0.05 deg`. False accepts: 64.8% [sim].
- With no calibration or fixture bias at all the true pass rate is 97.6%, so
  this one link is responsible for most of the gap [sim].
- It degrades fast: calibration residual 0.030 deg gives 6.0% true pass, fixture
  repeatability 0.030 deg gives 16.2% [sim].
- A seating gap of 10 um across a 100 mm base is already 0.0057 deg; 30 um is
  0.017 deg; 50 um is 0.029 deg [arith]. A hand-tightened clamp on a painted or
  dusty surface is in that range.
- Sun on one side of the bracket: 5 K across a 50 mm wide, 100 mm long aluminium
  bracket is 11.5 um, i.e. 0.013 deg [arith]. This is not in the budget at all.

**Cheapest fix by Monday.**
1. Reversal calibration on every setup, not once in the lab: measure, rotate the
   head 180 deg about the datum axis, measure again. What it does and does not
   remove [arith, using the assumed wind]:
   - Camera-to-fixture offset, including a slow thermal bend of the bracket, is
     measured directly: 0.0043 deg with 10 s per position, 0.0019 deg with 60 s.
   - Seat error is NOT removed. Each seating has its own error, and the estimate
     is wrong by their mean, i.e. sigma_seat / sqrt(2): 0.0071 deg for a
     0.010 deg seat, 0.0035 deg for 0.005 deg.
   - Total: 0.0083 deg (0.010 seat, 10 s), 0.0040 deg (0.005 seat, 60 s).
   - It assumes the radar tripod does not move while the head is handled.
   - In the flipped position the camera is on the other side of the axis, so the
     matched-offset cancellation is lost (baseline 20 cm). A 5 cm range error
     then costs 0.0020 deg on delta at 12 m but 0.069 deg at 2 m; on the table
     the range must be known to about 2 mm (0.0028 deg).
   So "0.005 deg with reversal" is a target that needs the kinematic seat and
   60 s per position. It is not demonstrated: task 5 has to simulate it.
2. Three-point (kinematic) seat on the fixture instead of a flat clamp.
3. Report the criterion honestly: show `|mean| + 2*sigma + U_bias`, where
   `U_bias` is the calibration uncertainty from the reversal, and show it on the
   dashboard. Without that term the green light is not evidence.
4. Whether a reversal is physically possible depends on the datum (Q1). If the
   datum is a flat face with no rotation axis, we need a different transfer
   (for example an optical cube or a mirror on the face) and must say so.

### W2. Wind alone uses 60% of the tolerance, and there is a cliff just above it

**Mechanism.** The pass criterion adds `2*sigma` of the individual samples. That
sigma is real motion of the tripod heads. Averaging longer does not shrink it,
it only measures it better.

**Size.**
- Assumed wind 0.015 deg per head: `2*sigma` = 0.030 deg = 60% of the tolerance,
  leaving 0.020 deg for `|mean|` [arith].
- A 10 s verify window instead of 5 s does not help: 32.4% true pass against
  35.2% [sim].
- Halving the wind raises the true pass rate from 35.2% to 77.0% [sim].
- 1.5x the assumed wind: the system declares ALIGNED in only 16.2% of runs.
  2x: 0.0%, every run ends in FAULT "unstable" [sim].
- So the design works only in a narrow band of conditions, and the assumed
  0.015 deg has no measurement behind it.

**Cheapest fix by Monday.**
1. Measure it: `scripts/measure_stability.py` (task 4) on a real tripod, indoors
   and outdoors, and replace the assumption.
2. Mechanical: low and wide tripod stance, weight bag on the centre hook, heads
   close to the tripod apex.
3. Ask the mentors what the +/-0.05 deg applies to (Q3, Q8): the instantaneous
   pointing or the mean over a measurement. If the radar integrates over
   seconds, the fair statistic is the mean with its confidence interval, and the
   wind term mostly drops out. Until answered, keep the stricter reading.
4. Keep the FAULT. Refusing to declare alignment in unstable conditions is the
   correct behaviour and a selling point (R10).

### W3. "Horn center" and "horn axis" are not tied to anything measured

**Mechanism.** The radar camera sees the horn's beacon, not the horn. The horn
camera measures relative to its own optical axis, not the horn's boresight. Both
are converted with mechanical offsets and a horn-side `delta` that we assume are
known exactly. No task measures them: task 5 calibrates the radar side only.

**Size.**
- An error in the known beacon-to-horn-center offset is a direct, invisible
  offset of the horn: 1 mm is 0.0048 deg at 12 m, 2 mm is 0.0095 deg. At the 2 m
  tabletop 1 mm is 0.029 deg and 2 mm is 0.057 deg, which alone fails the demo
  [arith].
- The horn-side camera-to-boresight error enters the horn error one-to-one. The
  simulator assumes 0.015 deg for it with nothing behind the number.
- How much horn pointing matters depends on the horn. For a pattern with a
  10-30 deg 3 dB beamwidth the gain loss at 0.05 deg is 0.0003-0.00003 dB, and
  even at 1 deg it is 0.12-0.013 dB [arith, beamwidth is a placeholder, Q5]. If
  that holds, the horn-pointing requirement can be far looser than the radar
  error requirement and the budget should be spent on the radar side (Q3).
- "Horn center" may mean aperture center or phase center (Q5). They lie on the
  same axis, so the difference only matters through horn pointing error.

**Cheapest fix by Monday.**
1. Build the heads with matched offsets (already the rig rule in config.yaml)
   and measure the residual mismatch with a caliper to 0.1-0.3 mm; put the
   number in config.yaml.
2. Check the horn's mechanical face directly with a flat mirror on its aperture
   flange, seen by the RADAR-side camera as a reflection of the radar-side
   beacon. (The horn's own camera looks the same way as the flange and cannot
   see it.) A flange tilt of theta moves the reflection by theta, not 2 theta,
   because the beacon is next to the camera and not a collimated beam; 0.3 px
   resolves 0.0011 deg [arith]. The difference between this reading and the
   horn camera's reading is the horn-side `delta`. Limits: it only works near
   alignment (the reflection point moves 10.5 mm per 0.05 deg at 12 m, 209 mm at
   1 deg), and the mirror must sit flat on the flange. UNVERIFIED: geometry by
   arithmetic only, to be simulated in task 5 before anyone relies on it.
3. Add this as a task (see proposals at the end).

### W4. The tabletop demo does not scale the way the angles do

**Mechanism.** At 2 m the tolerance is 1.75 mm instead of 10.47 mm. Every error
that is fixed in millimetres becomes six times larger in angle. A judge will ask
whether a 2 m demo proves anything about 12 m, and whether the demo itself is
inside tolerance.

**Size.**
- Offset knowledge: see W3 (1 mm = 0.029 deg at 2 m).
- A 5 mm LED subtends 0.143 deg at 2 m. If its centroid moves by 5% of its
  diameter with viewing angle, that is 0.0072 deg at 2 m against 0.0012 deg at
  12 m [arith, the 5% is a placeholder].
- With unmatched 10 cm offsets and +/-5 cm range knowledge the tabletop missed
  completely: true residual p95 0.157 deg [sim, before the rig rule]. With
  matched offsets it is 0.040 deg [sim].
- The measurement model has a second-order term that is zero when aligned but
  reaches 0.0077 deg at 2 m during convergence (0.0005 deg at 12 m) [sim].
- A 25 mm lens focused for 12 m is out of focus at 2 m. Blur is acceptable for a
  centroid, but saturation and blob size limits must be set for the demo
  distance.

**Cheapest fix by Monday.**
1. Matched offsets (done in config, must be built that way).
2. A small aperture in front of the LED (1 mm pinhole) for the tabletop.
3. Ground truth on the table: a ruler or micrometer stage under the horn mock-up,
   move it by a known amount and show the measured angle changes by
   `atan(move / d)` (validation test 2 in task 9). This is the only independent
   evidence the demo offers, so it is not optional.
4. Say it plainly in the pitch: the demo proves the angular measurement chain
   and the closed loop; field stability and the datum transfer are shown by
   simulation and still need a field test.

### W5. Continuous tracking (R11) conflicts with the RF test and with the tolerance

**Mechanism.** Three separate problems.
1. The radar reference head is a metal object near the radar aperture. The
   design says "outside the aperture or removable after alignment". If it is
   removed, there is no radar-side measurement during the test and R11 is not
   met. If it stays, it can reflect.
2. Corrections during a radar measurement move the target the radar is looking
   at (Q8).
3. The tracking rule lets the mean drift to 0.03 deg before correcting. With
   `2*sigma` = 0.03 deg that is 0.060 deg, above the tolerance, without any
   correction and with the system still in TRACK [arith].

**Size.** Items 1 and 2 are not angles we can compute; they are acceptance
questions for RADA (Q8, Q9). Item 3 is a logic gap of up to 0.010 deg beyond the
tolerance, and in simulation the tracker re-aligned 0.18 times per run in a 10 s
tracking period, so it is active, not dormant [sim].

**"No radar data" (R8).** Nothing in the design reads radar data, and the
simulator and tests have no radar input at all. The risk is not a violation but
a proof gap: we must be able to show a full run with the radar powered off
(validation test 6). Lab calibration against a theodolite does not use radar
data either.

**Cheapest fix by Monday.**
1. Track on the criterion itself: correct when `|mean| + 2*sigma` approaches the
   tolerance and `|mean|` is above the deadband, instead of a fixed 0.03 deg.
2. A "hold" input: no motor motion while the radar measures, corrections only
   between measurements, and every correction logged with a timestamp so the
   radar data can be tagged.
3. Mount the reference head on an arm that puts it clearly outside the aperture.
   The matched-offset rule means a long arm costs no accuracy. How far is "clear"
   is Q9.

## 2. Error-budget entries that are optimistic

The preliminary table (docs/CHALLENGE.md) lists 1-sigma values and sums them in
RSS to 0.0248 deg, called "2x margin". The pass criterion is
`|mean| + 2*sigma`, so 2 x 0.0248 = 0.0497 deg: there is no margin [arith]. The
table also mixes two kinds of error that behave differently: biases (invisible,
constant) and noise (visible, and counted twice by the criterion).

| Entry | Preliminary | Problem | Assume instead until measured |
| --- | --- | --- | --- |
| Camera-to-datum calibration | 0.015 | Optimistic unless reversal is done per setup. It is a bias. | 0.008 with per-setup reversal on a flat clamp, 0.004 if a kinematic seat and 60 s per position are achieved (W1, not demonstrated); 0.03 or worse without a rotation axis |
| Fixture repeatability | 0.010 | 10 um over 100 mm is 0.0057; a hand clamp is 30-50 um, 0.017-0.029. Reversal only reduces it by sqrt(2) | 0.02 for a flat clamp, 0.005 for a kinematic seat (placeholder, to be measured) |
| Wind and settling | 0.015 | No measurement behind it; counted as `2*sigma` it is 0.030; cliff at 1.5x | Measure. Plan for 0.0075 indoors, treat 0.02+ outdoors as "do not declare alignment" |
| Actuator residual | 0.005 | Simulated control floor is 0.0085 at 12 m and 0.0118 at 2 m, set by the 0.01 deg deadband, not by the motors | 0.009 |
| Thermal drift of the head | missing | 5 K across the bracket is 0.013 | 0.01 until shaded or measured |
| Offset knowledge (beacon/camera to reference point) | missing | 1 mm is 0.0048 at 12 m, 0.029 at 2 m | 0.005 at 12 m with caliper-measured offsets |
| Horn camera to horn boresight | missing | Not calibrated by any task | 0.015 until a horn-side calibration exists (mirror method is unverified) |
| Parallax (distance error) | 0.002 | Fine at 12 m only with matched offsets: 0.0031 above the floor | 0.003 |

Overstated entries, which we can say so about: the inclinometer (0.005) adds
0.0000 deg in this architecture because elevation is measured optically and the
inclinometer is only used for roll; the centroid (0.002) adds 0.0000 deg after
averaging [sim].

What it would take to pass, in simulation: IF calibration and fixture were both
0.005 deg the true pass rate would be 82.2%; halving the wind as well gives
98.8% with 1.2% false accepts [sim]. These are targets, not results: nothing
yet shows we can reach 0.005 deg.

## 3. Eight questions a judge would ask

Answers marked [after P<n>] describe something that is NOT built yet and depends
on proposal n below being approved and implemented. Do not use them as they
stand in the pitch or the demo script until then.

**1. How do you know the camera axis is the radar's mechanical axis?**
[after P3 = task 5] By reversal: the head is measured in two orientations 180 deg apart about the
datum axis, the mean is the true axis and half the difference is the camera
offset. It is repeated on every setup and its uncertainty is added to the
reported result. Seat repeatability is not removed by this and is budgeted
separately. What we still need from you is which datum feature we may use.

**2. Your screen says 0.01 deg. Why should I believe it?**
[the ground-truth check is task 9] You should not believe the screen alone, and we do not ask you to. Our simulator
scores the system against the true geometry, and with our first assumptions the
system was right only 35% of the time it said "aligned", because of the datum
transfer. That is why the result is reported with a bias term and checked
against an independent reference: a ruler or micrometer on the table, a total
station in the field.

**3. Do you use any radar data?**
No. The inputs are two cameras, two inclinometers and a rangefinder. The whole
alignment runs with the radar switched off, and that is one of our validation
tests.

**4. What happens in wind?**
The pass criterion includes twice the measured jitter, so wind shows up in the
number instead of being hidden. If the jitter alone makes the criterion
impossible the system stops and says "unstable" instead of showing a green
light. With our assumed wind this happens at roughly 1.5 to 2 times the nominal
level, which we still have to replace with a field measurement.

**5. Why not GPS, a compass or a laser pointer?**
Compass and GNSS heading are tenths of a degree, an order of magnitude too
coarse. A laser gives one direction only: it shows where the horn is relative to
the radar, not where the horn points. Mutual sighting gives both errors on both
axes, with cameras that resolve 0.0036 deg per pixel before any sub-pixel
processing.

**6. Your demo is at 2 m. The range is 12 m.**
The requirement is an angle and the demo keeps the same angle, so the tolerance
on the table is 1.75 mm instead of 10.5 mm, which is harder mechanically, not
easier. The demo proves the measurement chain and the closed loop. It does not
prove outdoor stability or the datum transfer, and we say which numbers come
from simulation.

**7. Will your hardware disturb the radar measurement?**
[the arm is a task 3 rig item] The radar-side head carries no motors and sits on an arm outside the aperture;
how far outside is your call. The horn-side motors can be held still during a
measurement and only correct between measurements, with every correction
time-stamped.

**8. How long does it take, and what does the operator do?**
In simulation about 22 s on average and 31 s at the 95th percentile from a
+/-1 deg manual setup, with no operator action after pressing start. The
operator sees the error before, the residual after, and a clear aligned or
fault state with the reason.

## Proposed changes

STATUS: all seven approved by Liam on 2026-11-14 and applied (P1, P2, P4 in code; P3, P5 in
docs/TASKS.md; P6 in CLAUDE.md; P7 in docs/OPEN_QUESTIONS.md). The W5 item about the fixed
0.03 deg drift threshold describes the controller before P1.

Ordered by cost against the Monday deadline.

Documents only:
- **P6.** CLAUDE.md: record the matched-offset rig rule; record that Monte Carlo
  is scored against truth and that results must quote the true pass rate and
  false accepts next to the reported pass rate.
- **P7.** OPEN_QUESTIONS.md: add (11) does the datum allow a 180 deg rotation
  about the reference axis, and (12) does the radar integrate over seconds, i.e.
  does +/-0.05 deg apply to the mean or to the instantaneous pointing.
- **P5.** Task 4: run `measure_stability.py` on a real tripod as early as
  possible and replace the wind assumption in config.yaml.

Changes the scope of tasks not yet started:
- **P3.** Task 5: reversal on every setup instead of a one-time lab calibration;
  simulate it with independent seat errors and the flipped-position parallax;
  add the horn-head calibration (flange mirror, after it is verified in the
  simulator) and the caliper-measured offset mismatch in config.yaml.

Reopens task 2 code:
- **P1.** Controller: track on the criterion (correct when `|mean| + 2*sigma`
  approaches the tolerance and `|mean|` is above the deadband) instead of the
  fixed 0.03 deg drift threshold.
- **P2.** Reporting: add `U_bias` (calibration uncertainty) to the criterion in
  the status message and docs/messages.md; the dashboard shows it.
- **P4.** Controller and dashboard: a "hold" input (no motor motion while the
  radar measures), every correction time-stamped.
