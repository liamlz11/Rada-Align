# Final design: Raster Scan + PSD + KLT

Status: closed by the team on 2026-11-16 (Liam). This file is the single source of
truth for the design. It supersedes the "mutual sighting" architecture (camera +
IR beacon as the measuring sensor on each side, radar fixed, horn on X/Z/Pan/Tilt
steppers with an ESP32). That code and those documents stay in the repo and are
marked superseded; nothing is deleted.

Sources: Liam's design D1-D12 (2026-11-16), the whiteboard
`docs/slides/06-final-design-whiteboard.jpg`, the mentor answers in
`docs/MENTOR_FEEDBACK.md`, and the existing simulator parameters in `config.yaml`.

Tags used for every number:
- **[team]**: a value from the team's design.
- **[repo]**: a value from config.yaml or the docs.
- **[datasheet]**: read from the manufacturer's datasheet on 2026-11-16, with the
  link in section 9.
- **[vendor page]**: read from a manufacturer's product page.
- **[arith]**: computed by hand, with the formula shown.
- **[sim]**: from the numerical 3D model in the numbers script, which is planned
  as `scripts/final_design_numbers.py` and is added with the doc diff.
- **ASSUMPTION**: a placeholder to replace with a datasheet value or a measurement.
- **[unverified]**: a spec or price not confirmed on a datasheet.

The rule "NEVER use radar data" is unchanged.

---

## 0. Team decisions on the open points (Liam, 2026-11-16)

| Point | Decision | Closes |
| --- | --- | --- |
| PSD type | Continuous 2D PSD (S5991-01 class), not a quadrant detector. Field size for A and C still per 7.1 (≥ 20 mm or diffuser) | C6, Q26 |
| Laser board | Adjusted by hand (micrometer) in CALIB only, then locked. The whiteboard's "3 stepper motors" are not a separate laser drive | C5, Q29 |
| Raster scan | With the horn Pan/Tilt servos; no separate scanner. BOM item 7 = 0. **Superseded by 0.1: with the lasers on the radar the raster uses the radar Pan/Tilt (approved by Liam, 2026-11-16)** | D12, Q24 |
| Radar motion | Before the test and in allowed gaps only. During measurement it is braked and monitored, with an alert above 0.03°. The gap signal is still Q21 | C1, Q22 |
| Servos | Not chosen. We propose 2-3 models from datasheets in task 3. Code is written against the mock (task 17) | Q25 (in progress) |
| Laser | Per recommendation: 960 nm, Class 1, focusable 10 mm beam, attenuated to about 0.1 mW. Models and the Class to be verified from an official source in task 3 | C7, Q27 (open: does the camera see 960 nm?) |
| Controller | Laptop for the demo, industrial PC for the field. Lock-in on an ADC card or a small microcontroller, cabled | Q28 |
| Radar tilt without X/Z | Ask DRS. Until then R3 is measured and reported | C2 stays open, Q23 |

Still proposals that the doc-diff OK would approve: C2 (pending Q23), C8, C9,
C10, C11.

**Superseded by 0.1:** the layout below (lasers on the horn, PSDs on the radar) and
the raster-scan row above (horn servos). Sections 3, 4, 5 and 7 describe the
earlier layout where not updated; the physics carries over with the two sides
swapped.

## 0.1 Layout swap: lasers and camera on the radar, PSDs on the horn (Liam, 2026-11-16)

**Decision:** the 3 lasers (on their common board) and the camera move to the radar
side; the 3 PSDs (A, B with its lens, C) move to the horn. **Why:** the initial manual
aiming is done at the radar, and a laser on the horn would point at the person
aiming it. With the lasers on the radar they point away from that person, toward
the horn.

**What flips** [sim, `rada_align/psd/model.py`, 12 m, mm per deg of each error]:

| PSD (now on the horn) | Reads | Gives |
| --- | --- | --- |
| A, no lens | A_y = 205.9·er_az + 1.75·roll, A_z = −205.9·er_el − 2.62·roll | **radar error directly** |
| B, lens | B = 0.873·(eh − er) per axis | horn error = B/f + er |
| C, no lens | C_y = 205.9·er_az − 1.75·roll, C_z = −205.9·er_el | roll from C − A, redundancy |

- **Observability is unchanged:** rank 5, all 5 errors observable [sim, tests/test_psd.py].
- **The critical radar error is now read by a lensless PSD 11.8 m away.** That
  gives 205.9 mm per degree, so PSD noise is negligible.
- **Datum transfer moves to the laser board.**
  - The laser board on the radar datum must be calibrated parallel to the radar
    mechanical axis, and that error goes 1:1 into the radar error.
  - Before the swap it was the PSD-B null on the radar.
  - The budget row is the same: 0.015° datum and 0.010° seat. In config.yaml the
    calibration residuals are now keyed by mounting point
    (`calib.radar_datum_sigma_deg`, `radar_seat_sigma_deg`, `horn_axis_sigma_deg`)
    instead of by component.
- **Raster scan now uses the RADAR Pan/Tilt, not the horn's.** Turning the horn
  barely moves the spot on its own PSDs (only the 0.1 m lever), so only the
  emitter side can sweep the beam.
  - This replaces the team decision "raster with the horn servos" (Q24). Approved
    by Liam, 2026-11-16.
  - Scanning happens before the test, while the radar may still move (C1).
- **ACQUIRE:** the camera on the radar finds the PSD unit on the horn (template
  matching), and the radar coarse-points.
- **TRACK** (radar braked):
  - The horn may still correct its own error (B + A).
  - If the radar moves (wind, settling, a knock), the spot drifts on the horn PSDs
    and only the radar can fix it. The system raises an alert ("radar moved", or
    "beam lost") and corrects only in an allowed gap: radar raster, then ALIGN,
    VERIFY, brake.
  - Simulated: after a 0.08° knock the spot leaves the 20 mm PSD; with the gap
    allowed the system recovers to errors < 0.001° [sim, tests/test_psd.py].
- **Eye safety:** the beams now point at the horn. Class 1 at 960 nm, attenuated
  to about 0.1 mW (C7), is unchanged.
- **Spot size vs PSD size (7.1) is unchanged in form.** It is now radar head motion
  (wind, settling) that moves the spot on the horn PSDs.

Monte Carlo against truth after the swap, 200 runs each
(`python -m rada_align.psd.montecarlo --runs 200 --distance D`, assumed parameters):

| d | reported pass | TRUE pass | false accepts | p95 true score (eh_az, eh_el, er_az, er_el) |
| --- | --- | --- | --- | --- |
| 2 m | 98.5 % | 23.0 % | 75.5 % | 0.067, 0.065, 0.0754, 0.0749 |
| 12 m | 99.0 % | 43.5 % | 55.5 % | 0.0548, 0.0559, 0.0695, 0.0642 |
| 15 m | 93.5 % | 43.0 % | 50.5 % | 0.0556, 0.0525, 0.0627, 0.0622 |

- **The dominant cause is the calibration residuals** (datum, seat, horn axis), which
  the PSDs cannot see. Run at 12 m with them switched off:
  - reported pass 99.0 %, TRUE pass 96.0 %, false accepts 3.0 %
  - p95 true score 0.0438-0.0464°
- **STOP gate of task 14:** true pass is below reported, and p95 > 0.05°. No
  parameter was tuned.
- **Resolution (Liam, 2026-11-16):** the system now also reports score + U_bias
  per axis, and a strict state "aligned including calibration uncertainty".
  - U_bias (k = 2) is propagated from the calibration sigmas through the
    estimator and matches sampling within 15 % [tests/test_psd.py].
  - At 12 m, U_bias = 0.037° on the radar axes and 0.024° on the horn axes
    (A+B+C) [sim].
  - Monte Carlo, 200 runs at 2/12/15 m: pass incl. U_bias 0.0 %, false accepts
    incl. U_bias 0.0 %.
  - The strict state never lies, but it is unreachable with these assumptions:
    0.037 + 2σ wind (about 0.03) > 0.05 [arith].
  - To pass strictly, the radar-side calibration must reach roughly
    U_bias < 0.05 − 0.03 = 0.02°, i.e. a 1-sigma datum+seat residual of about
    0.01° instead of 0.018° [arith], or the wind sigma must be smaller (measure it,
    task 9.8). The reported pass does not include the calibration
  uncertainty (U_bias), which is the same lesson as the old design (PROGRESS,
  task 5).

---

## 0.2 Units below the axes, height difference, field conditions (Liam, 2026-11-16)

Liam: the laser board and the PSD unit hang BELOW the radar and the horn, not on them; the demo shows a radar that stands higher than the horn; the demo shows that light rain and other field conditions do not stop the alignment.

**Units below.** `psd.geometry.laser_*_m` now has negative "up": A and B 0.40 m, C 0.58 m below the radar reference; the PSDs sit at the mirror positions below the horn centre (same drop on both sides, so the matched pattern holds). Both units stay rigid parts of their side (the radar unit is still clamped to the radar datum, on a longer bracket; its calibration sigma is unchanged and is an ASSUMPTION).

- Consequence [sim]: a relative roll error now reaches the azimuth readings through the 0.4-0.58 m lever. With roll taken from PSD A/C, U_bias (k=2) at 12 m rose from 0.0237/0.0369 deg (horn/radar Az) to 0.0405/0.0494 deg, and at 2 m to 0.104/0.108 deg.
- Decision (design change, approved by Liam 2026-11-16): the relative roll comes from the two inclinometers (`control.roll_from_inclinometers: true`); PSD A/B/C solve Az/El only. PSD-C is no longer used for roll, but it stays in the Az/El least-squares solution as a third position reading whenever it is valid in the ALIGN window, and then VERIFY needs it whole on its PSD. The U_bias figures below depend on that: with Az/El from A+B only, U_bias at 2 m is 0.040/0.049 deg (measured 2026-11-16, see the 15 m diagnosis). U_bias includes the inclinometer bias through the lever. Result [sim]: U_bias back to 0.0237/0.0369 deg at 12 m, 0.033/0.043 deg at 2 m.

**Height difference (R3).** `psd.geometry.radar_height_m: 0.60` (field) and `radar_height_short_m: 0.10` (tabletop) are demo scenario values. Both sides are aimed by hand along the sloped line of sight (2.9 deg at 12 m). The model linearises at that aligned attitude. The controller does not read the scenario value: it takes the height from the two inclinometer tilts and the range. R3 is reported as (range - levers) * tan(mean of the two tilts, each corrected by its PSD elevation error); example run: 636 mm reported, 634 mm true. The old R3 line had the wrong sign for a radar above the horn.

**Field conditions** (`psd.weather`, every value an ASSUMPTION fixed before the runs; formulas quoted from memory, to be checked against the sources):

| Condition | Model | Effect at 12 m |
|---|---|---|
| Rain 2.5 / 25 mm/h | path extinction 1.076 R^0.67 dB/km; drops on a hooded window: 0.01 events/s per mm/h per window, 1 s mean stay, 2-50 % of the beam scattered, centroid shift 1.0 mm (A/C) or 0.6 mm (B) per unit scattered fraction | path loss 0.5 % / 2.5 %; a wet window dims one PSD and biases it |
| Fog, visibility 200 m | extinction 3.91 / V per metre | all three beams lose 21 % together; PSD noise up by 1/0.79 |
| Heat shimmer, Cn2 = 1e-13 | beam wander and angle of arrival, 10 ms correlation | 0.05 mm on A/C, 0.0002 deg on B |
| Direct sun | PSD noise x2 after filter and lock-in | noise only |

- Design additions: amplitude gate (`sensor.amplitude_gate: 0.95`): a sample is used only if its lock-in amplitude is at least 95 % of the strongest of the three PSDs, so a drop on one window drops that sample while fog (all three alike) does not. A dimmed PSD is not treated as "spot leaving the PSD" (no false radar alert, no re-centre). VERIFY also needs `control.min_clean_fraction: 0.5` of samples free of a dip.
- The weather has its own random generator, so clear and rainy runs of the same seed start from the same setup. The sensor noise is shared only until the first dropped sample (from then on the main random stream advances differently), so the rows are not a paired comparison; differences of 2-3 points between rows are noise.
- Not modelled: rain or fog on the camera (acquisition and KLT), water film on a window, wind-driven spray, wet-ground settling. The drop model is the weakest part: the hit rate and the centroid shift are estimates, to be measured with a wet window on the bench.

Monte Carlo, 200 runs at 12 m, same seeds, all error sources on [sim]:

| Condition | Reported pass | TRUE pass | False accepts | Median time to TRACK | Faults |
|---|---|---|---|---|---|
| clear | 100.0 % | 44.0 % | 56.0 % | 11.2 s | 0 |
| light rain | 99.5 % | 45.5 % | 54.0 % | 11.3 s | 1 |
| heavy rain | 93.0 % | 40.0 % | 53.0 % | 16.3 s | 14 |
| fog | 100.0 % | 45.0 % | 55.0 % | 11.3 s | 0 |
| heat shimmer | 99.5 % | 47.5 % | 52.0 % | 11.3 s | 1 |
| direct sun | 100.0 % | 44.5 % | 55.5 % | 11.3 s | 0 |

Pass including U_bias is 0.0 % in every row (unchanged finding: the assumed radar-side calibration is the limit, section 0.1). Reading: light rain, fog, shimmer and sun do not change the result in this simulation; heavy rain costs 7 points of reported pass and 5 s, from VERIFY windows with too many dips and ALIGN runs that do not converge. Clear weather at 2 / 12 / 15 m after the two geometry changes: reported 98.0 / 100.0 / 88.0 %, TRUE 24.0 / 44.0 / 44.0 %, false accepts 74.0 / 56.0 / 44.0 %.

Holding TRACK, 20 seeds at 12 m, weather on from the start, then 60 s after first reaching TRACK [sim]:

| Condition | Reached TRACK | Faults | A+B samples used | Time with an alert | p95 TRUE worst axis |
|---|---|---|---|---|---|
| clear | 20/20 | 0 | 95.1 % | 2.3 % | 0.0539 deg |
| light rain | 20/20 | 0 | 90.7 % | 2.1 % | 0.0527 deg |
| heavy rain | 20/20 | 0 | 61.4 % | 2.2 % | 0.0556 deg |
| fog | 20/20 | 0 | 95.5 % | 2.6 % | 0.0536 deg |
| heat shimmer | 19/20 | 1 | 95.6 % | 3.2 % | 0.0530 deg |
| direct sun | 20/20 | 0 | 95.7 % | 2.3 % | 0.0538 deg |

Reading: rain costs samples (a drop on a window drops them), not accuracy; the p95 TRUE error is the same in every row and is set by the calibration residuals, as in clear weather (section 0.1). With 20 runs, a 1-run difference is noise.

15 m regression check, 200 runs, clear [sim]. Before the units moved below, 15 m gave reported 93.5 % / TRUE 43.0 % / false accepts 50.5 %, 10 faults "too few valid samples":

| 15 m, units below | Reported pass | TRUE pass | False accepts | "Too few valid samples" faults |
|---|---|---|---|---|
| roll from PSD A/C | 91.5 % | 35.0 % | 56.5 % | 15 |
| roll from inclinometers (chosen) | 88.0 % | 44.0 % | 44.0 % | 21 |

Reading: the changes since that run (units below, 0.60 m height, amplitude gate; not separated) together add 5 faults. The inclinometer roll then trades about 3.5 points of reported pass (6 more faults) for 9 points of TRUE pass and 12.5 fewer points of false accepts. The fault means too many samples were not valid in the VERIFY window. Diagnosis [sim, 200 runs at 15 m, every VERIFY sample with the spot not whole on a PSD in use]: with roll from PSD A/C, PSD-C alone at its edge caused 17 % of those samples; with roll from the inclinometers 25 %. PSD-A is involved in most of the rest (alone 24 %, with C 15 %, all three dark 15 %); the radar true error at those samples has a median of 0.041 deg, i.e. the braked radar swaying under the assumed wind against a 20 mm PSD. Likely mechanism for the increase (not measured): the roll servo no longer centres PSD-C, so C sits off-centre by its calibration residuals and reaches the edge sooner. Variant tested and rejected: Az/El from A+B only, C not required. 2 / 12 / 15 m: reported 98.0 / 100.0 / 98.0 %, TRUE 17.5 / 43.0 / 41.5 %, false accepts 80.5 / 57.0 / 56.5 %, U_bias at 2 m 0.040/0.049 deg instead of 0.033/0.043, 4 new ALIGN faults at 2 m. Of the 24 runs that fault at 15 m on the current code, the variant brings 23 to TRACK and only 7 of those truly pass, so the FAULT was mostly the right outcome. Kept: C in the Az/El solution and required in VERIFY when used. Binomial noise at 200 runs is about +/-2 points (1 sigma) on the reported pass and +/-3.5 points on the TRUE pass, so the TRUE gain is suggestive, not firm.

Edge clips in TRACK [sim, pre-existing]: in clear weather at 12 m the braked radar sways 0.05-0.09 deg under the assumed wind, and the spot leaves the 20 mm PSD-A (+/-0.048 deg) for one or two samples, 3-90 times a minute (10 seeds; the same before today's changes). Rain does not cause them. The controller bridges them with KLT as before; the log now counts losses shorter than `control.short_loss_s` and prints one summary line per `short_loss_summary_s` instead of one "beam back" line each. This is the field-PSD-size question (section 7.1).

## 0.3 Requirement 0.005 deg, demo profile, NanoTrack, 2-of-3 capture (Liam, 2026-11-17)

- Requirement R7 tightened to 0.005 deg (the mentor's request; DRS value 0.05 deg). Controller thresholds scaled x0.1: deadband 0.001, horn TRACK threshold 0.003, radar alert 0.003 deg.
- With every assumed error source on, no run reaches TRACK: 200/200 at each of 2, 12 and 15 m end in "no convergence in ALIGN" [sim]. One source at a time (40 runs): wind alone 0 %; calibration residuals alone TRUE 0 %; laser-board residual alone TRUE 20 %; drift, PSD noise, inclinometer, rangefinder, camera alone 100 %.
- Demo profile (Liam: remove disturbances if needed): wind, drift, laser and calibration residuals off, sensor noise on (`DEMO_SOURCES`). Monte Carlo `--profile demo`, 200 runs, reported / TRUE / false accepts / incl. U_bias: 2 m 99.5 / 98.0 / 1.5 / 0.0 %; 12 m 100 / 100 / 0 / 96.0 %; 15 m 100 / 100 / 0 / 98.5 %. At 2 m U_bias is 0.0039 deg (inclinometer bias through the 0.4 m lever). The Monte Carlo default stays the full set.
- Reading: the method resolves 0.005 deg; wind on the braked radar and the calibration residuals as assumed do not allow it. A field system at 0.005 deg needs wind shielding / a stiffer stand and a calibration about 10x better than assumed.
- Camera tracker: NanoTrack (lightweight AI tracker, OpenCV TrackerNano with ONNX models) replaces KLT for the backup tracking; same role, modelled the same way.
- Capture: SCAN ends when 2 of the 3 beams are whole on their PSDs; ALIGN uses B plus A, or B plus C when A is off its PSD (both rank 4, same U_bias).
- Tests: the superseded mutual-sighting controller cannot meet 0.005 deg under its own noise; 4 of its tests are xfail(strict) with that reason.

## 1. The design, D1-D12

| # | Decision | Our note |
| --- | --- | --- |
| D1 | Aligned = horn axis and radar axis on one line. Horn error: horn axis vs line to the radar centre. Radar error: radar axis vs line to the horn centre. Both 0 = "calibrated to 0,0". | Same two errors as before (CLAUDE.md). Shown in section 7.2 to be fully observable. |
| D2 | Both sides move. 6 servos with optical encoders: horn Pan/Tilt/Roll, radar Pan/Tilt/Roll. No X/Z. | Conflicts C1, C2, C3. |
| D3 | 3 lasers fixed on one adjustable board on the horn. ASSUMPTION: the board is adjusted by hand (micrometer) in CALIB only. Raster scan uses the horn Pan/Tilt. | Whiteboard step 2 says "3 stepper motors" aim the lasers: conflict C5. |
| D4 | 3 PSDs on the radar at its mechanical reference. PSD-A without a lens: horn error. PSD-B behind a lens: beam angle, i.e. radar error. PSD-C without a lens: relative roll and a cross-check. The target on each PSD is its calibrated point. | Section 4. We propose a change: PSD-B does not need PSD-A at null (conflict C8). |
| D5 | CALIB -> SETUP -> ACQUIRE -> SCAN -> ALIGN -> VERIFY -> TRACK, plus FAULT. | Section 5. |
| D6 | The camera is no longer the measuring sensor. Its roles are coarse acquisition, a KLT backup for tracking, and the picture for the UI. KLT replaces phase correlation. | `rada_align/vision/detect.py` is reused for spot and target detection. |
| D7 | Band-pass filter, modulated laser with lock-in, averaging against heat shimmer. No GPS, no radio; cabled links only. | Section 7.6. |
| D8 | Field-engineer UI: state, the 3 PSD spots with targets, Az/El errors vs threshold, camera with KLT marks, inclinometers, servo/encoder status, sensor health, alerts. No radar data (mentor, Q16). | Task 19. |
| D9 | Tabletop demo at 2 m with hardware, simulation for 10-15 m. | The S5991-01 works at 2 m and not at 12-15 m (section 7.1). |
| D10 | BOM, rough prices. | Section 9. |
| D11 | Laser wavelength matched to the PSD peak (960 nm for the S5991-01), eye-safe Class 1 or 2. | Class 2 exists only for visible light (400-700 nm), so a 960 nm laser can only be Class 1 (or 1M): conflict C7. |
| D12 | ASSUMPTION: raster scan with the horn Pan/Tilt, no separate scanner. | The BOM lists "1 raster scan" as an item: question Q24. |

---

## 2. Geometry

The two errors are the same as before (CLAUDE.md):
- **Radar error (critical):** radar mechanical axis vs line of sight to the horn centre.
- **Horn error:** horn axis vs reverse line of sight.

### Rotations alone null both errors
The radar reference point and the horn centre define one line. Pan/Tilt on the
radar turns its axis onto that line; Pan/Tilt on the horn turns its axis onto it.
Roll aligns the two lateral frames, which the matched laser/PSD pattern needs.

Pivot offsets couple the two sides, because rotating a side moves its reference
point and with it the line. The loop absorbs this in a few iterations [sim, 7.2].

**What changes without X/Z (conflict C2):**
- **R3, height difference.** It is no longer removed; it is measured and reported.
  - Measurement: Δh = d · tan(elevation of the common line vs gravity), with the
    elevation from the inclinometers and d from the rangefinder.
  - Consequence: after alignment the radar boresight is tilted from level by that
    elevation.
  - Size: 25 cm at 10 m gives 1.43° [arith: atan(0.25/10)]. The sim agrees: up to
    1.42° at 10 m, 1.19° at 12 m, 0.95° at 15 m [sim].
- **Servo travel.** It must cover the setup pointing error plus the line elevation:
  - 10 m: 1° + atan(0.25/10) = 2.43° [arith]
  - 12 m: 2.19° [arith]
  - 15 m: 1.95° [arith]
  - DESIGN proposal: ±5° on every Pan/Tilt axis. The 2 m demo with ±50 mm
    placement needs the same 2.43° [arith].
- **R11.** The horn can no longer correct radar drift (conflict C1).

---

## 3. Layout (text sketch, not to scale; offsets are ASSUMPTION)

> Superseded in two steps: the lasers and camera are on the radar and the PSDs on the horn (section 0.1), and both units hang 0.40-0.58 m below their side (section 0.2). The sketch and offsets below are the original layout.

```
 HORN SIDE (servos Pan/Tilt/Roll)                RADAR SIDE (servos Pan/Tilt/Roll)

   laser board (invar, adjustable in CALIB)        PSD unit on the radar datum (outside aperture)
   +-----------------------------------+           +-----------------------------------+
   |  L_B o                   o L_A    |  ----->   |  PSD-A [ ]               [=]PSD-B |
   |   (+150, +100)    (-150, +100)    |  3 beams  |  (no lens)        (lens f, focal  |
   |                                   |  parallel |                    plane PSD)     |
   |              o L_C                |  to horn  |              [ ] PSD-C            |
   |           (0, -100)               |  axis     |             (no lens)             |
   +-----------------------------------+           +-----------------------------------+
        [ horn aperture ]  [cam+zoom]                     [ radar aperture ]
        inclinometer, rangefinder                         inclinometer
   offsets in mm, horn body [left, up]               mirror pattern in the radar body,
                                                     so each beam meets its PSD at null

   |<--------------------------- d = 10-15 m (field), 2 m (demo) --------------------->|
   cabled: PSD ADC + lock-in, servos, camera, lasers -> controller (no radio, no GPS)
```

- The whiteboard puts two PSDs on top and one in the middle below. The offsets above
  follow that layout and are the ones used in the model (ASSUMPTION).
- **Matched pattern (the old rig rule, kept):** each PSD sits at the mirror image of
  its laser, so all targets are PSD centres.
  - Why: a target away from the centre makes the null depend on spot shape and
    roll, which adds bias.
  - The calibrated targets of D4 only absorb build residuals.

---

## 4. PSD roles

> Since section 0.1, PSD-A reads the radar error and PSD-B the horn error. Since section 0.2, the relative roll comes from the two inclinometers; PSD-C is a third position reading in the Az/El solution and a cross-check, not the roll sensor.

| PSD | Optics | Reads [sim, 12 m, mm per deg of each error] | Gives | Drives |
| --- | --- | --- | --- | --- |
| A | none | A_y = 205.9·eh_az + 1.75·roll; A_z = −205.9·eh_el − 2.62·roll | horn error (with C for roll) | horn Pan/Tilt |
| B | lens, PSD in focal plane (f = 50 mm, ASSUMPTION) | B = 0.873·(er − eh) per axis, i.e. f·(beam angle vs radar axis) | radar error = B/f + eh | radar Pan/Tilt |
| C | none | C_y = 205.9·eh_az − 1.75·roll; C_z = −205.9·eh_el | roll from C − A; horn error redundancy | Roll (horn), cross-check |

- **Scale factors:** 205.9 mm/deg = (11.8 m)·π/180. That is the laser-to-PSD
  path: d minus the two 0.1 m levers, ASSUMPTION.
- **B is insensitive to translation.** A sideways shift of either side changes A and
  C but not B. That is why B + A gives the radar error exactly, whatever moved [sim].
- **Least-squares estimator over the 6 readings** [sim, 12 m]:
  - eh_az = (A_y + C_y)/(2·205.9)
  - er_az = B_y/0.873 + eh_az
  - roll from (C − A)
  - Elevation analogous, with its own signs.

---

## 5. State machine

| State | Does | Leaves to |
| --- | --- | --- |
| CALIB (lab, once per build) | Lasers parallel to the horn axis, PSD targets and the PSD-B null vs the radar datum, written to a calibration file. | The controller refuses to run without the file (same rule as today, `CalibrationMissing`). |
| SETUP | Manual placement and coarse pointing until the camera sees the PSD unit. Inclinometers level both rolls. | ACQUIRE |
| ACQUIRE | Template matching finds the PSD unit, KLT tracks it, optical zoom. Closed-loop coarse pointing of the laser board onto the PSDs with horn Pan/Tilt, using the spot when the camera sees it. | SCAN; FAULT on timeout |
| SCAN | Raster scan with horn Pan/Tilt over a window set by the ACQUIRE accuracy, until all three PSDs report a valid spot. Roll search if C is missing. | ALIGN; ACQUIRE if the window is exhausted |
| ALIGN | Outer loop on the PSD estimates, one PID per axis (axis mapping in section 4); encoders close the inner loop of each servo. Horn Pan/Tilt from eh, radar Pan/Tilt from er, horn Roll from C − A, radar Roll held level by its inclinometer. Repeat until all are inside the deadband. | VERIFY; FAULT (unstable, travel limit) |
| VERIFY | 5-10 s with no correction. Pass only if \|mean\| + 2σ < 0.05° in Az and El for both errors. Cross-checks: C redundancy, and inclinometers (radar tilt + horn tilt ≈ 0 on a common line). | TRACK or ALIGN |
| TRACK | PSDs at high rate. Horn: PID allowed. Radar: braked and monitored only (policy C1). Short beam block: KLT holds the horn, radar frozen. Beam lost longer than a set time: SCAN. | ALIGN (horn), alert (radar), SCAN |
| FAULT | Reason and log, as today. | IDLE |

---

## 6. Conflicts with the previous design, with proposals

**C1. Radar on servos vs "radar static, only the horn moves"** (CLAUDE.md,
MENTOR_FEEDBACK update 2).
- **Reconciling reading:** the mentor said "aligned before the test, static during it".
  The system moves the radar only in ALIGN, before the test. During the test the
  radar axes are braked and the servos disabled: no motion, no servo dither.
- **Proposed TRACK policy:**
  - Horn: may correct during the test. Horn pointing changes only the returned power.
  - Radar: PSD-B monitored continuously. If the radar error exceeds 0.03° [repo:
    track_drift_threshold_deg], raise an alert and log it.
  - Radar correction only in an allowed gap, granted by an operator button or a
    "not transmitting" status from the radar PC (Q8, Q21).
- **Consequence for R11:** the radar side has real-time monitoring, with correction
  only in gaps. This needs DRS's answer (Q22).

**C2. No X/Z.** Shown in section 2 and in 7.2 [sim]:
- 6 rotations reach zero error from any setup in the range.
- The PSD readings are zero at that solution: largest reading 3e-9 mm [sim].
- R3 becomes measure-and-report; the radar ends tilted from level by the line
  elevation. Whether DRS accepts this is Q23. If not, a height adjustment (manual
  or motorized Z) comes back.

**C3. Steppers + ESP32 + NEMA17 → servos with encoders.**
- Model, interface and resolution are unknown (Q25).
- A `ServoAxis` interface with a mock comes first (task 17).
- The ESP32 firmware (task 6) is superseded, and its build failure stops mattering.

**C4. IR beacon + camera as the measurement → laser + PSD.**
- Task 4 becomes acquisition and tracking only.
- Task 5's calibration target changes from "camera to datum" to "PSD-B null and
  laser parallelism" (task 18).
- The reversal maths in `rada_align/calibration.py` is reused.

**C5. (new) Whiteboard vs D2/D3: "3 stepper motors" aim the lasers in closed loop.**
- The paste has 6 servos and a hand-adjusted board.
- Proposal: keep D3 (board fixed after CALIB; scanning and aiming with the horn
  servos). Ask what the 3 steppers were meant to move (Q24).

**C6. (new) Whiteboard draws quadrant cells; the BOM names the S5991-01.**
- The S5991-01 is a continuous 2D PSD (4 electrodes, centroid anywhere on the
  9×9 mm area) [datasheet].
- A quadrant detector is linear only within the spot radius.
- Proposal: a continuous PSD for A and C. B can be either.
- Confirm (Q26).

**C7. (new) D11 "Class 1 or 2" at 960 nm.**
- Class 2 is defined only for 400-700 nm [Illinois DRS laser classification page].
  At 960 nm the laser must be Class 1 (or 1M).
- Class 1 is enough:
  - Class 1 limit ≈ 1.3 mW at 960 nm [unverified recall of IEC 60825-1: 0.39 mW ×
    C4, C4 = 10^(0.002·(960−700)) = 3.31].
  - 1 mW already saturates the PSD (540 µA vs 500 µA [datasheet], 7.6), so we
    attenuate anyway.
- Open point: can the camera see 960 nm (IR-cut filter)? ACQUIRE uses the spot
  (Q27).

**C8. (new) D4/D5 "PSD-B valid only when PSD-A is at null".**
- The model gives B = f·(er − eh), so er = B/f + eh holds at any horn error [sim].
- Proposal: run the horn and radar loops at the same time with the combined
  estimate. The whiteboard order (antenna point, radar point, middle point) stays
  as the display sequence.

**C9. (new) PSD size at 12-15 m.**
- The 9 mm S5991-01 is too small for PSD-A and C in the field (7.1). Fine for the
  2 m demo.

**C10. (new) Only relative roll is observable optically.**
- 6 servos, 5 observable quantities: rank 5 [sim, 7.2].
- Common roll of both sides is set by the inclinometers (radar roll levelled).

**C11. (new) D4: "the target on each PSD is the calibrated offset, not the sensor
centre".**
- The beams are parallel to the horn axis, so each beam lands at its laser's
  offset from the line of sight. Each PSD therefore has to sit at the mirror
  position of its laser (the matched pattern, section 3).
- Proposal: build it matched, so every target is near the PSD centre. The
  calibrated target of D4 then only absorbs build residuals.
- Why: a target far from the centre makes the null depend on spot shape and roll,
  and on the PSD detection error, which is specified only over the central part
  [unverified, 7.6].
- This rewords D4 and needs the team's OK.

**Team decisions (section 0) close C1, C5, C6 and C7.** Approving the doc diff
also approves the remaining proposals: C2 (pending Q23), C8, C9, C10, C11. In the
proposed CLAUDE.md they are marked "(proposal Cn)".

---

## 7. Engineering checks

### 7.1 Spot size vs PSD area (check 1)

Inputs:
- S5991-01: 9×9 mm, peak 960 nm, 0.6 A/W [datasheet].
- Head pointing wind σ = 0.015° per head [repo: 0.0106 jitter ⊕ 0.0106 sway,
  ASSUMPTION].
- A linear reading needs the whole spot (D99, the 99 % power diameter) on the
  active area: linear range = ±(L − D99)/2.

**(a) Collimated Gaussian beam, best launch size** (w0 = √(λz/π)) [arith]:

| d | D99 | linear range | ±3σ wind on the PSD |
| --- | --- | --- | --- |
| 2 m | 3.36 mm | ±2.82 mm = ±0.081° | 1.57 mm |
| 12 m | 8.22 mm | ±0.39 mm = ±0.0019° | 9.42 mm |
| 15 m | 9.19 mm | none (spot > PSD) | 11.77 mm |

**(b) Geometric model D = D0 + θ·d, D0 = 2 mm (ASSUMPTION), maximum full divergence** [arith]:

| d | Spot only fits | Leaves ±3σ wind |
| --- | --- | --- |
| 2 m | θ ≤ 3.5 mrad | θ ≤ 1.93 mrad |
| 15 m | θ ≤ 0.47 mrad (below the diffraction limit at 960 nm, so not reachable collimated) | impossible |

**(c) Beam focused on the PSD distance, launch radius W = 5 mm (a 10 mm beam)** [arith]:

| d | D99 | linear range | depth of focus |
| --- | --- | --- | --- |
| 12 m | 2.25 mm | ±3.38 mm = ±0.016° | ±1.8 m |
| 15 m | 2.83 mm | ±3.08 mm = ±0.012° | ±2.9 m |

With W = 10 mm: 1.11 mm at 12 m, but depth of focus only ±0.44 m.

**Conclusion:**
- Focusing makes the spot small. The binding limit is PSD size vs wind.
- For ±3σ of the assumed wind, with the focused 10 mm beam [arith]:
  - 10 m: L ≥ 17.6 mm
  - 12 m: L ≥ 21.1 mm
  - 15 m: L ≥ 26.4 mm
  - 2 m: L ≥ 3.5 mm. The S5991-01 is fine for the demo.

**Field options:**
1. A 20×20 mm PSD, e.g. OSI DL-20 / DLS-20: 20×20 mm, detection error 200 µm
   [vendor page].
   - 12 m: ±(20 − 2.25)/2 = ±8.9 mm = ±0.042° ≈ 2.8σ [arith].
   - 15 m: ±8.6 mm = ±0.033° ≈ 2.2σ [arith].
2. A diffusing screen of 40-50 mm imaged onto the 9 mm PSD by a lens. It loses
   light, but there is power to spare (7.6). ASSUMPTION, to verify.
3. Accept the spot leaving the linear zone in gusts: flag those samples invalid and
   bridge with KLT. This biases the window statistics, so it is not recommended
   alone.

Even with a 20 mm PSD the assumed wind is still 2-3σ of range. Measuring the
real horn-mount wind sway (task 9.8) decides this.

### 7.2 Observability (check 2) [sim]

**Model:**
- Full 3D geometry with 10 states: 6 servo angles plus horn and radar lateral
  translations.
- Lever arms: horn pivot to centre 0.10 m [repo]; radar reference 0.10 m forward
  and 0.30 m above its tilt pivot (ASSUMPTION).
- Laser offsets as in section 3. PSD-B with f = 50 mm.

**Results at 2 m and at 12 m:**
- **Rank 5.** Rank of d(6 readings)/d(10 states) = 5; against the 6 servo angles
  alone, also 5.
- **All 5 errors observable:** eh_az, eh_el, er_az, er_el and relative roll. Their
  Jacobian lies in the row space of the readings' Jacobian (residual ≤ 1e-9).
- **The null space (dimension 5) changes no error** (largest change 2e-8). Its
  directions are:
  - a translation with compensating rotations on both sides (motion along the
    common line), in Y and in Z;
  - common roll.

  None of these is an alignment error. The elevation of the common line is what
  R3 reports, from the inclinometers.
- **Rotation vs translation:**
  - A lensless PSD cannot tell horn rotation from horn translation. It does not
    need to: both change the horn error by the same amount, and A reads exactly
    that.
  - The lens PSD (B) sees rotation only, which is why B + A gives the radar error.
- **Which subsets are enough:**
  - A alone: no radar error, and the horn error is not separable from roll.
  - A + B: both errors, provided roll is known.
  - A + B + C: everything.
- **Linear estimator vs true nonlinear errors:**

  | Range | 12 m | 2 m |
  | --- | --- | --- |
  | ±0.1° and ±10 mm from aligned | ≤ 0.00056° | ≤ 0.0019° |
  | ±1° and ±250 mm (setup range) | up to 0.09° | 0.55° |

  The large-range error only costs iterations.
- **Decoupled loop** (gain 1, inverse Jacobian at nominal): reaches < 0.001° in a
  median of 3 iterations at 2 m (max 4) and at 12 m (max 3), from ±1° and ±250 mm.
- **Laser misalignment, 0.01° on one laser** (12 m):

  | Misaligned | Effect |
  | --- | --- |
  | Laser B | 1:1 into the radar error |
  | Laser A or C alone | half into the horn error and the radar error (they are averaged), and large into roll (0.38°) |
  | Whole board, common mode | 1:1 into the horn error, 0 into the radar error |

  So the radar error depends on B versus the mean of A and C, not on the board as
  a whole.

### 7.3 Raster scan (check 3)

**Assumptions:**
- Step = PSD width at the PSD (9 mm), so the spot centre always crosses the active
  area.
- Pattern: serpentine raster (boustrophedon) in horn Pan/Tilt, centred on the
  ACQUIRE estimate. Stop at the first PSD with a valid sum, then centre locally.
- Speed: min(2°/s [repo max_speed, ASSUMPTION for the servos], 10 samples per PSD
  crossing at a 1 kHz lock-in output (ASSUMPTION)).
- 0.2 s per line turn (ASSUMPTION).

| d | step | ±0.05° | ±0.1° | ±0.25° | ±0.5° | ±1° | ±2.43° (blind) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2 m | 0.258° | 0.4 s | 0.5 s | 1.4 s | 3.8 s | 11.6 s | 54.8 s |
| 12 m | 0.043° | 0.9 s | 1.7 s | 5.9 s | 17.8 s | 58.4 s | 304.7 s |
| 15 m | 0.034° | 0.9 s | 2.0 s | 7.3 s | 22.0 s | 72.8 s | 378.3 s |

- **Search window vs ACQUIRE accuracy.**
  - A blind scan of the whole setup range takes 5-6 min at 12-15 m. The camera
    stage is what makes SCAN short.
  - Camera scale: 0.74 mm/px at 12 m with the 25 mm lens [arith from repo camera].
    The PSD unit and the spot are tens of pixels.
  - Window: ±3σ of the camera-to-laser-board boresight calibration. At
    σ = 0.05° (ASSUMPTION) the window is ±0.15°, between the 1.7 s (±0.1°) and
    5.9 s (±0.25°) columns at 12 m.
- **The other two spots.** Once A is captured, C is on its PSD (linear) if
  |relative roll| < atan(free margin / 250 mm baseline) [arith]:
  - S5991-01, focused beam: 0.77° at 12 m, 0.71° at 15 m.
  - 20 mm PSD: 2.0°.
  - Collimated beam on the 9 mm PSD: only 0.09° at 12 m [arith].

  Setup roll is ±0.5° [repo], so: level both sides with the inclinometers first,
  and add a roll search step if C is missing.
- **PSD-B capture:**
  - Angle field ±5.1° (f = 50 mm) [arith], which is wide.
  - Spatial field = lens aperture: 25 mm gives ±0.060° at 12 m (ASSUMPTION
    aperture). A is the binding one.

### 7.4 Laser and board stability vs temperature (check 4) [arith]

**Board bending from a through-thickness gradient:** θ = CTE·ΔT·L/t, with
L = 200 mm and t = 10 mm (ASSUMPTION).

| Board | ΔT 0.5 K | ΔT 1 K | ΔT 2 K |
| --- | --- | --- | --- |
| aluminium (23e-6/K [handbook]) | 0.013° | 0.026° | 0.053° |
| steel (12e-6) | 0.007° | 0.014° | 0.028° |
| invar (1.2e-6) | 0.0007° | 0.0014° | 0.0028° |

- **Aluminium is not acceptable.** Half a degree of sun gradient uses a quarter
  of the tolerance.
- **Proposal:**
  - invar or CFRP board (CFRP CTE [unverified]);
  - mount at one kinematic point;
  - sun shade;
  - measure after warm-up.
- **Uniform +20 K on aluminium** changes the laser spacing by 0.092 mm, which is
  0.00044° at 12 m: negligible.
- **Effect on the errors:**
  - Common-mode board rotation goes 1:1 into the horn error and not into the radar
    error [sim, 7.2].
  - Differential bending between laser B and A/C goes into the radar error.
- **The laser modules' own pointing drift per K is unknown.** It is an ASSUMPTION
  row in 7.7, to be measured in task 18.

### 7.5 Encoders, gearing, torque (check 5) [arith]

- **Resolution needed, 10× finer than 0.05° = 0.005° per count at the output:**
  72,000 counts/rev = 16.1 bit. Proposal 0.001° (360,000 counts = 18.5 bit) so the
  PID is smooth inside the 0.01° deadband [repo].
- **Encoder options:**
  - 17-bit motor encoder direct: 0.0027°/count. Meets 0.005°, misses the
    proposed 0.001°.
  - 17-bit behind 50:1: 0.000055°.
  - 20-bit direct: 0.00034°.
- **Backlash:** gear play after the motor encoder is invisible to the encoder.
  - The PSD outer loop closes it, but play that moves under wind is real pointing
    noise.
  - Proposal: zero-backlash or preloaded gearing (strain-wave or preloaded worm)
    [unverified specs], or an output-side encoder.
- **Scale check:** one 0.005° count = 1.05 mm on PSD-A at 12 m. The PSD resolves
  1.5 µm [datasheet], so the PSD is far finer than any encoder.
- **Radar torque, 35 kg [mentor], centre of gravity 3/5/10 cm off the tilt or roll
  axis:**
  - 10.3/17.2/34.3 N·m static.
  - 0.29-0.98 N·m at the motor through 100:1 at 35 % efficiency (ASSUMPTION).
  - Inertia about 1 kg·m² for a 0.5×0.5×0.3 m box (ASSUMPTION), so 1°/s² needs
    0.017 N·m: dynamics are trivial, gravity dominates.
- **Proposal:**
  - Balance the radar centre of gravity onto the tilt and roll axes.
  - Use a power-off holding brake on the 3 radar axes, needed for the "locked
    during the test" policy (C1).

#### 7.5.1 Motor resolution validation for 0.005° (2026-11-17) [datasheet + sim]

Question (Liam): can the motors really move in steps this fine?

- **Reference stage, Newport RV rotation stages [datasheet, rev 10/13]:**

  | Version | Min. incremental motion | Uni-dir. repeatability | Bi-dir. repeatability | Accuracy |
  |---|---|---|---|---|
  | RV standard | 0.001° | 0.002° | 0.004° | ±0.005° |
  | RV HAHLT (direct platen encoder, "better than 0.0001°") | 0.0002° | 0.0002° | 0.0012° | ±0.0025° |

  Datasheet values are guaranteed; typical values are about 2× better [datasheet].
  For contrast, a FLIR PTU-D300E pan-tilt has 0.0064° resolution [datasheet].
- **Servo model in the simulator (config.yaml `psd.servos`, values from the RV HAHLT row):**
  encoder 0.0001°, minimum step 0.0002° (moves fall on that grid), repeatability
  0.0002° p-p (uniform), lost motion on a direction change uniform(0, 0.001°)
  (bi-dir. minus uni-dir.), encoder on the output side, so it sees the lost motion.
- **Monte Carlo, demo profile, same seeds** (reported / TRUE / false accepts; incl. U_bias at 12 m):

  | Actuator model | 12 m, 200 runs | 2 m, 100 runs |
  |---|---|---|
  | ideal (no step, no play) | 100 / 100 / 0 % (96.0 %) | 100 / 98 / 2 % |
  | RV HAHLT | 100 / 100 / 0 % (93.0 %) | 100 / 99 / 1 % |
  | RV standard | 100 / 100 / 0 % (96.0 %) | 100 / 99 / 1 % |
  | 0.0064° step (FLIR PTU-D300E class) | 2 / 2 / 0 % (196 "no convergence in ALIGN") | 0 % (100 "no convergence in ALIGN") |

  Dominant cause for the PTU: one step (0.0064°) is wider than the whole
  tolerance (0.005°), so ALIGN cannot settle inside its 0.001° deadband.
- **Step test (controller.start_step_test, demo button "5 steps out and back"):**
  TRACK only, every loop idle; the axis gets n commanded steps out and n back; after
  each step the raw reading of one PSD (A for the radar, B for the horn; pan → x,
  tilt → y) divided by its sensitivity is averaged (0.5 s radar, 2 s horn). The PSD,
  not the encoder, is the judge. The first step after the reversal is reported apart
  (lost motion). Results, RV HAHLT model, seed 2:

  | Distance, axis | Step | PSD forward / back | Every step seen | PSD vs truth per step |
  |---|---|---|---|---|
  | 12 m, radar pan/tilt | 0.001° | 0.00096 / 0.00104° (sd 0.00008°) | yes | < 0.00001° |
  | 12 m, radar | 0.0005° | | yes | |
  | 12 m, radar | 0.0002° | forward 0.00015° | no (at the MIM and repeatability limit) | |
  | 12 m, horn | 0.001° | | yes | ≤ 0.0004° |
  | 2 m, radar pan | 0.001° / 0.0002° | | yes / yes | |
  | 2 m, radar tilt | 0.001° | forward 0.00085° (sd 0.00037°) | no | |

  Browser run (12 m, radar pan, 0.001°): forward 0.00104°, back 0.00096°, sd
  0.00005°, end point 0.0014° from the start (the lost motion on the reversal).
- **Conclusion [sim]:** a stage of the RV class (0.001° or finer minimum step)
  resolves the 0.005° requirement with margin of 5× (standard) to 25× (HAHLT); a
  pan-tilt with 0.0064° steps cannot. The simulation only shows that the loop works
  with these datasheet figures; it does not prove the stage meets them.
- **Caveats:**
  - Torque: RV240/350 HAHLT max torque 10 N·m [datasheet] against 10-34 N·m static
    for the 35 kg radar with the centre of gravity 3-10 cm off the axis (7.5): the
    radar must be balanced, or a larger stage or a counterweight is needed.
  - Load: centred load 4000/6500 N and max inertia 10 kg·m² [datasheet] are fine
    for the radar (about 350 N, about 1 kg·m², ASSUMPTION).
- **Bench protocol (before the field, ISO 230-2 style):** on the real stage, PSD-A
  at 12 m as the judge: 5 steps of 0.001° out and back, 5 cycles per axis, record
  forward and back mean, sd, the first step after the reversal and the end-point
  error; repeat at 0.0005° and 0.0002°. Pass if every 0.001° step is seen and the
  bi-directional end-point error is under 0.0012° (HAHLT) [proposal].

### 7.6 PSD noise, ADC, modulation, power (check 6)

**PSD resolution** [datasheet: 1.5 µm at Io = 1 µA, B = 1 kHz]:
- PSD-A: 7e-6° at 12 m (negligible).
- PSD-B, f = 50 mm: 0.0017° at 1 µA. At 50 µA and 100 Hz, about 1e-5° (scaled
  1/Io and √B, ASSUMPTION scaling).

**Position detection error** [datasheet: ±150 µm typ, ±250 max, measured with a
0.2 mm spot. The area it applies to is about the central 40 % of a side, from a
search excerpt of the Hamamatsu technical note only, so unverified]:
- PSD-A: 0.0007° at 12 m.
- PSD-B, f = 50 mm: 0.17°.
- Consequence: PSD-B must work as a null sensor at its calibrated null point, where
  this error cancels. Its scale is calibrated in task 18. "Before" errors far from
  the null (R5) are less accurate; show them as such.

**ADC:**

| ADC | Quantization | PSD-B f = 50 mm | PSD-A at 12 m |
| --- | --- | --- | --- |
| 12-bit | 2.2 µm | 0.0025° (marginal) | 1e-5° |
| 16-bit | 0.14 µm | 0.00016° | 7e-7° |

Proposal: 16-bit, simultaneous sampling.

**Modulation and lock-in:**
- PSD bandwidth ≈ 0.35/2 µs = 175 kHz [arith from the datasheet rise time].
- Modulation 10 kHz (ASSUMPTION), 4 samples per cycle.
- 3 PSDs × 4 electrodes = 12 channels × 40 kS/s = 480 kS/s.
- Lock-in output 100 Hz-1 kHz.

**Power:**
- 1 mW × 90 % filter × 0.6 A/W = 540 µA, above the 500 µA saturation [datasheet].
  So the laser is attenuated to about 0.1 mW (54 µA). That is far inside Class 1
  (C7) with very large SNR.
- **The problem is sunlight DC, not noise.** Direct sun into the PSD (0.6 W/m²/nm
  ASSUMPTION):
  - 10 nm filter: 292 µA
  - 3 nm filter: 87 µA
- Needs: a hood or baffle, an AC-coupled front end, and a narrow filter. A narrow
  filter needs a stable laser wavelength; diode drift per K is [unverified].

**Heat shimmer:** averaging over the 5-10 s window. Beam wander is kept at the old
0.003° row (ASSUMPTION) until the outdoor test (task 9.8).

### 7.7 Error budget (check 7), 1σ per axis, ASSUMPTION rows

**Radar error:**

| Source | ° |
| --- | --- |
| PSD-B optical axis to radar datum (calibration; replaces the camera-to-datum row; after the swap (0.1): laser board to radar datum) | 0.015 |
| Wind and tripod settling (real motion) | 0.015 |
| PSD unit seat repeatability on the datum | 0.010 |
| Laser B vs A/C parallelism (differential) | 0.005 |
| Servo residual inside the deadband | 0.005 |
| Turbulence / heat shimmer | 0.003 |
| PSD-B noise (f = 50 mm, 1 µA, 1 kHz) | 0.0017 |
| Board bending, differential (invar, 1 K) | 0.0014 |
| **RSS** | **0.0248 → 2σ = 0.0496** |

**Horn error:**

| Source | ° |
| --- | --- |
| Wind and tripod settling | 0.015 |
| Laser board to horn axis parallelism (lab CALIB; after the swap (0.1): PSD-B null to horn axis) | 0.010 |
| Servo residual | 0.005 |
| Turbulence | 0.003 |
| PSD-A position vs radar reference, 0.5 mm at 12 m | 0.0024 |
| Board bending, common (invar, 1 K) | 0.0014 |
| PSD-A noise and detection error at the null | 0.0001 |
| **RSS** | **0.0191 → 2σ = 0.038** |

**What changed against the old budget** (RSS 0.0248 [arith from the CHALLENGE.md
rows]):
- The camera, centroid and parallax rows are gone. The PSD noise rows are 10-100×
  smaller.
- The dominant rows are unchanged: transferring the radar datum to the sensor (now
  PSD-B), seat repeatability and wind. They still need Q1/Q11 (datum, 180°
  rotation for reversal).
- **The radar-error RSS is the same 0.0248° as the old budget.** The new design has
  not lost margin.
  - CHALLENGE.md called it "2× margin" (1σ vs 0.05). Read at 2σ it is 0.0496, i.e.
    no margin.
  - The rows are carried-over ASSUMPTIONs, so this is a reading, not a finding.
  - The Monte Carlo against truth in task 14 decides (true pass rate, false
    accepts).
- The horn's "RF axis" can only be its mechanical axis without RF data
  (ASSUMPTION, Q5).

### 7.8 KLT (check 8)

**Initialization:**
- Template matching (OpenCV `matchTemplate`, normalized correlation) over the
  full frame, multi-scale.
- The templates are of the PSD unit's outside appearance, captured in CALIB at the
  zoom levels used.
- Accept only a match above a DESIGN threshold whose 3 PSD housings fit the known
  layout.
- Then `goodFeaturesToTrack` inside that ROI, tracked with `calcOpticalFlowPyrLK`.

**Health, every frame:**
- forward-backward error per point;
- minimum number of points;
- a robust similarity transform (`estimateAffinePartial2D`);
- re-confirmation by template match every N frames.

**On loss:**
1. Points too few, FB error high or template score low: re-detect in an enlarged
   ROI, then over the full frame.
2. Lost longer than a set time (DESIGN): back to ACQUIRE. If the beams are also
   lost, then SCAN.

**Role in TRACK:**
- When the beam is briefly blocked, KLT gives the change in horn pointing relative
  to the radar target (image motion = f·Δθ), anchored at the last valid PSD sample.
- The horn holds on it; the radar side stays frozen and unobserved until the beam
  returns.
- KLT never enters VERIFY or the pass criterion.

**Practicalities:**
- With optical zoom, the intrinsics change per zoom step. Calibrate the zoom
  positions used, or use KLT only for relative motion at a fixed zoom.
- opencv-python is already an allowed dependency.
- Still needed: the friend's KLT data and code (task 16).

---

## 8. Requirements after the change

| Req | How it is met now |
| --- | --- |
| R1, R2 | PSD A/B/C estimator, both errors and roll (7.2) |
| R3 | Measured and reported (Δh from inclinometers + range); not removed without X/Z (C2, Q23) |
| R4 | 6 servos with encoders, PID |
| R5 | Before = first valid PSD estimate (B less accurate far from null, 7.6); after = VERIFY window |
| R6 | ALIGN on PSD-B null calibrated to the radar datum |
| R7 | Pass criterion unchanged; budget 7.7 |
| R8 | No radar data anywhere |
| R9 | CALIB file + SCAN from any setup; repeatability test in task 9 |
| R10 | Band-pass, lock-in, cabled, ALIGNED indicator |
| R11 | Horn: real-time correction. Radar: real-time monitoring, correction only in gaps (C1, Q22) |

---

## 9. BOM (D10), field system

Prices are the team's rough figures [team, unverified] unless a source is given.

| # | Part | Qty | Role | Rough USD | Notes |
| --- | --- | --- | --- | --- | --- |
| 1 | Servo with optical encoder + driver | 6 | 3 horn, 3 radar | 1,800-9,000 | ≥ 0.001° at the output, holding brake on the radar axes (7.5); model unknown (Q25) |
| 2 | Pan/Tilt/Roll mount for the 35 kg radar | 1 | carries the radar | 2,000-8,000 | CG balanced onto the axes; ±5° travel (section 2) |
| 3 | Pan/Tilt/Roll mount for the horn | 1 | mechanics | 1,000-4,000 | pivot close to the aperture centre |
| 4 | Dual-axis inclinometer, ~0.001° resolution | 2 | gravity reference, roll, R3, cross-check | 1,500-5,000 | team quote ATO ~1,324 each [unverified]; resolution ≠ accuracy (RIG.md, SCL3300 lesson) |
| 5 | 2D PSD + front end | 3 | A, B (lens), C | 300-2,700 | S5991-01: 9×9 mm, 960 nm peak, 0.6 A/W, 1.5 µm, ±150 µm, Isat 500 µA [datasheet]; team price ~98 EUR [unverified]. Field A/C: 20×20 mm class (OSI DL-20 [vendor page]) or diffuser (7.1) |
| 6 | Laser on the adjustable common board | 3 | PSD light source | 60-2,400 | Class 1, 960 nm, focus-adjustable 10 mm beam (7.1); attenuated (7.6); invar/CFRP board (7.4) |
| 7 | Raster scan | 1 | capture | 300-3,000 if separate hardware | 0 if done with the horn servos (D12, Q24) |
| 8 | High-resolution camera with optical zoom | 1 | acquisition, KLT, UI | 600-3,000 | must see 960 nm or carry a visible aid (Q27) |
| 9 | Laser rangefinder | 1 | d for scale and parallax | 150-1,000 | 10-15 m range; TF-Luna ends at 8 m [datasheet, RIG.md] |
| 10 | Fast multi-channel ADC | 1 | PSD readout | 50-500 | 12 channels, 16-bit, ≥ 40 kS/s per channel (7.6) |
| 11 | Controller | 1 | state machine, PID, UI | 500-2,000 | model open (Q28) |
| 12 | Calibration kit | 1 | CALIB of lasers and PSDs | 300-2,000 | horn axis reference, targets, fixture |
| 13 | Cables, power, field enclosures | - | infrastructure | 300-1,500 | cabled only |

- **Sums** [arith]: low 8,860, high 44,100. Without item 7 as hardware: 41,100.
- **Budget:** the high end exceeds the USD 30k budget by 14,100. Task 3 produces a
  ≤ 30k configuration from datasheets.
- **Where the money is:** items 1, 2 and 4 hold 22,000 of the high end. Spend it
  where 7.7 says the error is (datum seat, wind and stiffness), not on sensor
  resolution.

---

## 10. What is reused, what is superseded

| Reused | Superseded (kept in the repo, marked) |
| --- | --- |
| geometry, disturbances (wind, drift), estimator windows and pass criterion, Monte Carlo against truth, reversal maths, calibration file + refusal, vision detect/centroid (spot and target detection), dashboard server/session/schema, tests | camera-and-beacon as the measuring sensor, X/Z axes, ESP32 stepper firmware and driver, `Actuators` (x, z, pan, tilt) as the main actuator interface |

The code plan is in docs/TASKS.md (tasks 13-20).

---

## Appendix: numbers script

- **Where the script is:** `scripts/final_design_numbers.py`.
- **Run:** `python scripts/final_design_numbers.py`. It uses numpy only and has no
  repo imports.
- **Output:** every number in sections 2, 4, 7 and 9 comes from it. The raw
  output of the run on 2026-11-16 is below.

Sources read on 2026-11-16:
- [Hamamatsu S5990/S5991 datasheet](https://www.hamamatsu.com/content/dam/hamamatsu-photonics/sites/documents/99_SALES_LIBRARY/ssd/s5990-01_etc_kpsd1010e.pdf)
- [Hamamatsu PSD technical note](https://www.hamamatsu.com/content/dam/hamamatsu-photonics/sites/documents/99_SALES_LIBRARY/ssd/psd_kpsd9001e.pdf)
  (position error specified over 40 % of a side for 2D PSDs)
- [OSI duo-lateral PSD page](https://www.osioptoelectronics.com/products/photodetectors/duo-lateral-position-sensing-detectors-psd-two-dimensional)
- [Illinois DRS laser classification](https://drs.illinois.edu/Page/SafetyLibrary/LaserClassification)
  (Class 2 = visible 400-700 nm only)

```text

==============================================================================
1. Spot size vs PSD (S5991-01, 9 x 9 mm)
==============================================================================
head pointing sigma (wind, [repo] assumption) = 0.0150 deg
Gaussian beam, best possible collimation for each distance (w0 = sqrt(lambda z / pi)):
  d m   w0 mm  D1/e2 mm   D99 mm lin +/- mm lin +/- deg 3sig wind mm
    2    0.78      2.21     3.36       2.82      0.0809         1.57
   10    1.75      4.94     7.50       0.75      0.0043         7.85
   12    1.91      5.42     8.22       0.39      0.0019         9.42
   15    2.14      6.06     9.19      -0.09         nan        11.77
Fixed launch beam 3 mm (w0 = 1.5 mm):
  Rayleigh range 7.36 m, full divergence 0.407 mrad
  d    2 m: D1/e2  3.11 mm, D99  4.72 mm
  d   10 m: D1/e2  5.06 mm, D99  7.68 mm
  d   12 m: D1/e2  5.74 mm, D99  8.70 mm
  d   15 m: D1/e2  6.81 mm, D99 10.33 mm
Geometric model D = D0 + theta*d, D0 = 2 mm (ASSUMPTION), spot must leave
linear range r on the 9 mm PSD: D <= 9 - 2r
  d    2 m, fit only, r=0     : D <=   9.00 mm -> theta_full <=   3.500 mrad
  d    2 m, r = 3 sigma wind  : D <=   5.86 mm -> theta_full <=   1.930 mrad
  d   15 m, fit only, r=0     : D <=   9.00 mm -> theta_full <=   0.467 mrad
  d   15 m, r = 3 sigma wind  : D <= -14.55 mm -> theta_full <=  -1.103 mrad
Beam FOCUSED at the PSD distance (waist on the PSD), launch radius W at the horn:
  W  2.5 mm, d    2 m: spot D99  0.75 mm, depth of focus +/- 0.20 m, lin +/- 4.13 mm = 0.1182 deg, PSD for 3 sigma wind >=   3.9 mm
  W  2.5 mm, d   10 m: spot D99  4.77 mm, depth of focus +/- 8.09 m, lin +/- 2.12 mm = 0.0121 deg, PSD for 3 sigma wind >=  20.5 mm
  W  2.5 mm, d   12 m: spot D99  7.58 mm, depth of focus +/-20.45 m, lin +/- 0.71 mm = 0.0034 deg, PSD for 3 sigma wind >=  26.4 mm
  W  2.5 mm, d   15 m: spot D99  7.58 mm, depth of focus +/-20.45 m, lin +/- 0.71 mm = 0.0027 deg, PSD for 3 sigma wind >=  31.1 mm
  W  5.0 mm, d    2 m: spot D99  0.37 mm, depth of focus +/- 0.05 m, lin +/- 4.31 mm = 0.1236 deg, PSD for 3 sigma wind >=   3.5 mm
  W  5.0 mm, d   10 m: spot D99  1.87 mm, depth of focus +/- 1.24 m, lin +/- 3.57 mm = 0.0204 deg, PSD for 3 sigma wind >=  17.6 mm
  W  5.0 mm, d   12 m: spot D99  2.25 mm, depth of focus +/- 1.80 m, lin +/- 3.38 mm = 0.0161 deg, PSD for 3 sigma wind >=  21.1 mm
  W  5.0 mm, d   15 m: spot D99  2.83 mm, depth of focus +/- 2.85 m, lin +/- 3.08 mm = 0.0118 deg, PSD for 3 sigma wind >=  26.4 mm
  W 10.0 mm, d    2 m: spot D99  0.19 mm, depth of focus +/- 0.01 m, lin +/- 4.41 mm = 0.1263 deg, PSD for 3 sigma wind >=   3.3 mm
  W 10.0 mm, d   10 m: spot D99  0.93 mm, depth of focus +/- 0.31 m, lin +/- 4.04 mm = 0.0231 deg, PSD for 3 sigma wind >=  16.6 mm
  W 10.0 mm, d   12 m: spot D99  1.11 mm, depth of focus +/- 0.44 m, lin +/- 3.94 mm = 0.0188 deg, PSD for 3 sigma wind >=  20.0 mm
  W 10.0 mm, d   15 m: spot D99  1.39 mm, depth of focus +/- 0.69 m, lin +/- 3.80 mm = 0.0145 deg, PSD for 3 sigma wind >=  24.9 mm
PSD size needed for +/- 3 sigma wind with the diffraction-limited spot:
  d    2 m: L >=    6.5 mm
  d   10 m: L >=   23.2 mm
  d   12 m: L >=   27.1 mm
  d   15 m: L >=   32.7 mm

==============================================================================
2. Observability: 3D model, Jacobian, rank  [sim]
==============================================================================

-- d = 2.0 m, nominal aligned state: readings [0. 0. 0. 0. 0. 0.], errors [0. 0. 0. 0. 0.]
rank of d(readings)/d(10 states) = 5; singular values [3381.414  3354.4024  248.7897   59.7414   59.6496    0.    ]
rank vs the 6 servo angles only = 5
max residual of errors projected on readings = 9.14e-10 (0 => all 5 errors observable)
  readings per error, mm per deg (columns eh_az, eh_el, er_az, er_el, roll):
    A_y:   +31.4159    -0.0000    +0.0000    -0.0000    +1.7453
    A_z:    +0.0000   -31.4159    -0.0000    +0.0000    -2.6180
    B_y:    -0.8727    +0.0000    +0.8727    -0.0000    +0.0000
    B_z:    -0.0000    +0.8727    +0.0000    +0.8727    -0.0000
    C_y:   +31.4159    -0.0000    -0.0000    +0.0000    -1.7453
    C_z:    +0.0000   -31.4159    -0.0000    +0.0000    +0.0000
  LS estimator eh_az  [deg] = +0.015915*A_y +0.015915*C_y
  LS estimator eh_el  [deg] = -0.007639*A_y -0.010186*A_z +0.007639*C_y -0.021645*C_z
  LS estimator er_az  [deg] = +0.015915*A_y +1.145916*B_y +0.015915*C_y
  LS estimator er_el  [deg] = +0.007639*A_y +0.010186*A_z +1.145916*B_z -0.007639*C_y +0.021645*C_z
  LS estimator roll   [deg] = +0.183346*A_y -0.137510*A_z -0.183346*C_y +0.137510*C_z
null space dim = 5; max |d errors| along it = 2.08e-08
  null vector: ty_h -0.862, ty_r -0.361, pan_r +0.251, pan_h +0.251
  null vector: roll_r +0.675, roll_h -0.675, ty_r -0.229, tilt_h -0.078
  null vector: tz_h +0.845, tilt_h -0.345, tilt_r +0.345, tz_r +0.155
  null vector: ty_r +0.751, pan_r +0.429, pan_h +0.429, roll_r +0.147
  null vector: tz_r +0.845, tilt_h +0.345, tilt_r -0.345, tz_h +0.155
  A only : residual per error (eh_az, eh_el, er_az, er_el, roll) = [ 3.1311  4.6966 59.1046 59.1414 56.3595]
  B only : residual per error (eh_az, eh_el, er_az, er_el, roll) = [31.831  31.831  31.831  31.831  57.2958]
  A+B    : residual per error (eh_az, eh_el, er_az, er_el, roll) = [ 3.111   4.6665  3.111   4.6665 55.9977]
  linear estimator, near aligned +/-0.1 deg, +/-10 mm: worst |est - true| deg = [0.00165 0.00193 0.0002  0.0002  0.00181]
  linear estimator, setup +/-1 deg, +/-250 mm: worst |est - true| deg = [0.55455 0.49818 0.46345 0.35659 0.52912]

-- d = 12.0 m, nominal aligned state: readings [0. 0. 0. 0. 0. 0.], errors [0. 0. 0. 0. 0.]
rank of d(readings)/d(10 states) = 5; singular values [16953.8903 16948.539    249.953     50.7636    50.7564     0.    ]
rank vs the 6 servo angles only = 5
max residual of errors projected on readings = 1.45e-10 (0 => all 5 errors observable)
  readings per error, mm per deg (columns eh_az, eh_el, er_az, er_el, roll):
    A_y:  +205.9489    +0.0000    -0.0000    -0.0000    +1.7453
    A_z:    +0.0000  -205.9489    -0.0000    +0.0000    -2.6180
    B_y:    -0.8727    -0.0000    +0.8727    -0.0000    -0.0000
    B_z:    -0.0000    +0.8727    +0.0000    +0.8727    +0.0000
    C_y:  +205.9489    +0.0000    +0.0000    -0.0000    -1.7453
    C_z:    +0.0000  -205.9489    -0.0000    +0.0000    +0.0000
  LS estimator eh_az  [deg] = +0.002428*A_y +0.002428*C_y
  LS estimator eh_el  [deg] = -0.001165*A_y -0.001554*A_z +0.001165*C_y -0.003302*C_z
  LS estimator er_az  [deg] = +0.002428*A_y +1.145916*B_y +0.002428*C_y
  LS estimator er_el  [deg] = +0.001165*A_y +0.001554*A_z +1.145916*B_z -0.001165*C_y +0.003302*C_z
  LS estimator roll   [deg] = +0.183346*A_y -0.137510*A_z -0.183346*C_y +0.137510*C_z
null space dim = 5; max |d errors| along it = 1.92e-08
  null vector: ty_r +0.896, ty_h +0.429, roll_r +0.068, roll_h -0.068
  null vector: ty_h +0.825, ty_r -0.339, roll_r -0.301, roll_h +0.301
  null vector: tz_h +0.993, tilt_h -0.082, tilt_r +0.082, roll_r -0.019
  null vector: roll_r -0.635, roll_h +0.635, ty_h -0.349, ty_r +0.260
  null vector: tz_r +0.993, tilt_h +0.082, tilt_r -0.082, roll_r +0.019
  A only : residual per error (eh_az, eh_el, er_az, er_el, roll) = [ 0.4852  0.7279 57.7661 57.7665 57.2579]
  B only : residual per error (eh_az, eh_el, er_az, er_el, roll) = [29.1334 29.1334 29.1334 29.1334 57.2958]
  A+B    : residual per error (eh_az, eh_el, er_az, er_el, roll) = [ 0.4849  0.7274  0.4849  0.7274 57.2224]
  linear estimator, near aligned +/-0.1 deg, +/-10 mm: worst |est - true| deg = [4.7e-04 5.6e-04 1.0e-05 1.0e-05 5.5e-04]
  linear estimator, setup +/-1 deg, +/-250 mm: worst |est - true| deg = [0.08221 0.08971 0.00439 0.00346 0.09111]

-- Rotations only: solve the 6 servo angles for errors = 0 and radar roll level
  d    2 m, offset +/-50 mm: max |servo angle|  1.43 deg, radar boresight off level up to 1.43 deg, max |PSD reading| at solution 8.2e-11 mm, max |error| 9.0e-12 deg
  d    2 m, offset +/-250 mm: max |servo angle|  7.12 deg, radar boresight off level up to 7.10 deg, max |PSD reading| at solution 3.1e-09 mm, max |error| 9.6e-11 deg
  d   10 m, offset +/-250 mm: max |servo angle|  1.43 deg, radar boresight off level up to 1.42 deg, max |PSD reading| at solution 1.1e-10 mm, max |error| 2.0e-12 deg
  d   12 m, offset +/-250 mm: max |servo angle|  1.19 deg, radar boresight off level up to 1.19 deg, max |PSD reading| at solution 1.7e-10 mm, max |error| 3.4e-12 deg
  d   15 m, offset +/-250 mm: max |servo angle|  0.95 deg, radar boresight off level up to 0.95 deg, max |PSD reading| at solution 4.1e-11 mm, max |error| 6.5e-13 deg
  range needed (arith): 1 deg setup + atan(0.25/d): 10 m 2.43 deg, 12 m 2.19 deg, 15 m 1.95 deg

-- Decoupled loop (gain 1, estimator above, inverse of d(errors)/d(servos) at nominal)
  d    2 m: iterations to |errors| < 0.001 deg: median 3, max 4
  d   12 m: iterations to |errors| < 0.001 deg: median 3, max 3

-- Laser misalignment 0.01 deg on one laser at a time (aligned state, d = 12 m)
  laser A az: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [-0.005   0.0024 -0.005  -0.0024 -0.3776]
  laser A el: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [-0.     -0.0032 -0.      0.0032 -0.2832]
  laser B az: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [ 0.   -0.    0.01  0.    0.  ]
  laser B el: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [ 0.   -0.    0.   -0.01 -0.  ]
  laser C az: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [-0.005  -0.0024 -0.005   0.0024  0.3776]
  laser C el: estimated errors (eh_az, eh_el, er_az, er_el, roll) = [ 0.     -0.0068  0.      0.0068  0.2832]
  whole board 0.01 deg az (common mode): [-0.01  0.   -0.   -0.   -0.  ]

==============================================================================
3. Raster scan with the horn Pan/Tilt
==============================================================================
step = PSD width L = 9 mm at the PSD (spot centre always passes over the active area)
 d m  step deg speed deg/s     R=0.05      R=0.1     R=0.25      R=0.5      R=1.0     R=2.43
   2    0.2578        2.00       0.4s       0.5s       1.4s       3.8s      11.6s      54.8s
  10    0.0516        2.00       0.6s       1.4s       5.0s      15.0s      48.8s     254.7s
  12    0.0430        2.00       0.9s       1.7s       5.9s      17.8s      58.4s     304.7s
  15    0.0344        2.00       0.9s       2.0s       7.3s      22.0s      72.8s     378.3s
Capture of the other two spots once A is on its PSD:
  d    2 m: C on its PSD if |relative roll| < 0.647 deg (baseline 250 mm, spot D99 3.4 mm)
  d   10 m: C on its PSD if |relative roll| < 0.172 deg (baseline 250 mm, spot D99 7.5 mm)
  d   12 m: C on its PSD if |relative roll| < 0.090 deg (baseline 250 mm, spot D99 8.2 mm)
  d   15 m: C on its PSD if |relative roll| < 0.000 deg (baseline 250 mm, spot D99 9.2 mm)
  PSD-B angular field (f 50 mm, ASSUMPTION): +/- 5.14 deg; spatial capture = lens aperture 25 mm (ASSUMPTION) = +/- 0.060 deg at 12 m

==============================================================================
4. Laser board thermal stability  [arith]
==============================================================================
board 200 mm long, 10 mm thick (ASSUMPTION); through-thickness gradient bends it:
theta = CTE * dT * L / t
  aluminium dT 0.5 K: 0.0132 deg
  aluminium dT 1.0 K: 0.0264 deg
  aluminium dT 2.0 K: 0.0527 deg
  steel     dT 0.5 K: 0.0069 deg
  steel     dT 1.0 K: 0.0138 deg
  steel     dT 2.0 K: 0.0275 deg
  invar     dT 0.5 K: 0.0007 deg
  invar     dT 1.0 K: 0.0014 deg
  invar     dT 2.0 K: 0.0028 deg
uniform +20 K, aluminium: spacing change 0.092 mm -> 0.00044 deg at 12 m

==============================================================================
5. Encoder, gearing, torque  [arith]
==============================================================================
  0.005 deg per count at the output: 72,000 counts/rev = 16.1 bit
  0.001 deg per count at the output: 360,000 counts/rev = 18.5 bit
  17-bit motor encoder, gear 1:1 -> 0.002747 deg per count at the output
  17-bit motor encoder, gear 50:1 -> 0.000055 deg per count at the output
  20-bit motor encoder, gear 1:1 -> 0.000343 deg per count at the output
  23-bit motor encoder, gear 1:1 -> 0.000043 deg per count at the output
  one 0.005 deg count = 1.05 mm on PSD-A at 12 m; PSD resolves 1.5 um
  35 kg [mentor], CG 3 cm off the tilt/roll axis:  10.3 N m; via 100:1 at 35% (ASSUMPTION) 0.29 N m at the motor
  35 kg [mentor], CG 5 cm off the tilt/roll axis:  17.2 N m; via 100:1 at 35% (ASSUMPTION) 0.49 N m at the motor
  35 kg [mentor], CG 10 cm off the tilt/roll axis:  34.3 N m; via 100:1 at 35% (ASSUMPTION) 0.98 N m at the motor
  inertia (box 0.5x0.5x0.3 m, ASSUMPTION) 0.99 kg m^2; 1 deg/s^2 needs 0.017 N m

==============================================================================
6. PSD noise, ADC, modulation  [arith from datasheet]
==============================================================================
  PSD-A at    2 m: 1.5 um = 4.30e-05 deg; detection error 150 um = 0.0043 deg
  PSD-A at   12 m: 1.5 um = 7.16e-06 deg; detection error 150 um = 0.0007 deg
  PSD-A at   15 m: 1.5 um = 5.73e-06 deg; detection error 150 um = 0.0006 deg
  PSD-B f 50 mm: 0.05 deg = 43.6 um; 1.5 um = 0.00172 deg; detection error 150 um = 0.172 deg (cancels at a calibrated null point)
  PSD-B f 100 mm: 0.05 deg = 87.3 um; 1.5 um = 0.00086 deg; detection error 150 um = 0.086 deg (cancels at a calibrated null point)
  Io   1 uA, B 1000 Hz: resolution ~ 1.5000 um (scaled 1/Io, sqrt(B); ASSUMPTION scaling) -> PSD-B f 50 mm 1.72e-03 deg
  Io  50 uA, B  100 Hz: resolution ~ 0.0095 um (scaled 1/Io, sqrt(B); ASSUMPTION scaling) -> PSD-B f 50 mm 1.09e-05 deg
  Io 100 uA, B  100 Hz: resolution ~ 0.0047 um (scaled 1/Io, sqrt(B); ASSUMPTION scaling) -> PSD-B f 50 mm 5.44e-06 deg
  12-bit ADC on the electrode currents: ~2.197 um -> PSD-B f 50 mm 0.00252 deg, PSD-A 12 m 1.05e-05 deg
  16-bit ADC on the electrode currents: ~0.137 um -> PSD-B f 50 mm 0.00016 deg, PSD-A 12 m 6.56e-07 deg
  laser 1.0 mW (ASSUMPTION), filter 90%: photocurrent 540 uA vs saturation 500 uA [datasheet]
  direct sun into the PSD, 10 nm filter: 292 uA DC (needs hood + AC-coupled front end)
  direct sun into the PSD,  3 nm filter: 87 uA DC (needs hood + AC-coupled front end)
  PSD bandwidth ~ 0.35/tr = 175 kHz [arith from datasheet rise time]; modulation 10 kHz (ASSUMPTION), 4 samples per cycle -> 40 kS/s x 12 channels = 480 kS/s
  Class 1 AEL at 960 nm ~ 0.39 mW x C4, C4 = 10^(0.002(lambda-700)) = 3.31 -> 1.29 mW [unverified recall of IEC 60825-1]

==============================================================================
7. Error budget per error, 1 sigma, RSS  [ASSUMPTION rows]
==============================================================================
-- radar error
  PSD-B optical axis to radar datum (calibration)      0.0150
  PSD unit seat repeatability on the datum             0.0100
  wind and tripod settling (real motion)               0.0150
  laser A vs laser B parallelism (differential)        0.0050
  board bending, differential (invar, 1 K)             0.0014
  turbulence / heat shimmer                            0.0030
  servo residual inside deadband                       0.0050
  PSD-B noise (f 50 mm, 1 uA, 1 kHz)                   0.0017
  RSS                                                  0.0248  -> 2 sigma 0.0496 vs 0.05
-- horn error
  laser A to horn axis parallelism (lab CALIB)         0.0100
  PSD-A position vs radar reference, 0.5 mm at 12 m    0.0024
  wind and tripod settling (real motion)               0.0150
  board bending, common (invar, 1 K)                   0.0014
  turbulence / heat shimmer                            0.0030
  servo residual inside deadband                       0.0050
  PSD-A noise and detection error at the null          0.0001
  RSS                                                  0.0191  -> 2 sigma 0.0383 vs 0.05
previous camera budget RSS [repo CHALLENGE.md rows] = 0.0248

==============================================================================
BOM sums (D10 ranges)  [arith]
==============================================================================
  low 8,860  high 44,100  high without item 7 (scan = horn servos) 41,100

==============================================================================
Extra numbers quoted in docs/FINAL_DESIGN.md  [arith]
==============================================================================
  PSD    9 mm, d   12 m, focused W 5 mm: linear +/-3.38 mm = +/-0.0161 deg = 1.07 sigma wind; C captured if |relative roll| < 0.77 deg
  PSD    9 mm, d   15 m, focused W 5 mm: linear +/-3.08 mm = +/-0.0118 deg = 0.79 sigma wind; C captured if |relative roll| < 0.71 deg
  PSD   20 mm, d   12 m, focused W 5 mm: linear +/-8.88 mm = +/-0.0424 deg = 2.83 sigma wind; C captured if |relative roll| < 2.03 deg
  PSD   20 mm, d   15 m, focused W 5 mm: linear +/-8.58 mm = +/-0.0328 deg = 2.19 sigma wind; C captured if |relative roll| < 1.97 deg
  camera 1.55 um / 25 mm at 12 m: 0.744 mm per px
  ACQUIRE window +/-3 x 0.05 deg = +/-0.15 deg
  BOM items 1+2+4 high: 22,000; high end over the 30k budget by 14,100
  PSD-A scale at 12 m: 205.9 mm/deg (11.8 m laser-to-PSD path, lever ASSUMPTION); d*pi/180 at 12.0 m = 209.4 mm/deg
```
