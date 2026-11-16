# DRS RADA challenge – autonomous horn antenna alignment

## The challenge (from the slides)
How to precisely level two planes (azimuth and elevation) to ensure accuracy during a radar field test.

In radar tests, the radar and the test (horn) antenna must face each other so that their reference axes are aligned in Azimuth and Elevation. Alignment quality directly affects the reliability and validity of the test. A height, angle or direction error on either side creates misalignment: the measurement runs under conditions different from those defined, and the radar's performance can be misinterpreted.

### Test setup (slide diagram)
| Part | Role |
| --- | --- |
| RADAR on tripod | Device under test (DUT). Transmits toward the horn, receives the simulated target |
| Horn antenna on tripod | Receives the radar signal and transmits the simulated echo back |
| Target simulator | Cabled to the horn. Creates the artificial target |
| PS | Radar power supply |
| PC | Radar control and data recording |
| Distance | 10-15 m between horn and radar |

### The gap in today's process
Manual setup relies on mechanical measurements, spirit levels, visual estimation, distance measurement or GPS. These methods:
- depend on operator skill
- are slow and need several iterations
- have limited accuracy
- are hard to reproduce between setups of the same test
- give no quantitative measure of the residual alignment error
- accumulate error from terrain and weather (strong wind, etc.)
- Since the radar is the DUT, its own data cannot be used to decide whether the setup is aligned.

### Requirements
Design an autonomous, automatic system to align a horn antenna to a radar in a field test. The system must:
- R1 detect alignment errors in Azimuth and Elevation by itself
- R2 measure and compute the orientation of both sides: radar and horn antenna
- R3 detect height differences between the horn center and the radar reference axis
- R4 correct the alignment automatically with motors and pointing mechanisms
- R5 show the operator the error before correction and the residual error after it
- R6 end with the horn axis aligned to the radar's mechanical reference axis
- R7 achieve a maximum alignment error of +/-0.05 deg in Azimuth and Elevation
- R8 operate as an independent measurement system, without radar data
- R9 allow repeating the alignment after a move or re-setup
- R10 suit field conditions and give a clear indication that alignment is complete
- R11 keep accurate pointing throughout the test, with automatic real-time correction

## Engineering analysis (our team)

### Geometry
offset = d * tan(0.05 deg) = d * 8.73e-4
| d | allowed offset |
| --- | --- |
| 2 m (tabletop demo) | 1.75 mm |
| 10 m | 8.7 mm |
| 12 m | 10.5 mm |
| 15 m | 13.1 mm |

Insights:
- Two different errors: radar error (radar axis vs line of sight, the critical one) and horn error (horn axis vs reverse line of sight).
- A horn offset in mm IS an angular error: 1 cm at 12 m uses the whole El budget. R3 and elevation error are the same problem.
- Parallax: a camera 10 cm off the radar axis sees the horn at 0.48 deg. Must correct with known offset and distance; 5 cm distance error then costs only ~0.002 deg.
- GNSS heading and magnetometers are orders of magnitude too coarse for azimuth. Azimuth must be measured relatively, optically.
- Elevation vs gravity is easy: a dual-axis inclinometer is at the ~0.005 deg level.
- A 1.55 um pixel with a 25 mm lens = 0.0036 deg/px; subpixel centroiding reaches ~0.001 deg.

### Preliminary error budget (per axis, RSS; ESTIMATES, to be replaced by measurements)
| Source | deg |
| --- | --- |
| Camera-to-radar-datum calibration (dominant) | 0.015 |
| Wind and tripod settling | 0.015 |
| Fixture mounting repeatability | 0.010 |
| Inclinometer | 0.005 |
| Actuator residual in closed loop | 0.005 |
| Atmospheric turbulence / heat | 0.003 |
| Centroid | 0.002 |
| Parallax (distance error) | 0.002 |
| RSS total | ~0.025 (2x margin) |

Conclusion: sensors are not the problem. Transferring the radar's mechanical reference to the sensor, and setup stability in the field, are.

### Measurement options
| Method | Az | El | Typical accuracy | Decision |
| --- | --- | --- | --- | --- |
| Camera + LED beacon (mutual sighting) | yes | yes | 0.001-0.005 deg | core |
| Laser + position detector (PSD/target) | yes | yes | high | alternative / operator visual |
| Dual-axis inclinometer | no | yes | ~0.005 deg | complements El and roll |
| GNSS heading / compass | no | no | tenths of deg and worse | rejected |
| Total station / theodolite | yes | yes | arcseconds | ground truth for validation only |
| Independent RF receiver at horn (beam peak search) | yes | yes | limited by beamwidth | phase 2, ask RADA |

### Architecture
1. Radar reference head (RRH): clamped to the radar's mechanical datum. Narrow-FOV camera, IR beacon, inclinometer. Kept out of the radar aperture (RF reflections).
2. Motorized horn head: horn on a 4-axis mount: Z lift, X lateral, Pan, Tilt. Camera, IR beacon, inclinometer, rangefinder.
3. Controller: Raspberry Pi / laptop for vision and estimation; ESP32 for steppers.
4. Operator UI: before/after error, live state, ALIGNED indicator, log and report.

Which side moves: Option A (chosen) radar fixed, horn moves X/Z onto the radar axis then Pan/Tilt back; +/-1 deg at 12 m = +/-21 cm, so ~+/-25 cm travel. Option B: radar on a motorized Az/El positioner if RADA has one.
Confirmed by the mentor (2026-11-16): the radar is aligned once before the test and is static during it, so Option A holds. No positioner was mentioned. Travel check: +/-250 mm covers 1.43 deg at 10 m, 1.19 deg at 12 m, 0.95 deg at 15 m; a 1 deg error at 15 m needs 262 mm, so the field mount needs about +/-400 mm (1.53 deg at 15 m). Option B stays a field option within the budget (ASSUMPTION).

### Mentor facts (2026-11-16, via Liam; details in docs/MENTOR_FEEDBACK.md)
- Radar: 35 kg, 4-8 GHz (C-band). Static during the test.
- "0,0" = the radar's mechanical reference: its face perpendicular to the line of sight to the horn. This is the starting reference of the radar's measurement (our interpretation, medium confidence).
- Radar data is not needed on our display. The no-radar-data rule is unchanged.
- Simple sensors (laser, camera, IR, inclinometer) are welcome as backups. Do not rely on GPS or radio.
- Budget up to USD 30k for the hardware, if justified.

### Target simulator and what alignment guarantees
The radar transmits; the horn receives and passes the signal by cable to the target simulator, which sends it back through the same horn:
- Delay = range: R = c*tau/2 (1 us = 150 m). The physical 12 m path adds 80 ns round trip.
- Frequency shift = Doppler = speed: f_d = 2v/lambda, i.e. 26.7 Hz per m/s at 4 GHz, 53.3 Hz per m/s at 8 GHz.
- Amplitude = RCS.
- Angle: NOT set by the simulator. The radar sees the target where the horn physically is.
Delay, Doppler, amplitude and cable losses are DRS's. The angle is ours: alignment guarantees the simulated target appears at Az = 0, El = 0 relative to the radar's mechanical axis within +/-0.05 deg. Radar error moves the reported angle; horn pointing error changes only the returned power.

Calibration of camera to radar axis: reversal method (rotate the fixture 180 deg about the reference axis, the mean is the true axis), or a one-time lab calibration against a theodolite/collimator. Lab calibration does not violate R8.

### Operating flow
1. Coarse manual setup (+/-1 deg is enough), mount RRH on the datum.
2. Self-check: sensors, calibration file loaded, link between heads.
3. Acquire: each camera finds the other beacon (blinking LED synced with frames; on-minus-off frame difference removes sun and background). Spiral search with Pan/Tilt if not found.
4. Measure: centroid, distance, tilts, average N frames, compute errors. Show "before".
5. Position correction: move horn Z and X until radar error is within threshold.
6. Pointing correction: Pan/Tilt until horn error is within threshold. Loop back to 4 until both pass.
7. Verify: 5-10 s sample, pass only if |mean| + 2 sigma < 0.05 deg. Show "after" and green light.
8. Track during test: continuous measurement; drift > 0.03 deg -> auto correction and log; vibration beyond threshold -> "unstable conditions" warning.
9. Report: timestamp, setup ID, before/after, sigma (proves R9).

eps_Az = atan((u - cx) / f) - delta_Az - s_x / d
eps_El = atan((v - cy) / f) - delta_El - s_y / d
dh = d * tan(eps_El_radar). Filter with a moving average or simple Kalman, with a deadband so motors do not chase wind.

### Suggested hardware (verify against datasheets)
| Part | Suggestion | Why |
| --- | --- | --- |
| Camera x2 | Raspberry Pi HQ Camera or USB3 industrial camera, 25-50 mm lens | ~0.002-0.004 deg/px |
| Beacon x2 | 850 nm blinking IR LED + band-pass filter on camera | daylight operation |
| Inclinometer x2 | precision dual-axis MEMS (e.g. SCL3300 / ADXL355 class) | pitch/roll vs gravity |
| Rangefinder | small laser (TF-Luna / LIDAR-Lite) or tape | parallax only, +/-5 cm is enough |
| Pan/Tilt | NEMA17 + 1:100 worm gear + TMC2209 | 1.8/16/100 = 0.0011 deg/step, self-locking |
| X / Z | lead-screw slide with stepper, +/-25 cm | sub-mm resolution |
| Computer | Raspberry Pi 5 / laptop | vision + UI |
| Motor controller | ESP32 | step pulses, limit switches, e-stop |
Do NOT use hobby servos (SG90/MG996): resolution ~1 deg, 20x the budget.

### Risks
| Risk | Mitigation |
| --- | --- |
| Inaccurate camera-to-datum calibration | reversal method + ground-truth check |
| Forgetting parallax | measured distance + mechanical offsets in calibration file |
| Motor backlash | closed loop on the camera, worm gear, deadband |
| Wind and soft tripods | averaging, "unstable" warning, heavy tripods |
| Sunlight saturation | IR + band-pass + frame differencing |
| Metal parts near antennas | heads outside aperture or removable after alignment |
| Demo fails on judging day | simulator ready as backup + recorded video |
| Heat: sun on brackets and tripod legs | 5 K across the head bracket = 0.013 deg; 10 K on a 1 m aluminium leg over a 1 m base = 0.013 deg (arith); shade, low-expansion parts, reversal after warm-up |
| Ground vibration (generators, vehicles) | 10 Hz camera aliases motion above 5 Hz into slow apparent motion; faster global-shutter camera or short exposure; measure |
| Dust on lenses | asymmetric spot biases the centroid; lens covers, watch blob circularity over time |
| Radio near a C-band radar | 5 GHz WiFi is inside 4-8 GHz; all links cabled |
