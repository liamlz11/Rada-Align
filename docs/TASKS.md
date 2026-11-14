# Tasks – in order

Rules for every task: see "How we work" in CLAUDE.md. Plan first, wait for OK, build in milestones, run tests, tick the box, log in docs/PROGRESS.md, commit.
Items in [BRACKETS] are details to ask me for if they are still unfilled.
Suggested effort: high by default; xhigh for tasks 1, 2 (planning) and 5.

- [ ] 0. Repo setup
- [ ] 1. Design review (red team)
- [x] 2. Simulator and control core
- [ ] 3. Demo rig and parts
- [ ] 4. Camera beacon detection
- [ ] 5. Camera-to-datum calibration
- [ ] 6. ESP32 firmware + driver
- [ ] 7. Operator dashboard
- [ ] 8. Integration and tabletop demo
- [ ] 9. Validation suite R1-R11
- [ ] 10. Update from mentor answers
- [ ] 11. Pitch deck content
- [ ] 12. Demo script and Q&A

---

## 0. Repo setup
- `git init`, first commit with the existing files.
- Python package skeleton `rada_align/` with `interfaces.py` (abstract Camera, Inclinometer, Rangefinder, Actuators), `config.yaml` (empty sections), `pyproject.toml` or `requirements.txt` (numpy, scipy, matplotlib, pyyaml, pytest), `tests/`.
- Tell me how to create the venv on Windows and in WSL.
Done when `pytest -q` runs (even with 0 tests) on both.

## 1. Design review (red team)
Act as a senior DRS RADA radar test-range engineer judging this hackathon. Using docs/CHALLENGE.md, write docs/REVIEW.md:
1. The 5 weakest points that could make the system miss +/-0.05 deg in the field or violate "no radar data". For each: failure mechanism, error size it could cause (show the arithmetic), cheapest fix buildable by Monday.
2. Which error-budget entries are optimistic and what to assume instead.
3. The 8 questions a judge would ask, with the answer we should give.
Skip anything that is fine. Then tell me in Hebrew which changes you propose to CLAUDE.md or the task list, and wait for my OK before applying them.

## 2. Simulator and control core
Modules in rada_align/:
1. geometry - frames, line of sight, radar/horn Az-El errors, dx/dh, parallax correction.
2. sensors - simulated Camera, Inclinometer, Rangefinder, Actuators: pixel model with focal length in px, centroid noise, calibration offset; inclinometer noise and bias; rangefinder noise.
3. disturbances - wind (jitter + low-frequency sway), slow tripod drift, actuator step quantization and backlash.
4. estimator - moving average or simple Kalman filter, mean and sigma per axis.
5. controller - the state machine from CLAUDE.md with a deadband; FAULT with a reason when vibration makes the pass criterion impossible.
6. montecarlo - N runs with random initial errors: per axis mean, p95, max, pass rate, time to converge; plus sensitivity analysis (one error source at a time) = measured error budget.
7. CLI `python -m rada_align.sim --runs 500 --distance 12`: results table, plots to ./out (convergence, residual histogram, error-budget bars); `--stream` emits live state as JSON lines for the dashboard.
Required tests: 10.47 mm at 12 m -> 0.050 deg +/-0.001; parallax correction recovers the true angle; zero-noise run converges; extreme wind ends in FAULT with a reason.
STOP and ask if Monte Carlo p95 > 0.05 deg on any axis.
Report: Monte Carlo table, measured vs preliminary budget, the 3 assumptions that drive the result most.

## 3. Demo rig and parts
Parts we have: [LIST OF PARTS]. Demo distance: [DISTANCE] m.
Write docs/RIG.md:
1. The simplest rig from these parts: horn mock-up mount (all 4 axes or the subset we can motorize by Monday), radar reference head, rigid mounting of cameras and beacons. Text sketch with dimensions.
2. Angular resolution of each motorized axis with these parts, and whether it is good enough for 0.05 deg at the demo distance.
3. Field-version bill of materials: part, role, the spec that matters for 0.05 deg, approximate price, datasheet link (use web search). Mark any spec not confirmed on a datasheet as [unverified].
Copy the real values into config.yaml, still marked with their source.

## 4. Camera beacon detection
rada_align/vision/, implementing Camera:
1. Capture: OpenCV VideoCapture, device index and resolution from config.yaml, fixed exposure where the driver allows.
2. Beacon detection: ROI around last position, threshold, blob filter (size, circularity), reject saturated or multiple blobs. Frame differencing (LED on minus off) behind a flag.
3. Subpixel centroid: intensity-weighted with background subtraction. Return u, v, sigma over the window, quality flag.
4. Intrinsics: chessboard calibration script (cv2.calibrateCamera) saving calib/camera.yaml; undistort points before computing angles.
5. scripts/measure_stability.py: 30 s of a static LED -> centroid sigma in px and deg, plots.
Tests on synthetic images: Gaussian spot at a known subpixel position recovered within 0.05 px with noise; two spots rejected; saturated spot flagged.
Must run from native Windows Python (WSL2 has no webcam access by default). Tell me exactly how to run it there.
Report the measured sigma.

## 5. Camera-to-datum calibration
1. docs/CALIBRATION.md: reversal method (measure a distant target, rotate the head 180 deg about the datum axis, measure again; mean = true axis, half difference = camera offset delta) and ground-truth method (known offsets on a ruler/micrometer table at known distance), with distances and repetitions.
2. scripts/calibrate_reversal.py: guides the operator step by step, N measurements per position, computes delta_Az and delta_El with sigma, writes calib/datum.yaml.
3. Add the reversal procedure to the simulator: recovers a known injected delta; residual calibration error vs N and centroid noise.
4. The controller loads calib/datum.yaml and refuses to start without it.
Test: simulated reversal recovers the injected delta within the predicted sigma.

## 6. ESP32 firmware + driver
Hardware: board [ESP32 BOARD], drivers [TMC2209 / A4988 / OTHER], 4 steppers (X, Z, Pan, Tilt), limit switch per axis [YES/NO], e-stop button, IR beacon LED on a GPIO.
Firmware in firmware/ (PlatformIO, Arduino framework, AccelStepper allowed):
- Line-based serial protocol: MOVE <axis> <steps>, MOVETO <axis> <pos>, HOME <axis>, STOP, STATUS, SPEED <axis> <v> <a>, LED <on|off|blink hz>, PING. Every command answers OK or ERR with a reason. STATUS returns positions, moving flags, limits, e-stop.
- Safety: soft limits per axis in config.h, e-stop stops everything immediately, watchdog stops motion if no PING for 1 s.
- Pin map and steps per degree/mm in config.h, each marked ASSUMPTION.
Python: rada_align/hw/esp32_actuators.py implements Actuators over pyserial (port from config.yaml), unit conversion, timeouts, ERR handling.
Tests: driver against a mocked serial port incl. watchdog and errors. Firmware must build with `pio run`.
STOP: do not flash. Give me the flash command and a Hebrew bench-test checklist, motors disconnected first.

## 7. Operator dashboard
FastAPI + WebSocket backend, one static HTML/JS page (no build step, no framework). Hebrew UI, RTL, numbers left-to-right.
Shows: big numbers (radar Az/El, horn Az/El, dx/dh in mm; green within 0.05 deg, red outside); bullseye plot with the 0.05 deg circle, current point and trail; convergence chart per axis; state and fault reason; before/after of the last alignment; large ALIGNED indicator; buttons Start alignment, Stop, Re-align, Export report.
Data: the simulator `--stream` now, the real controller later, one schema in docs/messages.md.
Done when `python -m rada_align.ui` runs against the simulator and the page was checked in a browser.

## 8. Integration and tabletop demo
1. config.yaml `mode: sim | hw`. hw mode uses the real Camera, ESP32 actuators, inclinometer if connected; anything missing stays simulated and shows SIM in the dashboard.
2. Tabletop distance [DISTANCE] m, same angular tolerance.
3. Dry-run mode: compute and display corrections without moving motors.
4. Per-run logging to runs/<timestamp>/ (JSONL measurements, commands, state changes) + HTML report (before/after per axis, sigma, pass/fail).
STOP before the first real motor motion: show soft limits, max speeds, e-stop status, wait for my explicit OK.
Done when one ACQUIRE -> ALIGN -> VERIFY -> TRACK cycle runs on the real rig, or report exactly which step fails and why.

## 9. Validation suite R1-R11
Runnable in sim and hw mode, scripts/validate/, each writes results/<test>.json:
1. Static noise: 1,000 measurements, no motion -> sigma per axis. Target < 0.005 deg.
2. Ground truth: known offsets on a ruler/micrometer table -> measured vs true. Target < 0.01 deg.
3. Convergence: initial errors up to +/-1 deg -> time and final residual.
4. Repeatability: 10 restarts from scratch -> spread of final results. Target < 0.02 deg.
5. Disturbance: push or fan during TRACK -> detected, corrected or warned.
6. Independence: full run with no radar connected.
7. Lighting: strong light behind the beacon -> beacon still acquired.
`python -m rada_align.validate --report` builds results/REQUIREMENTS.md: R1-R11 table with status, measured number, evidence file, plus pitch-ready plots.
Targets are fixed. NEVER change a threshold to make a test pass.

## 10. Update from mentor answers
When docs/OPEN_QUESTIONS.md has answers: write docs/DECISIONS.md with (1) which design decisions change, (2) the updated error budget with arithmetic, (3) proposed CLAUDE.md edits as a diff, (4) tasks to cut, add or reorder for the time left. Wait for my OK before applying.

## 11. Pitch deck content
Write docs/PITCH.md: content for 10 Hebrew slides, using only numbers from results/ and docs/:
1 problem and why manual alignment fails; 2 the number: 0.05 deg = 10.5 mm at 12 m; 3 our approach: mutual optical sighting, no radar data; 4 architecture diagram (describe it); 5 the algorithm; 6 error budget preliminary vs measured; 7 live demo; 8 results R1-R11; 9 path from tabletop to field; 10 team and next steps.
Missing result -> visible [TBD], never an invented number. List which plots from results/ go on which slide. (The deck itself is built in claude.ai from this file.)

## 12. Demo script and Q&A
Write docs/DEMO.md in Hebrew:
1. A 3-minute demo script, minute by minute: what we say, what the dashboard shows, what moves.
2. Fallback: if hardware fails mid-demo, switch to the simulator in under 30 seconds, and what we say.
3. The 10 hardest judge questions (accuracy proof, calibration, wind, independence from radar data, cost, field readiness) with 2-sentence answers based only on results/REQUIREMENTS.md.
4. A checklist for the last 15 minutes before the demo.
