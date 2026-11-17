# RADA horn-to-radar alignment (hackathon)

## Goal
Autonomous system that aligns a horn antenna to a radar in a field test: residual error <= +/-0.05 deg in azimuth AND elevation, without using any radar data.
Full challenge summary: docs/CHALLENGE.md. Final design (source of truth): docs/FINAL_DESIGN.md. Mentor feedback and gap analysis: docs/MENTOR_FEEDBACK.md. Original slides: docs/slides/. Original Hebrew docs (challenge summary, prompt pack): docs/reference/.

## Key numbers
- Requirement tightened to 0.005 deg (mentor, Liam 2026-11-17; DRS value 0.05 deg): 1.05 mm at 12 m, 0.17 mm at 2 m. The demo runs a profile without wind, drift and calibration residuals (rada_align/psd/config.py DEMO_SOURCES); with every assumed source on, 0.005 deg is not reached (docs/FINAL_DESIGN.md 0.3).
- Distance 10-15 m. 0.05 deg = 873 urad = 10.47 mm at 12 m (d*tan(theta)). Tabletop demo at 2 m: 1.75 mm.
- Two errors: radar error (radar mechanical boresight vs line of sight to the horn center) and horn error (horn boresight vs reverse line of sight). dx/dh = radar error in mm.
- Initial manual setup error: up to +/-1 deg and +/-25 cm.
- Pass criterion: |mean| + 2*sigma < 0.005 deg (was 0.05) per axis, both errors, over a 5-10 s window.
- Final design (docs/FINAL_DESIGN.md): servo travel without X/Z = 1 deg + atan(0.25/d) = 2.43 deg at 10 m (design +/-5 deg). PSD-A moves d*pi/180 per deg of horn error (209.4 mm at 12 m). The 9 mm S5991-01 PSD is fine at 2 m; 12-15 m needs about 21-26 mm under the assumed wind.

## Requirements
R1 autonomous detection of Az/El errors | R2 measure orientation of radar AND horn | R3 height difference horn center vs radar axis | R4 automatic correction with motors | R5 show error before and residual after | R6 align horn axis to radar mechanical reference axis | R7 <= +/-0.05 deg Az and El | R8 independent of radar data | R9 repeatable after move or re-setup | R10 field-ready with clear completion indication | R11 keep alignment during the whole test, real-time correction

## Architecture (decided 2026-11-16: docs/FINAL_DESIGN.md is the source of truth)
- Raster Scan + PSD + NanoTrack (AI tracker, was KLT; Liam 2026-11-17). 3 lasers on one board and the camera on the RADAR, the board clamped to the radar mechanical datum outside the RF aperture, lasers parallel to the radar axis. 3 PSDs on the HORN (Liam, 2026-11-16: the laser must not point at the person aiming the radar; docs/FINAL_DESIGN.md 0.1). PSD-A (no lens) reads the radar error; PSD-B (lens, focal plane) reads the beam angle, so horn error = B/f + radar error (proposal C8: both loops run together); PSD-C (no lens) is a third position reading in the Az/El solution and a cross-check. Both units hang BELOW the radar and the horn (0.40-0.58 m, Liam 2026-11-16), so the relative roll comes from the two inclinometers, not from PSD A/C (docs/FINAL_DESIGN.md 0.2).
- Rig rule: matched pattern. Each PSD sits at the mirror position of its laser, so every target is near a PSD centre (proposal C11, rewords D4).
- Both sides rotate: 6 servos with optical encoders (horn and radar Pan/Tilt/Roll). No X/Z: the height difference (R3) is measured and reported, not removed (proposal C2, DRS Q23).
- The radar moves only before the test (SCAN, ALIGN). During the test its axes are braked and only monitored; if it moves, the spot drifts on the horn PSDs and the system alerts; correction (radar raster + ALIGN) only in an allowed gap (team decision 2026-11-16; the gap signal is Q21).
- Continuous 2D PSDs (not quadrant). Laser board adjusted by hand in CALIB only. Raster scan with the radar Pan/Tilt (the emitter side; before the test), no separate scanner. Controller: laptop for the demo, industrial PC for the field (team decisions 2026-11-16).
- Camera with optical zoom: acquisition (template matching), NanoTrack backup tracking (OpenCV TrackerNano, ONNX model), UI picture. It is not a measuring sensor. Inclinometers: gravity reference, roll, R3 report, cross-check.
- Lasers: Class 1 at 960 nm (the PSD peak; Class 2 exists only for visible light; team decision 2026-11-16), focusable 10 mm beam attenuated to about 0.1 mW, modulated, lock-in on the PSD signals, band-pass filter.
- All links are cabled. No radio near the radar: it works at 4-8 GHz (C-band), which includes 5 GHz WiFi. No GPS.
- Hardware budget: up to USD 30k for the field system, if justified (mentor, 2026-11-16).
- State machine: CALIB -> SETUP -> ACQUIRE -> SCAN -> ALIGN -> VERIFY -> TRACK, plus FAULT.
- Python package `rada_align`. Hardware sits behind interfaces (Camera, Inclinometer, Rangefinder, PSD, ServoAxis) so simulated and real drivers are swappable.

### Superseded 2026-11-16 (code and docs kept in the repo)
- Mutual sighting (camera + IR LED beacon as the measuring sensor on each side), radar fixed, horn on X/Z/Pan/Tilt steppers + worm gear + ESP32, the `Actuators` (x, z, pan, tilt) interface, state machine IDLE -> ACQUIRE -> ALIGN -> VERIFY -> TRACK -> FAULT with Z/X first. The parallax formula eps = atan((u - cx) / f) - delta_calib - s / d and the matched camera/beacon offsets apply to that code only.

## How we work (read every session)
1. Read docs/PROGRESS.md to see where we are, then docs/TASKS.md for the next open task.
2. Work ONE task at a time, in order, unless I name a different one. Tasks 4-7 may run in parallel sessions (old numbering; final design: tasks 15-17 after task 14, proposal).
3. For each task: reply with a short plan (files, interfaces, assumptions) and wait for my OK. Then build in milestones and run the tests after each.
4. When a task is done: tick it in docs/TASKS.md and append an entry to docs/PROGRESS.md (date, what was built, test output summary, real numbers, open issues). Commit with a clear message.
5. Gates marked STOP in docs/TASKS.md always need my explicit OK.

## Rules
- NEVER use radar data.
- Monte Carlo is scored against truth. Always quote true pass rate and false accepts next to the reported pass rate.
- All component parameters live in config.yaml, each marked "ASSUMPTION - replace with datasheet/measurement". Never present an assumed value as a real spec.
- Never tune parameters or thresholds to make a test pass. Report failures with the dominant cause.
- Ask before adding dependencies beyond numpy, scipy, matplotlib, pyyaml, pytest, opencv-python, pyserial, fastapi, uvicorn.
- Ask before deleting files or touching anything outside this repo.
- Never flash hardware or move real motors without my explicit OK.
- If the same test fails 3 times in a row, stop and report instead of retrying.
- Communicate with me in Hebrew. Code and comments in English.
- Every number you report must come from an actual run. Show the command output.

## Environment
- Windows 11 laptop with WSL2. Repo lives on Windows: C:\Users\liaml\Desktop\Projects\Rada-Align (in WSL: /mnt/c/Users/liaml/Desktop/Projects/Rada-Align).
- WSL2 does not see USB webcams or serial ports by default: camera and ESP32 code must also run from native Windows Python.

## Commands
- Tests: `pytest -q`
- Simulator: `python -m rada_align.sim --runs 500 --distance 12`
- Final-design numbers: `python scripts/final_design_numbers.py` (lasers-on-horn model; the current layout is in rada_align/psd)
- Final-design demo (simulation): `python -m rada_align.psd` then http://127.0.0.1:8001
- Final-design Monte Carlo: `python -m rada_align.psd.montecarlo --runs 200 --distance 12` (`--weather clear|rain_light|rain_heavy|fog|shimmer|sun`, same seeds)
