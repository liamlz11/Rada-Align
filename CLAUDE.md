# RADA horn-to-radar alignment (hackathon)

## Goal
Autonomous system that aligns a horn antenna to a radar in a field test: residual error <= +/-0.05 deg in azimuth AND elevation, without using any radar data.
Full challenge summary: docs/CHALLENGE.md. Original slides: docs/slides/. Original Hebrew docs (challenge summary, prompt pack): docs/reference/.

## Key numbers
- Distance 10-15 m. 0.05 deg = 873 urad = 10.47 mm at 12 m (d*tan(theta)). Tabletop demo at 2 m: 1.75 mm.
- Two errors: radar error (radar mechanical boresight vs line of sight to the horn center) and horn error (horn boresight vs reverse line of sight). dx/dh = radar error in mm.
- Initial manual setup error: up to +/-1 deg and +/-25 cm.
- Pass criterion: |mean| + 2*sigma < 0.05 deg per axis, both errors, over a 5-10 s window.

## Requirements
R1 autonomous detection of Az/El errors | R2 measure orientation of radar AND horn | R3 height difference horn center vs radar axis | R4 automatic correction with motors | R5 show error before and residual after | R6 align horn axis to radar mechanical reference axis | R7 <= +/-0.05 deg Az and El | R8 independent of radar data | R9 repeatable after move or re-setup | R10 field-ready with clear completion indication | R11 keep alignment during the whole test, real-time correction

## Architecture (decided)
- Mutual sighting: each side has camera + IR LED beacon + dual-axis inclinometer. The radar-side head clamps to the radar mechanical datum, outside the RF aperture.
- Radar stays fixed. Horn on a motorized mount: X, Z, Pan, Tilt (steppers + worm gear, ESP32).
- Parallax correction is mandatory: eps = atan((u - cx) / f) - delta_calib - s / d.
- State machine: IDLE -> ACQUIRE -> ALIGN -> VERIFY -> TRACK -> FAULT. Correct Z/X first, then Pan/Tilt.
- Python package `rada_align`. Hardware sits behind interfaces Camera, Inclinometer, Rangefinder, Actuators so simulated and real drivers are swappable.

## How we work (read every session)
1. Read docs/PROGRESS.md to see where we are, then docs/TASKS.md for the next open task.
2. Work ONE task at a time, in order, unless I name a different one. Tasks 4-7 may run in parallel sessions.
3. For each task: reply with a short plan (files, interfaces, assumptions) and wait for my OK. Then build in milestones and run the tests after each.
4. When a task is done: tick it in docs/TASKS.md and append an entry to docs/PROGRESS.md (date, what was built, test output summary, real numbers, open issues). Commit with a clear message.
5. Gates marked STOP in docs/TASKS.md always need my explicit OK.

## Rules
- NEVER use radar data.
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
