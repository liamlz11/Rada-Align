# Progress log

Append one entry per finished task (newest at the bottom). Only real numbers from actual runs.

Template:
## <date> – Task <N>: <name>
- Built:
- Tests: <command> -> <result>
- Key numbers:
- Open issues / next:

---

## 2026-11-14 – Setup
- Built: CLAUDE.md, docs/CHALLENGE.md, docs/TASKS.md, docs/OPEN_QUESTIONS.md, slides in docs/slides/.
- Next: task 0 (git init + Python env), then task 1.

## 2026-11-14 – Task 0: Repo setup (Windows verified, WSL NOT verified -> box not ticked)
- Built: `git init -b main` + first commit of the existing files (818cd96). Skeleton: `pyproject.toml` (numpy, scipy, matplotlib, pyyaml; pytest as `dev` extra; requires-python >=3.10), `rada_align/__init__.py`, `rada_align/interfaces.py` (abstract Camera, Inclinometer, Rangefinder, Actuators + BeaconMeasurement, TiltReading; PROVISIONAL, signatures get finalized in the task 2 plan), `config.yaml` (mode + empty sections), `tests/test_smoke.py`. venv instructions for Windows and WSL in README.md.
- Tests: `.venv/Scripts/python.exe -m pytest -q` on native Windows, Python 3.11.9 -> `7 passed in 0.05s`.
- Key numbers: installed numpy 2.4.6, scipy 1.17.1, matplotlib 3.11.2, PyYAML 6.0.3, pytest 9.2.0.
- Open issues / next:
  - WSL half of the done-criterion is unverified: `wsl -l -v` lists only `docker-desktop` (no Linux distro). Liam must either install a distro (`wsl --install -d Ubuntu`) and run the README steps, or waive the WSL requirement. Tick task 0 in docs/TASKS.md only after that.
  - Windows has Python 3.14.3 (default `python`) and 3.11; the venv uses 3.11 (`py -3.11`) for the widest wheel coverage (opencv later).
  - Liam asked to go 0 -> 2; task 1 (design review) is still open and unticked.
  - Next: task 2 plan (file tree, interface signatures, assumption list) is waiting for Liam's OK. No task 2 code written yet.
- Doc-consistency findings (computed with `py -3.11 -c ...` on 2026-11-14, feed into tasks 1, 2, 3):
  - Tabletop parallax: with a 10 cm camera offset, a 5 cm range error costs 0.0020 deg at 12 m but 0.0697 deg at 2 m (> tolerance). "+/-5 cm is enough" holds only at field distance. If the far beacon's offset from its reference point matches the near camera's offset from its axis (net offset 0), the sensitivity is 0; with 1 cm net mismatch it is 0.0070 deg at 2 m. To be verified by a geometry test in task 2; rig rule for task 3.
  - Preliminary budget: RSS = 0.0248 deg; read as 1 sigma, 2*RSS = 0.0497 deg, i.e. no margin against |mean| + 2*sigma < 0.05. Calibration + fixture rows (RSS 0.0180 deg) are biases the system cannot see in its own measurement.
  - Travel: +/-1 deg needs 20.95 cm at 12 m and 26.18 cm at 15 m, more than the +/-25 cm travel.
  - Roll: 0.5 deg roll with a 10 cm offset shifts the parallax term by 0.87 mm = 0.0042 deg at 12 m, 0.0250 deg at 2 m. Roll must be measured (inclinometer) or the offset kept small.
  - Small-angle s/d vs atan(s/d) at 2 m with s = 10 cm: 0.0024 deg. Code will use atan.
  - Gaps: no task calibrates the horn-side camera to the horn boresight (task 5 covers the radar datum only); beacon LED control / frame sync and the data link between the heads are not in the four interfaces; the `--stream` schema is needed in task 2 but docs/messages.md is assigned to task 7; "p95 > 0.05" STOP gate does not say p95 of what; open question 3 (which error the 0.05 deg applies to) changes the pass logic.

## 2026-11-15 – Task 2: Simulator and control core (built; box NOT ticked: STOP gate triggered at 2 m, waiting for Liam)
- Built: `rada_align/` config.py (strict loader, ERROR_SOURCES), geometry.py, world.py (truth), sensors.py (SimCamera/Inclinometer/Rangefinder/Actuators, build_simulation), disturbances.py, estimator.py, controller.py (IDLE/ACQUIRE/ALIGN/VERIFY/TRACK/FAULT), runner.py (one run scored against truth), montecarlo.py, plots.py, sim.py (CLI). `docs/messages.md` = stream schema v0. All parameters in config.yaml, marked ASSUMPTION or DESIGN.
- Tests: `.venv/Scripts/python.exe -m pytest -q` -> `64 passed in 5.96s`. Required four are in: 10.47 mm at 12 m (test_geometry), parallax recovers true angle (test_geometry, test_estimator), zero-noise run converges at 2 m and 12 m (test_controller), 10x wind ends in FAULT "unstable: 2*sigma ... pass criterion impossible" (test_controller).
- Key numbers (SIMULATION, assumed parameters; all scored against truth; seed 0):
  - `python -m rada_align.sim --runs 500 --distance 12 --jobs 8`: true residual p95 per axis 0.0391 / 0.0407 / 0.0364 / 0.0364 deg (radar az, radar el, horn az, horn el), max 0.0609. STOP gate not triggered. Reported pass 100.0%, TRUE pass (|mean|+2sigma<0.05 on truth) 32.4%, false accepts 67.6%. True score p95 0.062-0.067 deg. Time to ALIGNED mean 21.7 s, p95 31.4 s, max 49.8 s. Re-alignments while tracking 0.17 per run.
  - Budget at 12 m, RMS true error, worst axis, one source at a time (100 runs each; preliminary in brackets): none/control residual 0.0085; calibration 0.0199 (0.015); wind 0.0176 (0.015); fixture 0.0131 (0.010); actuator 0.0101 (0.005); rangefinder 0.0094 (0.002); turbulence 0.0089 (0.003); drift 0.0088; inclinometer 0.0085 (0.005); centroid 0.0085 (0.002); all on 0.0248 (preliminary RSS 0.0248). Rows near 0.0085 are at the control-residual floor, i.e. the source itself adds ~0.
  - `--distance 2`: STOP gate TRIGGERED. True residual p95 0.157 deg on every axis, true pass 7.2%, reported pass 99.8%. Dominant cause: rangefinder (+/-5 cm with a 10 cm net camera/beacon offset): 0.0876 deg alone; all on 0.0901.
  - `--distance 15`: reported pass 92.2%; 39 of 500 runs FAULT on x/z travel limit (+/-250 mm). p95 0.036-0.039 deg.
- The 3 assumptions that drive the result: (1) camera-to-datum calibration residual 0.015 deg + fixture 0.010 deg: biases the system cannot see, the reason for false accepts; (2) wind 0.015 deg per tripod: 2*sigma = 0.03 deg eats 60% of the tolerance; (3) net camera/beacon offset 10 cm with +/-5 cm range error: negligible at 12 m, breaks the 2 m tabletop.
- Design notes / deviations to confirm with Liam:
  - Coarse ALIGN is sequential (X/Z, re-measure, Pan/Tilt) down to the 0.03 deg tracking band. Fine trims in VERIFY/TRACK command X/Z and Pan/Tilt in the same step with feed-forward of the known coupling (an X move of eps changes the horn error by eps). Without it a drifting radar starved the horn correction (found by test_track_corrects_drift).
  - Camera offset delta is defined as the pixel angle read for an on-axis target; correction uses a rotation (geometry.mount_rotation), not plain subtraction, so it is exact at the null at 2 m.
  - Remaining model term away from the null: up to 0.0077 deg at 2 m, 0.0005 deg at 12 m (other head tilted -> beacon depth shift). Zero at the null.
  - Inclinometer is used for roll only (parallax baseline rotation). Elevation is measured optically. Its budget row contributes ~0 in this architecture.
  - Not modelled: tripod translation under wind, lens distortion, beacon detection failures, camera roll misalignment, error in the known mechanical offsets, link latency between heads.
- Open issues / next:
  - DECISION NEEDED (STOP gate at 2 m): matched camera/beacon offsets on the rig (net offset ~0), or range known to ~1 mm at tabletop. Not changed in config; nothing tuned.
  - True pass rate 32% at 12 m with the preliminary budget: the budget has no margin against |mean|+2sigma (2*RSS = 0.0497). Needs a smaller calibration bias or less wind, or a decision on what the criterion applies to (open question 3).
  - WSL run of pytest still unverified (task 0).
  - Next after the decision: task 1 (design review) is still open, then task 3.
