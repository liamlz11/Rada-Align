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
