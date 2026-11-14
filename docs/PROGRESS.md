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
