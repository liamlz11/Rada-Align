# Rada-Align

Hackathon project for the DRS RADA challenge: autonomous alignment of a horn antenna to a radar in a field test, to within +/-0.05 deg in azimuth and elevation, without using radar data.

## Setup

One venv per OS (a Windows venv does not work from WSL and vice versa).

Windows (PowerShell, from the repo root):

```powershell
py -3.11 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
pytest -q
```

If activation is blocked: `Set-ExecutionPolicy -Scope Process RemoteSigned`, or skip activation and call `.venv\Scripts\python.exe -m pytest -q`.

WSL (needs a Linux distro, e.g. `wsl --install -d Ubuntu`; keep the venv off /mnt/c, it is slow there):

```bash
sudo apt update && sudo apt install -y python3-venv python3-pip
python3 -m venv ~/.venvs/rada-align
source ~/.venvs/rada-align/bin/activate
cd /mnt/c/Users/liaml/Desktop/Projects/Rada-Align
python -m pip install -e ".[dev]"
pytest -q
```

Camera and ESP32 code must run from the Windows venv (WSL2 has no USB webcam or serial access by default).

## Run

Simulator (Monte Carlo table and plots in ./out):

```bash
python -m rada_align.sim --runs 500 --distance 12
```

Operator dashboard against the simulator, then open http://127.0.0.1:8000 :

```bash
python -m rada_align.ui --distance 2
```

Camera tools (native Windows Python, from the repo root with the venv active):

```bash
python scripts/measure_stability.py --fake --seconds 30
```

```bash
python scripts/measure_stability.py --device 0 --seconds 30
```

```bash
python scripts/calibrate_camera.py --images calib/chessboard --cols 9 --rows 6 --square-mm 25
```

`--fake` uses a synthetic spot (no camera is opened). `--device N` opens camera N; if the wrong camera opens, try 1, 2.

## Docs

- Start here: CLAUDE.md
- Challenge and analysis: docs/CHALLENGE.md
- Task list: docs/TASKS.md
- Progress log: docs/PROGRESS.md
- Questions for mentors: docs/OPEN_QUESTIONS.md
- Original Hebrew docs (challenge summary, prompt pack): docs/reference/
