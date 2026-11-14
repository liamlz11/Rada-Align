"""Run the operator dashboard against the simulator.

    python -m rada_align.ui                 then open http://127.0.0.1:8000
    python -m rada_align.ui --distance 2 --port 8000 --seed 3
"""

from __future__ import annotations

import argparse

import uvicorn

from ..config import load_config
from .server import create_app
from .session import Session


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m rada_align.ui", description=__doc__.split("\n")[0])
    p.add_argument("--config", default=None, help="path to config.yaml (default: repo root)")
    p.add_argument("--distance", type=float, default=None, help="distance in metres (default: config)")
    p.add_argument("--seed", type=int, default=0, help="random setup number")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args(argv)

    session = Session(load_config(args.config), args.seed, args.distance)
    print(f"RADA Align dashboard (SIMULATOR): http://{args.host}:{args.port}")
    uvicorn.run(create_app(session), host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
