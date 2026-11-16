"""Run the final-design demo (simulation).

    python -m rada_align.psd                  then open http://127.0.0.1:8001
    python -m rada_align.psd --distance 2 --seed 3 --port 8001
"""

from __future__ import annotations

import argparse

import uvicorn

from .config import load_psd_config
from .server import create_app
from .session import PsdSession


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m rada_align.psd", description=__doc__.split("\n")[0])
    p.add_argument("--config", default=None)
    p.add_argument("--distance", type=float, default=12.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8001)
    args = p.parse_args(argv)
    session = PsdSession(load_psd_config(args.config), args.seed, args.distance)
    print(f"RADA Align final-design demo (SIMULATION): http://{args.host}:{args.port}")
    uvicorn.run(create_app(session), host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
