"""One live alignment session behind the dashboard.

Today the session owns a simulated rig. The controller only talks to the
Hardware interfaces, so task 8 swaps the simulated parts for real ones here
and nothing else in the dashboard changes.
"""

from __future__ import annotations

import threading
import time
from collections import deque

from ..config import Config
from ..controller import Controller
from ..sensors import build_simulation


class Session:
    def __init__(self, cfg: Config, seed: int = 0, distance_m: float | None = None):
        self.cfg = cfg
        self.distance_m = distance_m
        self.dt = 1.0 / cfg.camera.rate_hz
        self._seed = seed
        self._lock = threading.Lock()
        self._messages: deque[dict] = deque(maxlen=200)
        self._seq = 0
        self._events: deque[list] = deque(maxlen=500)
        self._thread: threading.Thread | None = None
        self._running = False
        self._build()

    def _build(self) -> None:
        self._sim = build_simulation(self.cfg, self._seed, self.distance_m)
        world = self._sim.world
        self._ctrl = Controller(
            self._sim.hardware, self.cfg, self._sim.calibration, clock=lambda: world.t
        )
        self._events.clear()

    # ------------------------------------------------------------- commands
    def start(self) -> None:
        with self._lock:
            self._ctrl.start()

    def stop(self) -> None:
        with self._lock:
            self._ctrl.stop()

    def realign(self) -> None:
        """Align again from where the rig is now."""
        with self._lock:
            self._ctrl.start()

    def set_hold(self, hold: bool) -> None:
        with self._lock:
            self._ctrl.set_hold(hold)

    def new_setup(self) -> None:
        """Simulator only: a fresh random manual setup (new seed)."""
        with self._lock:
            self._seed += 1
            self._build()

    # ----------------------------------------------------------------- loop
    def step_once(self) -> dict:
        with self._lock:
            self._sim.world.advance(self.dt)
            status = self._ctrl.step()
            message = status.to_dict()
            message["sim"] = True
            message["truth"] = self._sim.world.true_errors_deg()
            message["tolerance_deg"] = self.cfg.geometry.tolerance_deg
            message["setup_id"] = self._seed
            self._seq += 1
            message["seq"] = self._seq
            self._messages.append(message)
            self._events.extend(status.events)
            return message

    def run_in_background(self) -> None:
        if self._thread is not None:
            return
        self._running = True

        def loop():
            next_tick = time.monotonic()
            while self._running:
                self.step_once()
                next_tick += self.dt
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()  # fell behind; do not burst

        self._thread = threading.Thread(target=loop, name="rada-session", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -------------------------------------------------------------- reading
    @property
    def seq(self) -> int:
        return self._seq

    def latest(self) -> dict | None:
        with self._lock:
            return self._messages[-1] if self._messages else None

    def since(self, seq: int) -> list[dict]:
        """Messages newer than `seq` (bounded by the buffer length)."""
        with self._lock:
            return [m for m in self._messages if m["seq"] > seq]

    def report(self) -> dict:
        """Before / after of the last alignment, for the exported report."""
        with self._lock:
            last = self._messages[-1] if self._messages else {}
            tol = self.cfg.geometry.tolerance_deg
            after = last.get("after")
            return {
                "generated_unix_s": time.time(),
                "setup_id": self._seed,
                "sim": True,
                "distance_m": last.get("distance_m"),
                "tolerance_deg": tol,
                "state": last.get("state"),
                "aligned": last.get("aligned", False),
                "aligned_with_bias": last.get("aligned_with_bias", False),
                "fault_reason": last.get("fault_reason"),
                "before": last.get("before"),
                "after": after,
                # True only while the alignment that passed is still the current one.
                "passed": bool(after)
                and last.get("state") == "TRACK"
                and all(a["score_deg"] < tol for a in after.values()),
                "corrections": last.get("corrections", 0),
                "events": list(self._events),
            }
