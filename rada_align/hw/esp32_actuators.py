"""Actuators over the ESP32 serial protocol (firmware/src/main.cpp).

The firmware counts steps; this driver converts millimetres and degrees
using the step sizes in config.yaml, keeps the firmware watchdog fed, and
turns every "ERR ..." reply or timeout into an ActuatorError.

Must run from native Windows Python: WSL2 does not see serial ports by
default.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..config import Config
from ..interfaces import AXES, ActuatorError, Actuators

FIRMWARE_AXIS = {"x": "X", "z": "Z", "pan": "PAN", "tilt": "TILT"}


@dataclass(frozen=True)
class MountStatus:
    steps: dict[str, int]
    moving: dict[str, bool]
    limits: dict[str, bool]
    homed: dict[str, bool]
    estop: bool
    watchdog: bool


def parse_status(reply: str) -> MountStatus:
    """Parse 'OK pos=1,2,3,4 moving=0000 limits=0000 estop=0 wd=0 homed=0000'."""
    try:
        fields = dict(part.split("=", 1) for part in reply.split()[1:])
        steps = [int(v) for v in fields["pos"].split(",")]
        if len(steps) != len(AXES) or any(len(fields[k]) != len(AXES) for k in ("moving", "limits", "homed")):
            raise ValueError("wrong number of axes")

        def flags(key):
            return {axis: fields[key][i] == "1" for i, axis in enumerate(AXES)}

        return MountStatus(
            steps=dict(zip(AXES, steps)),
            moving=flags("moving"),
            limits=flags("limits"),
            homed=flags("homed"),
            estop=fields["estop"] == "1",
            watchdog=fields["wd"] == "1",
        )
    except (KeyError, ValueError, IndexError) as exc:
        raise ActuatorError(f"cannot parse STATUS reply {reply!r}: {exc}") from exc


class ESP32Actuators(Actuators):
    """`port` is anything with write(bytes), readline() -> bytes and close():
    a serial.Serial, or a simulated port for tests and dry runs."""

    def __init__(self, cfg: Config, port=None, keepalive: bool = True):
        self._cfg = {axis: getattr(cfg.actuators, axis) for axis in AXES}
        self._direction = dict(zip(AXES, cfg.hw.direction))
        for axis in AXES:
            if self._cfg[axis].step <= 0:
                raise ActuatorError(f"actuators.{axis}.step must be > 0 for real hardware")
            if self._direction[axis] not in (1, -1):
                raise ActuatorError("hw.direction entries must be +1 or -1")
        if port is None:
            import serial  # imported here so the simulator never needs a serial port

            port = serial.Serial(cfg.hw.serial_port, cfg.hw.baud, timeout=cfg.hw.timeout_s)
        self._port = port
        self._lock = threading.Lock()
        self._ping_interval = cfg.hw.ping_interval_s
        self._stop_keepalive = threading.Event()
        self._keepalive_error: str | None = None
        self._thread: threading.Thread | None = None
        self.ping()  # also arms the firmware watchdog and proves the link
        if keepalive:
            self._thread = threading.Thread(target=self._keepalive, name="esp32-keepalive", daemon=True)
            self._thread.start()

    # ------------------------------------------------------------- protocol
    def _command(self, text: str) -> str:
        with self._lock:
            self._port.write((text + "\n").encode("ascii"))
            raw = self._port.readline()
        if not raw:
            raise ActuatorError(f"timeout: no reply to {text!r}")
        reply = raw.decode("ascii", errors="replace").strip()
        if reply.startswith("OK"):
            return reply
        if reply.startswith("ERR"):
            raise ActuatorError(f"{text!r} rejected: {reply[3:].strip() or 'unknown reason'}")
        raise ActuatorError(f"unexpected reply to {text!r}: {reply!r}")

    def ping(self) -> None:
        self._command("PING")

    def _keepalive(self) -> None:
        while not self._stop_keepalive.wait(self._ping_interval):
            try:
                self.ping()
                self._keepalive_error = None
            except ActuatorError as exc:
                self._keepalive_error = str(exc)  # surfaced by the next command

    def _check_link(self) -> None:
        if self._keepalive_error:
            raise ActuatorError(f"keep-alive failed: {self._keepalive_error}")

    def status(self) -> MountStatus:
        return parse_status(self._command("STATUS"))

    # ------------------------------------------------------------ Actuators
    def _axis(self, axis: str) -> str:
        if axis not in FIRMWARE_AXIS:
            raise ActuatorError(f"unknown axis '{axis}'")
        return FIRMWARE_AXIS[axis]

    def _to_steps(self, axis: str, value: float) -> int:
        return round(value / self._cfg[axis].step) * self._direction[axis]

    def _from_steps(self, axis: str, steps: int) -> float:
        return steps * self._direction[axis] * self._cfg[axis].step

    def move_relative(self, axis: str, delta: float) -> None:
        name = self._axis(axis)
        self._check_link()
        target = self.position(axis) + delta
        lo, hi = self.limits(axis)
        if not lo <= target <= hi:
            raise ActuatorError(f"{axis} soft limit: target {target:.3f} outside [{lo}, {hi}]")
        self._command(f"MOVE {name} {self._to_steps(axis, delta)}")

    def position(self, axis: str) -> float:
        self._axis(axis)
        return self._from_steps(axis, self.status().steps[axis])

    def limits(self, axis: str) -> tuple[float, float]:
        self._axis(axis)
        return self._cfg[axis].min, self._cfg[axis].max

    def is_moving(self) -> bool:
        return any(self.status().moving.values())

    def home(self, axis: str) -> None:
        self._check_link()
        self._command(f"HOME {self._axis(axis)}")

    def stop(self) -> None:
        self._command("STOP")

    # ---------------------------------------------------------------- extras
    def set_speed(self, axis: str, speed: float, acceleration: float) -> None:
        """Max speed and acceleration in the axis' own units (mm or deg) per s, per s^2."""
        step = self._cfg[axis].step
        self._command(f"SPEED {self._axis(axis)} {speed / step:.1f} {acceleration / step:.1f}")

    def set_led(self, on: bool) -> None:
        """Beacon LED; fits the `led` callback of rada_align.vision.VisionCamera."""
        self._command("LED ON" if on else "LED OFF")

    def blink_led(self, hz: float) -> None:
        self._command(f"LED BLINK {hz:g}")

    def close(self) -> None:
        self._stop_keepalive.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self.stop()
        except ActuatorError:
            pass
        self._port.close()


class SimulatedESP32Port:
    """In-memory stand-in for the serial port: answers like the firmware.

    Mirrors firmware/src/main.cpp: same commands, same replies, same safety
    rules (soft limits, e-stop latch, watchdog). Motion is instantaneous
    unless `step_time_s` is set. For tests and for dry runs without a board.
    """

    AXES = ("X", "Z", "PAN", "TILT")

    def __init__(self, soft_min=(-100000, -100000, -13333, -13333), soft_max=(100000, 100000, 13333, 13333),
                 watchdog_s: float = 1.0, clock=time.monotonic, has_limit_switches: bool = False):
        self.pos = [0, 0, 0, 0]
        self.target = [0, 0, 0, 0]
        self.homed = [False] * 4
        self.soft_min, self.soft_max = list(soft_min), list(soft_max)
        self.watchdog_s, self._clock = watchdog_s, clock
        self.has_limit_switches = has_limit_switches
        self.last_ping: float | None = None
        self.estop_button = False  # set True to press the button
        self.estop_latched = False
        self.led = "OFF"
        self.silent = False  # set True to simulate a dead link
        self.log: list[str] = []
        self._replies: list[bytes] = []
        self.closed = False

    def _watchdog_tripped(self) -> bool:
        return self.last_ping is None or self._clock() - self.last_ping > self.watchdog_s

    def _safety(self) -> None:
        if self.estop_button:
            self.estop_latched = True
        if self.estop_latched or self._watchdog_tripped():
            self.target = list(self.pos)  # motion stops where it is

    def _blocked(self) -> str | None:
        if self.estop_latched:
            return "ESTOP"
        if self._watchdog_tripped():
            return "WATCHDOG"
        return None

    def _handle(self, text: str) -> str:
        self._safety()
        parts = text.upper().split()
        if not parts:
            return ""
        cmd, args = parts[0], parts[1:]
        if cmd == "PING":
            self.last_ping = self._clock()
            if self.estop_latched and not self.estop_button:
                self.estop_latched = False
            return "ERR ESTOP" if self.estop_latched else "OK"
        if cmd == "STATUS":
            self.pos = list(self.target)  # instantaneous motion completes between commands
            return ("OK pos=" + ",".join(str(p) for p in self.pos) + " moving=0000 limits=0000"
                    f" estop={int(self.estop_latched)} wd={int(self._watchdog_tripped())}"
                    " homed=" + "".join(str(int(h)) for h in self.homed))
        if cmd == "STOP":
            self.target = list(self.pos)
            return "OK"
        if cmd == "LED":
            if args[:1] in (["ON"], ["OFF"]):
                self.led = args[0]
                return "OK"
            if args[:1] == ["BLINK"] and len(args) == 2:
                self.led = f"BLINK {args[1]}"
                return "OK"
            return "ERR BAD_ARGUMENT"
        if cmd in ("MOVE", "MOVETO", "HOME", "SPEED"):
            if not args or args[0] not in self.AXES:
                return "ERR BAD_AXIS"
            i = self.AXES.index(args[0])
            if cmd == "SPEED":
                try:
                    v, a = float(args[1]), float(args[2])
                except (IndexError, ValueError):
                    return "ERR BAD_NUMBER"
                return "OK" if 0 < v <= 20000 and a > 0 else "ERR OUT_OF_RANGE"
            if cmd != "HOME":
                try:
                    value = int(args[1])
                except (IndexError, ValueError):
                    return "ERR BAD_NUMBER"
            blocked = self._blocked()
            if blocked:
                return f"ERR {blocked}"
            if cmd == "HOME":
                self.pos[i] = self.target[i] = 0
                self.homed[i] = True
                return "OK HOMING" if self.has_limit_switches else "OK ZEROED_HERE"
            target = value if cmd == "MOVETO" else self.target[i] + value
            if not self.soft_min[i] <= target <= self.soft_max[i]:
                return "ERR SOFT_LIMIT"
            self.target[i] = target
            return f"OK {target}"
        return "ERR UNKNOWN_COMMAND"

    # --- serial-port surface -------------------------------------------------
    def write(self, data: bytes) -> int:
        for text in data.decode("ascii").splitlines():
            self.log.append(text)
            reply = self._handle(text)
            if reply and not self.silent:
                self._replies.append((reply + "\n").encode("ascii"))
        return len(data)

    def readline(self) -> bytes:
        return self._replies.pop(0) if self._replies else b""

    def close(self) -> None:
        self.closed = True
