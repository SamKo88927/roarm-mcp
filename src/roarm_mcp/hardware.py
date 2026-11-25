"""Real-arm transport: the RoArm-M3 JSON protocol over USB serial.

The Arm controller plans and checks every motion in MuJoCo, then hands the checked waypoints to a transport. This
transport streams them to the arm's ESP32 firmware and reports the joints the arm actually measured, so the
controller's monitor checks what really ran, not what was planned.

Protocol (Waveshare RoArm-M3 firmware, JSON lines at 115200 baud):
    {"T":102, "base":..., "shoulder":..., "elbow":..., "wrist":..., "roll":..., "hand":..., "spd":..., "acc":...}
        all joints, firmware radians; spd in servo steps/s (0 = unlimited), acc in 100 steps/s^2
    {"T":105}  request feedback; the arm answers {"T":1051, "b":..., "s":..., "e":..., "t":..., "r":..., "g":..., ...}

Hardware notes, measured on one arm:
- Opening the port with DTR or RTS low resets the ESP32 on macOS, and the arm then re-runs its power-on motion.
  The port is opened with both lines asserted.
- Feedback answers at about 11 Hz, so it is polled every FEEDBACK_PERIOD_S, not after every waypoint.
- The five arm joints map 1:1 to the simulator (forward kinematics agreed within 3 mm). The gripper does not:
  each claw needs a GripperMap measured on that claw.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from .model import GRIPPER_OPEN
from .sim import FollowResult, Joints, Vec

FEEDBACK_PERIOD_S = 0.12
STEPS_PER_RAD = 4096 / (2 * math.pi)
ACC = 15  # x100 steps/s^2; measured as smooth as unlimited, without hard starts
SETTLE_S = 1.0
SETTLE_TOL_RAD = 0.03


class TransportError(RuntimeError):
    """The arm did not answer, or answered with something unusable. No motion should continue."""


class Port(Protocol):
    """The part of a pyserial Serial object this module uses."""

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class GripperMap:
    """Linear map between this package's gripper joint (0 = closed, GRIPPER_OPEN = open) and firmware radians.

    raw_closed / raw_open are what the firmware reads with the jaws closed / fully open; either may be larger.
    Commands are clamped to [min(raw), max(raw)], so a bad target can never drive the claw past its stops.
    """

    raw_closed: float
    raw_open: float
    name: str = "custom"

    def __post_init__(self) -> None:
        if not (math.isfinite(self.raw_closed) and math.isfinite(self.raw_open)):
            raise ValueError("gripper raw values must be finite")
        if abs(self.raw_open - self.raw_closed) < 0.2:
            raise ValueError("raw_open and raw_closed must differ by at least 0.2 rad")

    def to_raw(self, q: float) -> float:
        t = float(np.clip(q / GRIPPER_OPEN, 0.0, 1.0))
        return self.raw_closed + t * (self.raw_open - self.raw_closed)

    def from_raw(self, raw: float) -> float:
        t = (raw - self.raw_closed) / (self.raw_open - self.raw_closed)
        return float(np.clip(t, 0.0, 1.0)) * GRIPPER_OPEN

    @classmethod
    def load(cls, path: Path) -> GripperMap:
        data = json.loads(path.read_text())
        return cls(float(data["raw_closed"]), float(data["raw_open"]), str(data.get("name", path.stem)))


# The stock RoArm-M3 claw: about 1.57 rad open and 3.13 rad closed (fully closed reads 3.13; 3.2 squeezes).
STOCK_GRIPPER = GripperMap(raw_closed=3.13, raw_open=1.5708, name="stock")


class LockedPort:
    """A serial port plus an exclusive lock file, so a second program cannot drive the same arm."""

    def __init__(self, port: Any, lock: Any) -> None:
        self._port, self._lock = port, lock

    def read(self, size: int = 1) -> bytes:
        return self._port.read(size)

    def write(self, data: bytes) -> int | None:
        return self._port.write(data)

    def close(self) -> None:
        try:
            self._port.close()
        finally:
            self._lock.close()  # closing the file releases the flock


def open_serial(device: str, baudrate: int = 115200) -> Port:  # pragma: no cover - needs the arm
    """Open the arm's serial port without resetting the ESP32 (DTR and RTS asserted before open)."""
    import fcntl
    import tempfile

    try:
        import serial  # pyright: ignore[reportMissingImports]
    except ImportError as exc:
        raise TransportError("the real arm needs pyserial: pip install 'roarm-mcp[real]'") from exc
    lock = (Path(tempfile.gettempdir()) / f"roarm-mcp-{Path(device).name}.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock.close()
        raise TransportError(f"{device} is already in use by another roarm-mcp process") from exc
    port: Any = serial.Serial()
    port.port, port.baudrate, port.timeout = device, baudrate, 0.05
    port.dtr = True
    port.rts = True
    try:
        port.open()
    except Exception as exc:
        lock.close()
        raise TransportError(f"cannot open {device}: {exc}") from exc
    return LockedPort(port, lock)


class SerialTransport:
    """Streams checked waypoints to the real arm and reports measured joints."""

    name = "real"

    def __init__(
        self,
        port: Port,
        gripper: GripperMap = STOCK_GRIPPER,
        feedback_timeout_s: float = 1.0,
        max_tracking_error_rad: float = 0.35,
    ) -> None:
        self.port = port
        self.gripper = gripper
        self.feedback_timeout_s = feedback_timeout_s
        self.max_tracking_error_rad = max_tracking_error_rad
        self._write_lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._latest_t = 0.0
        self._fresh = threading.Condition()
        self._closed = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name="roarm-serial-reader", daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------ wire
    def _read_loop(self) -> None:
        buf = b""
        while not self._closed.is_set():
            try:
                chunk = self.port.read(4096)
            except Exception:
                if self._closed.is_set():
                    return
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                frame = _parse_feedback(line)
                if frame is not None:
                    with self._fresh:
                        self._latest, self._latest_t = frame, time.monotonic()
                        self._fresh.notify_all()
            if len(buf) > 65536:  # never a frame: drop boot noise
                buf = b""

    def _send(self, command: dict[str, Any]) -> None:
        with self._write_lock:
            self.port.write((json.dumps(command) + "\n").encode())

    def _feedback(self) -> dict[str, Any]:
        asked = time.monotonic()
        self._send({"T": 105})
        with self._fresh:
            ok = self._fresh.wait_for(lambda: self._latest_t >= asked, timeout=self.feedback_timeout_s)
            if not ok or self._latest is None:
                raise TransportError(f"no feedback from the arm within {self.feedback_timeout_s:.1f} s")
            return self._latest

    def _to_raw(self, q: Joints) -> list[float]:
        q = np.asarray(q, dtype=np.float64)
        return [float(v) for v in q[:5]] + [self.gripper.to_raw(float(q[5]))]

    def _command(self, q: Joints, previous: Joints, period_s: float) -> None:
        b, s, e, w, r, h = self._to_raw(q)
        delta = float(np.max(np.abs(np.asarray(q, dtype=np.float64) - np.asarray(previous, dtype=np.float64))))
        # Ask for the speed that arrives in about one period, so motion stays continuous between waypoints.
        spd = int(np.clip(delta * STEPS_PER_RAD / max(period_s, 1e-3) * 1.15, 40, 2500))
        self._send(
            {"T": 102, "base": b, "shoulder": s, "elbow": e, "wrist": w, "roll": r, "hand": h, "spd": spd, "acc": ACC}
        )

    # ------------------------------------------------------------------ Transport
    def read_joints(self) -> Vec:
        frame = self._feedback()
        q = np.array([frame["b"], frame["s"], frame["e"], frame["t"], frame["r"], 0.0], dtype=np.float64)
        q[5] = self.gripper.from_raw(float(frame["g"]))
        return q

    def follow(
        self,
        waypoints: Sequence[Joints],
        period_s: float,
        should_stop: Callable[[], bool] = lambda: False,
        on_waypoint: Callable[[Vec], None] = lambda _q: None,
    ) -> FollowResult:
        """Send one waypoint every period_s; poll measured joints every FEEDBACK_PERIOD_S and pass them on.

        Stops early when should_stop() turns true or the measured joints fall more than max_tracking_error_rad
        behind the command; the arm is then told to hold its measured pose.
        """
        measured = self.read_joints()
        previous: Vec = measured
        start = time.monotonic()
        next_feedback = start + FEEDBACK_PERIOD_S
        done = 0
        stopped = False
        worst = 0.0
        for i, wp in enumerate(waypoints):
            if should_stop():
                stopped = True
                break
            target = np.asarray(wp, dtype=np.float64)
            self._command(target, previous, period_s)
            previous = target
            done += 1
            due = start + (i + 1) * period_s
            while (now := time.monotonic()) < due:
                if now >= next_feedback:
                    measured = self.read_joints()
                    on_waypoint(measured)
                    worst = max(worst, _arm_error(measured, target))
                    next_feedback = time.monotonic() + FEEDBACK_PERIOD_S
                    if worst > self.max_tracking_error_rad or should_stop():
                        break
                time.sleep(min(0.005, max(0.0, due - time.monotonic())))
            if worst > self.max_tracking_error_rad:
                stopped = True
                break
        if stopped:
            measured = self.read_joints()
            self._command(measured, measured, period_s)  # hold where it is, no snap-back
            on_waypoint(measured)
        else:
            deadline = time.monotonic() + SETTLE_S
            while True:
                measured = self.read_joints()
                on_waypoint(measured)
                # The gripper counts here, so a gripper move reports where the jaws ended up. A jaw blocked by an
                # object just runs out the settle time; only the arm joints feed the tracking watchdog.
                settled = float(np.max(np.abs(measured - previous))) < SETTLE_TOL_RAD
                if settled or time.monotonic() > deadline:
                    break
                time.sleep(FEEDBACK_PERIOD_S)
            worst = max(worst, _arm_error(measured, previous))
        return FollowResult(stopped, done, worst)

    def close(self) -> None:
        self._closed.set()
        with contextlib.suppress(Exception):
            self.port.close()
        self._reader.join(timeout=1.0)


def _arm_error(measured: Vec, target: Vec) -> float:
    return float(np.max(np.abs(measured[:5] - target[:5])))


def _parse_feedback(line: bytes) -> dict[str, Any] | None:
    text = line.decode(errors="replace").strip()
    if not text.startswith("{"):
        return None
    try:
        frame = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(frame, dict) or frame.get("T") != 1051:
        return None
    try:
        for key in ("b", "s", "e", "t", "r", "g"):
            value = float(frame[key])
            if not math.isfinite(value):
                return None
    except (KeyError, TypeError, ValueError):
        return None
    return frame


class FakeFirmware:
    """A stand-in for the arm's serial port that speaks the same protocol. For tests and --dry-run.

    Joints move toward the last T:102 target at the commanded servo speed. Every command is kept in `sent`.
    """

    def __init__(self, start: Sequence[float], lag: float = 1.0, log: Any = None) -> None:
        self.q = np.asarray(start, dtype=np.float64).copy()
        self.target = self.q.copy()
        self.speed = np.full(6, 1.0)
        self.lag = lag  # < 1 slows the joints, to test the tracking watchdog
        self.sent: list[dict[str, Any]] = []
        self.silent = False
        self.log = log  # a text stream: every motion command is echoed there (--dry-run)
        self._out = b"boot log line\n"
        self._t = time.monotonic()
        self._lock = threading.Lock()

    def _advance(self) -> None:
        now = time.monotonic()
        dt, self._t = now - self._t, now
        step = self.speed * dt * self.lag
        self.q += np.clip(self.target - self.q, -step, step)

    def write(self, data: bytes) -> int:
        with self._lock:
            self._advance()
            for line in data.decode().splitlines():
                cmd = json.loads(line)
                self.sent.append(cmd)
                if self.log is not None and cmd["T"] != 105:
                    print(f"dry-run would send: {line}", file=self.log, flush=True)
                if cmd["T"] == 102:
                    keys = ("base", "shoulder", "elbow", "wrist", "roll", "hand")
                    self.target = np.array([float(cmd[k]) for k in keys])
                    self.speed = np.full(6, max(float(cmd["spd"]), 1.0) / STEPS_PER_RAD)
                elif cmd["T"] == 105 and not self.silent:
                    b, s, e, t, r, g = (round(float(v), 4) for v in self.q)
                    frame = {"T": 1051, "b": b, "s": s, "e": e, "t": t, "r": r, "g": g, "tB": 0, "tG": 0}
                    self._out += (json.dumps(frame) + "\n").encode()
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            out, self._out = self._out[:size], self._out[size:]
        if not out:
            time.sleep(0.002)
        return out

    def close(self) -> None:
        pass
