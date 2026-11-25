"""Episodes: what each motion planned, what the arm measured while it ran, and how it ended.

These are the raw material of the learning loop: replay one in the simulator (`replay`) and compare, so gaps
between the model and the real arm show up as numbers instead of surprises.

One JSON file per tool call that tried to move the arm, in the directory given to EpisodeRecorder:
    {"version": 1, "t": unix start, "tool", "arguments", "backend", "fitted_tool", "period_s",
     "start_joints": [6], "planned": [[6], ...], "measured": [[t_s, q0..q5], ...], "result": {...}}
`planned` is empty and `measured` holds only the start when the call was refused before moving.
"""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .sim import Joints, Vec

VERSION = 1


@dataclass
class Episode:
    tool: str
    arguments: dict[str, Any]
    backend: str
    fitted_tool: str | None
    start_joints: Vec
    clock: Callable[[], float] = time.monotonic  # the transport's clock: simulated time in simulation
    t: float = field(default_factory=time.time)
    period_s: float = 0.0
    planned: list[Vec] = field(default_factory=list[Vec])
    measured: list[tuple[float, Vec]] = field(default_factory=list[tuple[float, Vec]])
    _t0: float | None = None

    def sample(self, q: Joints) -> None:
        now = self.clock()
        if self._t0 is None:
            self._t0 = now
        self.measured.append((now - self._t0, np.asarray(q, dtype=np.float64).copy()))

    def to_dict(self, result: dict[str, Any]) -> dict[str, Any]:
        return {
            "version": VERSION,
            "t": round(self.t, 3),
            "tool": self.tool,
            "arguments": self.arguments,
            "backend": self.backend,
            "fitted_tool": self.fitted_tool,
            "period_s": self.period_s,
            "start_joints": _round(self.start_joints),
            "planned": [_round(q) for q in self.planned],
            "measured": [[round(t, 4), *_round(q)] for t, q in self.measured],
            "result": result,
        }


def _round(q: Joints) -> list[float]:
    return [round(float(v), 5) for v in q]


class EpisodeRecorder:
    """Writes each finished episode to its own JSON file. Thread-safe; never raises into a motion."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._n = 0

    def save(self, episode: Episode, result: dict[str, Any]) -> Path | None:
        with self._lock:
            self._n += 1
            stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(episode.t))
            path = self.directory / f"{stamp}-{self._n:04d}-{episode.tool}.json"
        try:
            path.write_text(json.dumps(episode.to_dict(result)), encoding="utf-8")
        except OSError:
            return None
        return path


def load(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("version") != VERSION:
        raise ValueError(f"{path}: unsupported episode version {data.get('version')!r}")
    return data
