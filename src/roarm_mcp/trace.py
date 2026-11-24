"""Structured trace of every tool call: arguments, verdict and outcome."""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any


class Trace:
    """Keeps the last `keep` events in memory and, if a path is given, appends each event as one JSON line."""

    def __init__(self, path: Path | None = None, keep: int = 200) -> None:
        self.path = path
        self.events: deque[dict[str, Any]] = deque(maxlen=keep)
        self._lock = threading.Lock()
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)

    def record(self, tool: str, arguments: dict[str, Any], result: dict[str, Any]) -> None:
        event = {"t": round(time.time(), 3), "tool": tool, "arguments": arguments, "result": result}
        with self._lock:
            self.events.append(event)
            if self.path is not None:
                with self.path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(event, ensure_ascii=False) + "\n")

    def latest(self, n: int = 50) -> list[dict[str, Any]]:
        with self._lock:
            return list(self.events)[-n:]
