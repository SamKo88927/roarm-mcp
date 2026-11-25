"""Live arm state for a 3D view: Server-Sent Events on a loopback HTTP port.

    GET /state   text/event-stream; one `data:` JSON line per update, about LIVE_HZ times a second while the
                 arm moves and every IDLE_PERIOD_S while it is idle.

Each event:
    {"t": unix seconds, "backend": "sim" | "real", "moving": bool, "joints_rad": [base, shoulder, elbow,
     wrist_pitch, wrist_roll, gripper], "tool": {...}, "min_clearance_mm": ...}

Joint angles use the same frames and order as the MCP tools (radians; gripper 0 = closed). The stream is read
only: nothing sent to this port can move the arm. It binds to 127.0.0.1 only.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from .arm import Arm

LIVE_HZ = 20
IDLE_PERIOD_S = 0.5

log = logging.getLogger("roarm_mcp.live")


class LiveState:
    """Serves the arm's latest measured state to any number of browser clients."""

    def __init__(self, arm: Arm, port: int, host: str = "127.0.0.1") -> None:
        self.arm = arm
        self._stop = threading.Event()
        live = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:
                log.debug(format, *args)

            def do_GET(self) -> None:
                if self.path.split("?")[0] != "/state":
                    self.send_error(404, "only /state is served")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")  # read-only data, loopback only
                self.end_headers()
                live.stream(self)

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.port = int(self.server.server_address[1])
        self._thread = threading.Thread(target=self.server.serve_forever, name="roarm-live", daemon=True)

    def start(self) -> LiveState:
        self._thread.start()
        return self

    def event(self) -> dict[str, Any]:
        latest = self.arm.latest_state()
        return {"t": round(time.time(), 3), **latest}

    def stream(self, handler: BaseHTTPRequestHandler) -> None:
        last_seq = -1
        last_sent = 0.0
        while not self._stop.is_set():
            seq = self.arm.state_seq
            moving = self.arm.is_moving()
            due = (time.monotonic() - last_sent) >= (1 / LIVE_HZ if moving else IDLE_PERIOD_S)
            if (seq != last_seq and due) or (time.monotonic() - last_sent) >= IDLE_PERIOD_S:
                if not moving and seq == last_seq:
                    self.arm.refresh_state()  # idle: re-read the arm, so hand-moved joints show up too
                try:
                    handler.wfile.write(f"data: {json.dumps(self.event())}\n\n".encode())
                    handler.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                last_seq, last_sent = self.arm.state_seq, time.monotonic()
            time.sleep(0.01)

    def close(self) -> None:
        self._stop.set()
        self.server.shutdown()
        self.server.server_close()
