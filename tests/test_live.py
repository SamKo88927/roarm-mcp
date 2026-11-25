"""The live state stream a 3D view subscribes to."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

from roarm_mcp.arm import Arm
from roarm_mcp.live import LiveState
from roarm_mcp.sim import SimTransport


@pytest.fixture
def live() -> Iterator[LiveState]:
    arm = Arm(SimTransport(realtime=True))
    server = LiveState(arm, port=0).start()
    yield server
    server.close()


def _events(port: int, count: int) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=10) as resp:
        assert resp.headers["Content-Type"] == "text/event-stream"
        for raw in resp:
            line = raw.decode().strip()
            if line.startswith("data: "):
                out.append(json.loads(line[6:]))
                if len(out) >= count:
                    break
    return out


def test_idle_stream_reports_the_measured_state(live: LiveState) -> None:
    (event,) = _events(live.port, 1)
    assert event["backend"] == "sim"
    assert event["moving"] is False
    joints = event["joints_rad"]
    assert isinstance(joints, list) and len(joints) == 6


def test_stream_follows_a_motion(live: LiveState) -> None:
    mover = threading.Thread(target=lambda: live.arm.move_joints([40, 0, 90, 0, 0], speed=1.0))
    mover.start()
    events = _events(live.port, 12)
    mover.join(timeout=20)
    bases = [e["joints_rad"][0] for e in events]  # type: ignore[index]
    assert any(e["moving"] for e in events)
    assert max(bases) - min(bases) > 0.1  # the base visibly turned while we watched


def test_only_state_is_served(live: LiveState) -> None:
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(f"http://127.0.0.1:{live.port}/move", timeout=5)
    assert e.value.code == 404
