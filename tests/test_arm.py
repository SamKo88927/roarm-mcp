from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np

from roarm_mcp.arm import Arm
from roarm_mcp.sim import SimTransport
from roarm_mcp.trace import Trace


def test_move_to_reaches_target(arm: Arm) -> None:
    r = arm.move_to(250, 40, 60, 90, 0)
    assert r["ok"] is True
    tool = r["state"]["tool"]
    assert abs(tool["x_mm"] - 250) < 3 and abs(tool["y_mm"] - 40) < 3 and abs(tool["z_mm"] - 60) < 3
    assert abs(tool["pitch_deg"] - 90) < 1.5


def test_refusal_leaves_arm_where_it_was(arm: Arm) -> None:
    before = arm.get_state()["state"]["tool"]
    r = arm.move_to(250, 0, 0, 90, 0)
    assert r["ok"] is False
    assert r["refusal"]["code"] == "TABLE_CLEARANCE"
    assert arm.get_state()["state"]["tool"] == before


def test_gripper_and_rest(arm: Arm) -> None:
    assert arm.gripper(0.0)["state"]["gripper_opening"] < 0.05
    assert arm.gripper(1.0)["state"]["gripper_opening"] > 0.95
    assert arm.rest()["ok"] is True


def test_move_joints_wrong_length(arm: Arm) -> None:
    assert arm.move_joints([0, 0, 0])["refusal"]["code"] == "BAD_JOINTS"


def test_stop_interrupts_a_running_motion() -> None:
    arm = Arm(SimTransport(realtime=True))
    result: dict[str, object] = {}
    worker = threading.Thread(target=lambda: result.update(arm.move_to(-200, 100, 60, 90, 0, speed=0.1)))
    worker.start()
    time.sleep(0.4)
    assert arm.get_state()["state"]["moving"] is True
    busy = arm.move_to(250, 0, 100)
    assert busy["refusal"]["code"] == "BUSY"
    arm.stop()
    worker.join(timeout=5)
    assert not worker.is_alive()
    assert result["ok"] is False and result["stopped"] is True
    executed = result["executed"]
    plan = result["plan"]
    assert isinstance(executed, dict) and isinstance(plan, dict)
    assert executed["waypoints"] < plan["waypoints"]


def test_trace_records_calls(arm: Arm) -> None:
    arm.move_to(250, 0, 60)
    arm.move_to(250, 0, 0)
    events = arm.trace.latest()
    assert [e["tool"] for e in events] == ["move_to", "move_to"]
    assert events[1]["result"]["refusal"]["code"] == "TABLE_CLEARANCE"


def test_many_moves_in_a_row_do_not_jam(arm: Arm) -> None:
    # Regression: the measured gripper settles a hair past "open"; later plans must not inherit that as a goal.
    for z in (150, 120, 60, 120, 150, 60):
        assert arm.move_to(250, 0, z)["ok"] is True
    assert arm.move_joints([30, 20, 90, 40, 0])["ok"] is True
    assert arm.gripper(1.0)["ok"] is True
    assert arm.rest()["ok"] is True


class SlippingTransport(SimTransport):
    """A transport whose servos give way: after a few waypoints the measured arm drops toward the table."""

    def follow(self, waypoints, period_s, should_stop=lambda: False, on_waypoint=lambda _q: None):  # type: ignore[override]
        seen = [0]

        def report(q):  # type: ignore[no-untyped-def]
            seen[0] += 1
            on_waypoint(np.array([0, 0.9, 1.8, 1.2, 0, 1.2]) if seen[0] > 3 else q)

        return super().follow(waypoints, period_s, should_stop, report)


def test_monitor_halts_when_what_runs_leaves_the_envelope() -> None:
    arm = Arm(SlippingTransport(realtime=False))
    r = arm.move_to(250, 0, 60)
    assert r["ok"] is False
    assert r["refusal"]["code"] == "ENVELOPE_BREACH"
    assert r["stopped"] is True
    assert r["executed"]["waypoints"] < r["plan"]["waypoints"]


def test_trace_file(tmp_path: Path) -> None:
    path = tmp_path / "t" / "trace.jsonl"
    arm = Arm(SimTransport(realtime=False), trace=Trace(path))
    arm.move_to(250, 0, 60)
    arm.stop()
    lines = [json.loads(line) for line in path.read_text().splitlines()]
    assert [e["tool"] for e in lines] == ["move_to", "stop"]


def test_following_a_suggestion_is_always_accepted() -> None:
    # Regression: suggestions sat on the reach boundary, so the rounded pose a model sends back was refused.
    rng = np.random.default_rng(7)
    arm = Arm(SimTransport(realtime=False))
    followed = 0
    for _ in range(25):
        start = (float(rng.uniform(150, 330)), float(rng.uniform(-150, 150)), float(rng.uniform(25, 150)))
        assert arm.move_to(*start)["ok"] is True
        target = (float(rng.uniform(-700, 700)), float(rng.uniform(-700, 700)), float(rng.uniform(-150, 500)))
        r = arm.move_to(*target)
        if r["ok"] or "suggestion" not in r["refusal"]:
            continue
        s = r["refusal"]["suggestion"]
        again = arm.move_to(s["x_mm"], s["y_mm"], s["z_mm"], s["pitch_deg"], s["roll_deg"])
        assert again["ok"] is True, (start, target, s, again.get("refusal"))
        followed += 1
    assert followed >= 10


def test_transport_fault_is_a_traced_refusal_and_releases_the_lock() -> None:
    transport = SimTransport(realtime=False)
    arm = Arm(transport)
    real_follow = transport.follow

    def broken_follow(*args: object, **kwargs: object) -> object:
        raise RuntimeError("serial link dropped")

    transport.follow = broken_follow  # type: ignore[method-assign]
    r = arm.move_to(250, 0, 60, 90, 0)
    assert r["ok"] is False
    assert r["refusal"]["code"] == "INTERNAL_ERROR"
    assert "serial link dropped" in r["refusal"]["message"]
    assert arm.trace.latest(1)[0]["result"]["refusal"]["code"] == "INTERNAL_ERROR"
    transport.follow = real_follow  # type: ignore[method-assign]
    assert arm.move_to(250, 0, 60, 90, 0)["ok"] is True  # the motion lock was released
