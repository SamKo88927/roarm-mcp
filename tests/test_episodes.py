"""Recording episodes and replaying them in the simulator."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from roarm_mcp.arm import Arm
from roarm_mcp.episodes import EpisodeRecorder, load
from roarm_mcp.hardware import STOCK_GRIPPER, FakeFirmware, SerialTransport
from roarm_mcp.model import HOME
from roarm_mcp.replay import compare
from roarm_mcp.sim import SimTransport


def test_every_motion_is_recorded(tmp_path: Path) -> None:
    arm = Arm(SimTransport(realtime=False), recorder=EpisodeRecorder(tmp_path))
    assert arm.move_to(250, 40, 120, speed=1.0)["ok"]
    assert not arm.move_to(230, 0, -30)["ok"]
    files = sorted(tmp_path.glob("*.json"))
    assert len(files) == 2
    moved, refused = load(files[0]), load(files[1])
    assert moved["planned"] and len(moved["measured"]) > len(moved["planned"]) // 2
    assert moved["result"]["ok"] is True
    assert refused["planned"] == [] and refused["result"]["refusal"]["code"] == "TABLE_CLEARANCE"


def test_a_simulated_episode_replays_onto_itself(tmp_path: Path) -> None:
    arm = Arm(SimTransport(realtime=False), recorder=EpisodeRecorder(tmp_path))
    assert arm.move_joints([40, 10, 80, 20, 0], speed=1.0)["ok"]
    report = compare(load(next(tmp_path.glob("*.json"))))
    assert report["tool_error_mm"]["max"] < 1.0, report
    assert report["lag_s"] < 0.05


def test_replay_exposes_a_lagging_arm(tmp_path: Path) -> None:
    # A fake arm whose joints move at half the commanded speed: the replay must show a real gap.
    firmware = FakeFirmware([*HOME[:5], STOCK_GRIPPER.to_raw(HOME[5])], lag=0.5, latency_s=0.02)
    transport = SerialTransport(firmware, feedback_timeout_s=0.5)
    try:
        arm = Arm(transport, recorder=EpisodeRecorder(tmp_path))
        arm.move_joints([30, 0, 90, 0, 0], speed=0.5)
    finally:
        transport.close()
    report = compare(load(next(tmp_path.glob("*.json"))))
    assert report["tool_error_mm"]["max"] > 10, report
    assert np.isfinite(report["joint_error_deg"]["base"]["rms"])
