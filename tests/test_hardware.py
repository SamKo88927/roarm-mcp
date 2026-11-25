"""The real-arm transport, driven against a fake firmware that speaks the same serial protocol."""

from __future__ import annotations

import threading
import time
from collections.abc import Iterator

import numpy as np
import pytest

from roarm_mcp import hardware
from roarm_mcp.arm import Arm
from roarm_mcp.hardware import (
    STOCK_GRIPPER,
    FakeFirmware,
    GripperMap,
    SerialTransport,
    TransportError,
    _parse_feedback,
)
from roarm_mcp.model import GRIPPER_OPEN, HOME


def _raw_home() -> list[float]:
    return [*HOME[:5], STOCK_GRIPPER.to_raw(HOME[5])]


@pytest.fixture(autouse=True)
def quick_grip(monkeypatch: pytest.MonkeyPatch) -> None:
    # The real arm answers ~90 ms late, so each closing step waits 150 ms and a full close takes ~13 s.
    # Tests use a faster fake and a shorter wait; the stepping logic is the same.
    monkeypatch.setattr(hardware, "GRIP_STEP_WAIT_S", 0.04)
    monkeypatch.setattr(hardware, "GRIP_SPD", 1000)  # a step then takes 0.02 s, half the wait, as on the real arm


@pytest.fixture
def firmware() -> FakeFirmware:
    return FakeFirmware(_raw_home(), latency_s=0.02)


@pytest.fixture
def transport(firmware: FakeFirmware) -> Iterator[SerialTransport]:
    t = SerialTransport(firmware, feedback_timeout_s=0.5)
    yield t
    t.close()


def _moves(firmware: FakeFirmware) -> list[dict[str, float]]:
    return [c for c in firmware.sent if c["T"] == 102]


def test_stock_gripper_map_round_trips() -> None:
    assert STOCK_GRIPPER.to_raw(0.0) == pytest.approx(3.13)
    assert STOCK_GRIPPER.to_raw(GRIPPER_OPEN) == pytest.approx(1.5708)
    for q in np.linspace(0, GRIPPER_OPEN, 7):
        assert STOCK_GRIPPER.from_raw(STOCK_GRIPPER.to_raw(q)) == pytest.approx(q)


def test_gripper_commands_never_pass_the_stops() -> None:
    claw = GripperMap(raw_closed=1.18, raw_open=2.58)
    assert claw.to_raw(-5.0) == pytest.approx(1.18)
    assert claw.to_raw(50.0) == pytest.approx(2.58)
    with pytest.raises(ValueError):
        GripperMap(raw_closed=1.0, raw_open=1.1)


def test_feedback_parser_ignores_noise() -> None:
    assert _parse_feedback(b"ESP-ROM boot") is None
    assert _parse_feedback(b'{"T":1051,"b":0') is None
    assert _parse_feedback(b'{"T":1051,"b":"nan","s":0,"e":0,"t":0,"r":0,"g":0}') is None
    assert _parse_feedback(b'{"T":1051,"b":0,"s":0,"e":1.5,"t":0,"r":0,"g":1.57}') is not None


def test_read_joints_matches_the_arm(transport: SerialTransport) -> None:
    q = transport.read_joints()
    assert np.allclose(q, HOME, atol=1e-3)


def test_move_runs_on_the_real_protocol(firmware: FakeFirmware, transport: SerialTransport) -> None:
    arm = Arm(transport)
    result = arm.move_to(250, 40, 120, speed=1.0)
    assert result["ok"], result
    tool = result["state"]["tool"]
    assert abs(tool["x_mm"] - 250) < 5 and abs(tool["y_mm"] - 40) < 5 and abs(tool["z_mm"] - 120) < 5
    moves = _moves(firmware)
    assert len(moves) == result["plan"]["waypoints"]
    assert all(40 <= m["spd"] <= 2500 and m["acc"] == 15 for m in moves)
    assert result["state"]["backend"] == "real"


def test_free_close_reaches_closed_in_small_steps(firmware: FakeFirmware, transport: SerialTransport) -> None:
    result = Arm(transport).gripper(0.0, speed=1.0)
    assert result["ok"], result
    assert "gripper_contact" not in result["executed"]
    hands = [m["hand"] for m in _moves(firmware)]
    assert hands[-1] == pytest.approx(3.13)
    assert all(abs(b - a) <= 0.0301 for a, b in zip([1.5708, *hands], hands, strict=False))  # never one big jump
    assert all(m["spd"] == hardware.GRIP_SPD for m in _moves(firmware))
    assert result["state"]["gripper_opening"] < 0.03


def test_close_stops_at_contact_and_backs_off(firmware: FakeFirmware, transport: SerialTransport) -> None:
    firmware.hand_stop = 2.4  # an object between the jaws, stock claw (raw grows as it closes)
    result = Arm(transport).gripper(0.0, speed=1.0)
    assert result["ok"], result
    assert result["executed"]["gripper_contact"] is True
    hands = [m["hand"] for m in _moves(firmware)]
    assert max(hands) < 2.4 + 0.12  # stopped within a few steps of the object, not driven to 3.13
    assert hands[-1] == pytest.approx(2.4 - 0.03, abs=0.01)  # holds just open of the contact point


def test_close_on_a_claw_that_opens_upward() -> None:
    claw = GripperMap(raw_closed=1.18, raw_open=2.58)
    firmware = FakeFirmware([*HOME[:5], 2.4], latency_s=0.02)
    firmware.hand_stop = 1.6  # a ~20 mm object
    transport = SerialTransport(firmware, claw, feedback_timeout_s=0.5)
    try:
        result = Arm(transport).gripper(0.0, speed=1.0)
        assert result["ok"], result
        assert result["executed"]["gripper_contact"] is True
        assert _moves(firmware)[-1]["hand"] == pytest.approx(1.6 + 0.03, abs=0.01)
        assert all(m["hand"] >= 1.18 for m in _moves(firmware))
    finally:
        transport.close()


def test_backend_without_grip_support_refuses_to_close(firmware: FakeFirmware) -> None:
    class NoGrip(SerialTransport):
        can_close_gripper = False

    transport = NoGrip(firmware, feedback_timeout_s=0.5)
    try:
        result = Arm(transport).gripper(0.0)
        assert result["refusal"]["code"] == "GRIPPER_CLOSE_UNSUPPORTED"
        assert _moves(firmware) == []
    finally:
        transport.close()


def test_custom_claw_opens_in_its_own_units() -> None:
    # A claw whose raw value grows as it opens, powered on slightly past its closed calibration point.
    claw = GripperMap(raw_closed=1.18, raw_open=2.58)
    firmware = FakeFirmware([*HOME[:5], 1.065])
    transport = SerialTransport(firmware, claw, feedback_timeout_s=0.5)
    try:
        arm = Arm(transport)
        assert arm.get_state()["state"]["gripper_opening"] == 0.0
        moved = arm.move_to(250, 0, 150, speed=1.0)
        assert moved["ok"], moved
        hands = {round(m["hand"], 3) for m in _moves(firmware)}
        assert hands == {1.18}  # an arm move holds the claw at its measured (clamped) position
        opened = arm.gripper(0.7, speed=1.0)
        assert opened["ok"], opened
        assert _moves(firmware)[-1]["hand"] == pytest.approx(1.18 + 0.7 * 1.4)
        assert all(1.18 <= m["hand"] <= 2.58 for m in _moves(firmware))
    finally:
        transport.close()


def test_stream_never_exceeds_twenty_waypoints_per_second(firmware: FakeFirmware, transport: SerialTransport) -> None:
    t0 = time.monotonic()
    result = Arm(transport).move_joints([30, 0, 90, 0, 0], speed=1.0)
    assert result["ok"], result
    assert time.monotonic() - t0 >= 0.05 * result["plan"]["waypoints"]


def test_refused_target_sends_no_motion(firmware: FakeFirmware, transport: SerialTransport) -> None:
    result = Arm(transport).move_to(230, 0, -30)
    assert not result["ok"]
    assert result["refusal"]["code"] == "TABLE_CLEARANCE"
    assert _moves(firmware) == []


def test_silent_arm_is_reported_and_nothing_moves(firmware: FakeFirmware, transport: SerialTransport) -> None:
    firmware.silent = True
    with pytest.raises(TransportError):
        transport.read_joints()
    result = Arm(transport).move_to(250, 0, 120)
    assert not result["ok"]
    assert result["refusal"]["code"] == "INTERNAL_ERROR"
    assert _moves(firmware) == []


def test_a_silent_port_is_reopened_once(firmware: FakeFirmware, transport: SerialTransport) -> None:
    firmware.silent = True
    firmware.recovers_on_reopen = True
    assert np.allclose(transport.read_joints(), HOME, atol=1e-3)
    assert firmware.reopened == 1


def test_tracking_watchdog_halts_a_lagging_arm(firmware: FakeFirmware, transport: SerialTransport) -> None:
    firmware.lag = 0.05  # the joints barely follow, as with a stalled servo
    result = Arm(transport).move_joints([60, 0, 90, 0, 0], speed=1.0)
    assert not result["ok"]
    assert result["stopped"]
    assert result["executed"]["tracking_error_deg"] > 20
    hold = _moves(firmware)[-1]
    measured = transport.read_joints()
    assert hold["base"] == pytest.approx(measured[0], abs=0.02)  # told to hold where it is, not to snap back


def test_stop_halts_a_running_motion(firmware: FakeFirmware, transport: SerialTransport) -> None:
    arm = Arm(transport)
    out: dict[str, object] = {}
    worker = threading.Thread(target=lambda: out.update(arm.move_joints([90, 0, 90, 0, 0], speed=0.3)))
    worker.start()
    time.sleep(0.4)
    arm.stop()
    worker.join(timeout=10)
    assert out["stopped"] is True
    assert transport.read_joints()[0] < np.radians(80)
