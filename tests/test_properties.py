"""Random commands never take what actually runs outside the envelope.

Clearance is measured on the stepped simulation (what really ran), not only on the plan.
"""

from __future__ import annotations

import math
from collections import Counter

from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from roarm_mcp.arm import Arm
from roarm_mcp.sim import SimTransport

ARM = Arm(SimTransport(realtime=False))
MIN_CLEARANCE = ARM.envelope.min_clearance_mm
OUTCOMES: Counter[str] = Counter()
SETTINGS = settings(
    max_examples=80,
    deadline=None,
    derandomize=True,  # same examples every run, so the "really moved" floor below is not flaky
    database=None,
    suppress_health_check=[HealthCheck.too_slow],
)

speed = st.floats(0.05, 1.0)
wild_angle = st.floats(-400, 400, allow_nan=False)


def _check(result: dict[str, object]) -> None:
    OUTCOMES["ok" if result["ok"] else "refused"] += 1
    if result["ok"]:
        executed = result["executed"]
        assert isinstance(executed, dict)
        assert executed["min_clearance_mm"] >= MIN_CLEARANCE
    state = ARM.get_state()["state"]
    assert state["min_clearance_mm"] >= MIN_CLEARANCE
    assert ARM.governor.arm.contacts(ARM.transport.read_joints()) == []


@SETTINGS
@given(
    st.floats(110, 440),
    st.floats(-math.pi, math.pi),
    st.floats(0, 330),
    st.floats(0, 90),
    st.floats(-90, 90),
    speed,
)
def test_move_to_inside_workspace(r: float, heading: float, z: float, pitch: float, roll: float, s: float) -> None:
    _check(ARM.move_to(r * math.cos(heading), r * math.sin(heading), z, pitch, roll, s))


@SETTINGS
@given(st.floats(-800, 800), st.floats(-800, 800), st.floats(-200, 800), wild_angle, wild_angle, speed)
def test_move_to_anywhere(x: float, y: float, z: float, pitch: float, roll: float, s: float) -> None:
    _check(ARM.move_to(x, y, z, pitch, roll, s))


@SETTINGS
@given(
    st.tuples(st.floats(-175, 175), st.floats(-88, 88), st.floats(1, 168), st.floats(-88, 88), st.floats(-175, 175)),
    speed,
)
def test_move_joints_inside_limits(joints: tuple[float, ...], s: float) -> None:
    _check(ARM.move_joints(list(joints), s))


@SETTINGS
@given(st.lists(wild_angle, min_size=5, max_size=5), speed)
def test_move_joints_anywhere(joints: list[float], s: float) -> None:
    _check(ARM.move_joints(joints, s))


@SETTINGS
@given(st.floats(-1, 2, allow_nan=False), speed)
def test_gripper(opening: float, s: float) -> None:
    result = ARM.gripper(opening, s)
    if not 0 <= opening <= 1:
        assert result["ok"] is False
    _check(result)


def test_the_property_tests_really_moved_the_arm() -> None:
    # Guards against a suite that passes only because everything was refused.
    assert OUTCOMES["ok"] >= 100, OUTCOMES
