from __future__ import annotations

import math

import numpy as np
import pytest

from roarm_mcp.safety import Envelope, Governor, Plan, Refusal
from roarm_mcp.sim import HOME, Pose

CURRENT = list(HOME)


@pytest.fixture(scope="module")
def gov() -> Governor:
    return Governor()


def refusal(result: Plan | Refusal) -> Refusal:
    assert isinstance(result, Refusal), result
    return result


@pytest.mark.parametrize(
    ("target", "code"),
    [
        (Pose(600, 0, 100, math.pi / 2, 0), "OUT_OF_REACH"),
        (Pose(30, 0, 100, math.pi / 2, 0), "TOO_CLOSE"),
        (Pose(250, 0, 5, math.pi / 2, 0), "TABLE_CLEARANCE"),
        (Pose(250, 0, 600, 0, 0), "TOO_HIGH"),
        (Pose(440, 0, 400, math.pi / 2, 0), "UNREACHABLE"),
        (Pose(float("nan"), 0, 100, 0, 0), "BAD_TARGET"),
    ],
)
def test_target_refusals(gov: Governor, target: Pose, code: str) -> None:
    assert refusal(gov.plan_pose(CURRENT, target)).code == code


def test_non_converged_ik_is_never_a_plan(gov: Governor) -> None:
    # Inside the radius/height box, but no joint solution exists for this pitch.
    r = refusal(gov.plan_pose(CURRENT, Pose(440, 0, 400, math.pi / 2, 0)))
    assert r.code == "UNREACHABLE"
    assert r.details["miss_mm"] > gov.envelope.ik_tolerance_mm


def test_every_waypoint_is_checked(gov: Governor) -> None:
    # Both ends are safe; the straight joint-space path between them dips below the clearance limit.
    start = [-0.06, 0.59, 2.09, 0.95, 0.0, 1.2]
    goal = [0.93, 0.38, 2.54, -1.39, 0.0, 1.2]
    assert gov.check_configuration(start)[0] is None
    assert gov.check_configuration(goal)[0] is None
    r = refusal(gov.check_path(start, goal))
    assert r.code == "TABLE_CLEARANCE"
    assert r.details["waypoint"] < r.details["of"]


def test_plan_waypoints_are_what_gets_checked(gov: Governor) -> None:
    plan = gov.plan_pose(CURRENT, Pose(250, 0, 60, math.pi / 2, 0))
    assert isinstance(plan, Plan)
    steps = np.diff(np.vstack([CURRENT, *plan.waypoints]), axis=0)
    assert np.max(np.abs(steps)) <= gov.envelope.max_waypoint_step + 1e-9
    assert np.allclose(plan.waypoints[-1], plan.goal)
    assert plan.min_clearance_mm >= gov.envelope.planning_clearance_mm


def test_joint_limit_refusal(gov: Governor) -> None:
    r = refusal(gov.check_path(CURRENT, [0, 0, 1.5708, math.radians(95), 0, 1.2]))
    assert r.code == "JOINT_LIMIT"
    assert r.details["joint"] == "wrist_pitch"


def test_refusal_suggests_a_checked_safe_pose(gov: Governor) -> None:
    r = refusal(gov.plan_pose_with_fallback(CURRENT, Pose(250, 0, 5, math.pi / 2, 0)))
    assert r.suggestion is not None
    s = r.suggestion
    safe = Pose(s["x_mm"], s["y_mm"], s["z_mm"], math.radians(s["pitch_deg"]), math.radians(s["roll_deg"]))
    assert isinstance(gov.plan_pose(CURRENT, safe), Plan)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_clearance_mm": -1.0},
        {"min_radius_mm": 500.0},
        {"max_waypoint_step": 0.5},
        {"max_joint_speed": 10.0},
        {"max_height_mm": math.inf},
    ],
)
def test_envelope_rejects_unsafe_settings(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        Envelope(**kwargs)


def test_envelope_is_immutable() -> None:
    env = Envelope()
    with pytest.raises(AttributeError):
        env.min_clearance_mm = 0.0  # type: ignore[misc]
