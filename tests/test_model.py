"""The primitive model must keep the kinematics of the reference model it was built from."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roarm_mcp.sim import HOME, ArmModel, Pose

# Tool centre point (world frame, mm) of the mesh-based reference model at these joint angles (rad).
REFERENCE_TCP = [
    ([0, 0, 1.5708, 0, 0], (343.67, 0.0, 343.73)),
    ([0.5, 0.3, 1.2, 0.4, 0], (349.26, 190.8, 280.66)),
    ([-0.8, -0.4, 2.0, -0.6, 1.0], (160.52, -165.28, 426.25)),
    ([0, 0.6, 1.8, 1.2, 0], (167.73, 0.0, 49.03)),
    ([1.2, -0.2, 0.6, -0.9, -2.0], (-10.55, -27.13, 648.93)),
]


@pytest.mark.parametrize(("q", "expected"), REFERENCE_TCP)
def test_forward_kinematics_matches_reference(q: list[float], expected: tuple[float, float, float]) -> None:
    pose = ArmModel().pose([*q, 0.5])
    assert np.allclose([pose.x, pose.y, pose.z], expected, atol=0.05)


def test_home_is_collision_free() -> None:
    model = ArmModel()
    assert model.contacts(HOME) == []
    assert model.table_clearance_mm(HOME) > 50


@pytest.mark.parametrize(
    "target",
    [
        Pose(250, 0, 60, math.pi / 2, 0),  # straight down: the most common pick pose
        Pose(150, -120, 20, math.pi / 2, 0.5),
        Pose(-200, 100, 60, math.pi / 2, 0),  # behind the base
        Pose(300, -50, 170, 0.3, 0),
        Pose(0, -300, 100, 0.5, 0),
    ],
)
def test_ik_converges(target: Pose) -> None:
    model = ArmModel()
    q, err_mm, err_rad = model.solve_ik(target, HOME)
    assert err_mm < 0.5
    assert err_rad < 1e-3
    pose = model.pose(q)
    assert abs(pose.pitch - target.pitch) < 1e-3


def test_ik_reports_failure_instead_of_hiding_it() -> None:
    _, err_mm, _ = ArmModel().solve_ik(Pose(600, 0, 0, 0, 0), HOME)
    assert err_mm > 50
