"""The safety layer. Every motion is planned and checked here before any transport sees it.

The envelope is fixed when the server starts. No MCP tool can widen it.

A plan is the exact list of joint waypoints the transport will be asked to follow. The governor checks every one of
those waypoints (joint limits, self-collision, arm-table collision, clearance of every moving part and of the tool
centre point), so what is checked is what runs.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from .model import GRIPPER_OPEN, HOME, JOINT_NAMES
from .sim import ArmModel, Joints, Pose, Vec, interpolate

MEASUREMENT_TOL = 0.01  # rad; servo readings settle this close to a commanded limit


@dataclass(frozen=True)
class Envelope:
    """Limits every command must stay inside. Lengths in mm, angles in rad, speeds in rad/s."""

    min_clearance_mm: float = 15.0  # hard limit on what actually runs
    tracking_margin_mm: float = 3.0  # plans keep this much extra, for tracking error
    min_radius_mm: float = 100.0
    max_radius_mm: float = 450.0
    max_height_mm: float = 450.0
    joint_margin_rad: float = 0.02
    max_joint_speed: float = 1.5
    max_waypoint_step: float = 0.05
    ik_tolerance_mm: float = 1.0
    ik_tolerance_rad: float = 0.02

    @property
    def planning_clearance_mm(self) -> float:
        return self.min_clearance_mm + self.tracking_margin_mm

    def __post_init__(self) -> None:
        values = asdict(self)
        if not all(math.isfinite(v) and v >= 0 for v in values.values()):
            raise ValueError("envelope values must be finite and non-negative")
        if self.min_radius_mm >= self.max_radius_mm:
            raise ValueError("min_radius_mm must be below max_radius_mm")
        if not 0 < self.max_waypoint_step <= 0.15:
            raise ValueError("max_waypoint_step must be in (0, 0.15] rad")
        if not 0 < self.max_joint_speed <= 3.0:
            raise ValueError("max_joint_speed must be in (0, 3] rad/s")


@dataclass(frozen=True)
class Refusal:
    """Why a command was not executed, in a form a model can act on."""

    code: str
    message: str
    details: dict[str, Any] = field(default_factory=dict[str, Any])
    suggestion: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {"code": self.code, "message": self.message, "details": self.details}
        if self.suggestion is not None:
            out["suggestion"] = self.suggestion
        return out


@dataclass(frozen=True)
class Plan:
    goal: Vec
    waypoints: list[Vec]
    min_clearance_mm: float


def _round(v: float, nd: int = 1) -> float:
    return round(float(v), nd)


class Governor:
    """Plans and checks motions. Owns its own ArmModel; use from one thread at a time."""

    def __init__(self, envelope: Envelope | None = None) -> None:
        self.envelope = envelope or Envelope()
        self.arm = ArmModel()
        m = self.envelope.joint_margin_rad
        self.lower: Vec = self.arm.lower.copy()
        self.upper: Vec = self.arm.upper.copy()
        self.lower[:5] += m
        self.upper[:5] -= m
        self.lower[5], self.upper[5] = 0.0, GRIPPER_OPEN
        # IK searches only inside the margined limits, so a solution is never refused for the margin alone.
        self.arm.lower = self.lower.copy()
        self.arm.upper = self.upper.copy()

    # ------------------------------------------------------------------ single configurations
    def check_joints(self, q: Joints, tolerance: float = 0.0) -> Refusal | None:
        """Joint limits. `tolerance` (rad) absorbs measurement noise on waypoints that start from a reading."""
        if len(q) != len(JOINT_NAMES):
            return Refusal("BAD_JOINTS", f"expected {len(JOINT_NAMES)} joint values")
        for i, name in enumerate(JOINT_NAMES):
            v = float(q[i])
            if not math.isfinite(v):
                return Refusal("BAD_JOINTS", f"{name} is not a finite number")
            if not self.lower[i] - tolerance - 1e-9 <= v <= self.upper[i] + tolerance + 1e-9:
                if name == "gripper":
                    return Refusal("BAD_GRIPPER", "gripper opening must be between 0 and 1", {"joint": name})
                return Refusal(
                    "JOINT_LIMIT",
                    f"{name} would be {math.degrees(v):.1f} deg, outside "
                    f"[{math.degrees(self.lower[i]):.1f}, {math.degrees(self.upper[i]):.1f}] deg",
                    {"joint": name},
                )
        return None

    def clip(self, q: Joints) -> Vec:
        """Clip a measured configuration into the limits (to build goals from readings)."""
        return np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)

    def check_configuration(self, q: Joints, tolerance: float = 0.0) -> tuple[Refusal | None, float]:
        """Joint limits, collisions and clearance for one configuration. Returns (refusal, clearance_mm)."""
        bad = self.check_joints(q, tolerance)
        if bad is not None:
            return bad, math.nan
        contacts = self.arm.contacts(q)
        if contacts:
            c = contacts[0]
            code = "TABLE_COLLISION" if "table" in (c.geom1, c.geom2) else "SELF_COLLISION"
            return Refusal(code, f"{c.geom1} would touch {c.geom2}", {"geoms": [c.geom1, c.geom2]}), math.nan
        clearance = self.arm.table_clearance_mm(q)
        if clearance < self.envelope.planning_clearance_mm:
            return (
                Refusal(
                    "TABLE_CLEARANCE",
                    f"part of the arm would be {clearance:.1f} mm above the table; "
                    f"the minimum is {self.envelope.planning_clearance_mm:.0f} mm",
                    {"clearance_mm": _round(clearance)},
                ),
                clearance,
            )
        return None, clearance

    def check_target(self, target: Pose) -> Refusal | None:
        env = self.envelope
        values = (target.x, target.y, target.z, target.pitch, target.roll)
        if not all(math.isfinite(v) for v in values):
            return Refusal("BAD_TARGET", "target values must be finite numbers")
        r = target.radius
        if r < env.min_radius_mm:
            return Refusal(
                "TOO_CLOSE",
                f"target is {r:.0f} mm from the base axis; the minimum is {env.min_radius_mm:.0f} mm",
                {"radius_mm": _round(r)},
            )
        if r > env.max_radius_mm:
            return Refusal(
                "OUT_OF_REACH",
                f"target is {r:.0f} mm from the base axis; the maximum is {env.max_radius_mm:.0f} mm",
                {"radius_mm": _round(r)},
            )
        if target.z < env.planning_clearance_mm:
            return Refusal(
                "TABLE_CLEARANCE",
                f"target z is {target.z:.0f} mm; the tool must stay at least "
                f"{env.planning_clearance_mm:.0f} mm above the table",
                {"z_mm": _round(target.z)},
            )
        if target.z > env.max_height_mm:
            return Refusal(
                "TOO_HIGH",
                f"target z is {target.z:.0f} mm; the maximum is {env.max_height_mm:.0f} mm",
                {"z_mm": _round(target.z)},
            )
        return None

    # ------------------------------------------------------------------ plans
    def check_path(self, start: Joints, goal: Joints) -> Plan | Refusal:
        """Build the waypoint list from start to goal and check every waypoint."""
        bad = self.check_joints(goal)
        if bad is not None:
            return bad
        waypoints = interpolate(start, goal, self.envelope.max_waypoint_step)
        lowest = math.inf
        for i, wp in enumerate(waypoints):
            # The goal must be inside the limits exactly; intermediate waypoints start from a measured reading.
            bad, clearance = self.check_configuration(wp, 0.0 if i + 1 == len(waypoints) else MEASUREMENT_TOL)
            if bad is not None:
                details = dict(bad.details)
                details.update({"waypoint": i + 1, "of": len(waypoints)})
                where = "at the goal" if i + 1 == len(waypoints) else f"on the way (step {i + 1}/{len(waypoints)})"
                return Refusal(bad.code, f"{bad.message} {where}", details)
            lowest = min(lowest, clearance)
        return Plan(np.asarray(goal, dtype=np.float64), waypoints, lowest)

    def solve(self, target: Pose, current: Joints) -> Vec | Refusal:
        """Inverse kinematics with an explicit convergence check. Never returns a non-converged solution."""
        env = self.envelope
        best: tuple[float, Vec] | None = None
        worst_err = (math.inf, math.inf)
        for seed in (current, HOME):
            q, err_mm, err_rad = self.arm.solve_ik(target, self.clip(seed))
            q[5] = float(self.clip(current)[5])
            if err_mm <= env.ik_tolerance_mm and err_rad <= env.ik_tolerance_rad:
                cost = float(np.max(np.abs(q[:5] - np.asarray(current[:5]))))
                if best is None or cost < best[0]:
                    best = (cost, q)
            elif err_mm < worst_err[0]:
                worst_err = (err_mm, err_rad)
        if best is None:
            return Refusal(
                "UNREACHABLE",
                "no joint solution reaches this position and tool pitch "
                f"(closest miss {worst_err[0]:.0f} mm, {math.degrees(worst_err[1]):.0f} deg)",
                {"miss_mm": _round(worst_err[0]), "miss_deg": _round(math.degrees(worst_err[1]))},
            )
        return best[1]

    def plan_pose(self, current: Joints, target: Pose) -> Plan | Refusal:
        bad = self.check_target(target)
        if bad is not None:
            return bad
        goal = self.solve(target, current)
        if isinstance(goal, Refusal):
            return goal
        return self.check_path(current, goal)

    def plan_pose_with_fallback(self, current: Joints, target: Pose) -> Plan | Refusal:
        """Like plan_pose; on refusal, attach the farthest safe pose on the straight line toward the target."""
        result = self.plan_pose(current, target)
        if isinstance(result, Plan):
            return result
        start = self.arm.pose(current)

        def at(t: float) -> Pose:
            return Pose(
                start.x + (target.x - start.x) * t,
                start.y + (target.y - start.y) * t,
                start.z + (target.z - start.z) * t,
                start.pitch + (target.pitch - start.pitch) * t,
                start.roll + (target.roll - start.roll) * t,
            )

        lo, hi = 0.0, 1.0
        for _ in range(12):
            mid = (lo + hi) / 2
            if isinstance(self.plan_pose(current, at(mid)), Plan):
                lo = mid
            else:
                hi = mid
        if lo < 0.05:
            return result
        safe = at(lo)
        return Refusal(
            result.code,
            result.message,
            result.details,
            suggestion={
                "x_mm": _round(safe.x),
                "y_mm": _round(safe.y),
                "z_mm": _round(safe.z),
                "pitch_deg": _round(math.degrees(safe.pitch)),
                "roll_deg": _round(math.degrees(safe.roll)),
                "note": f"farthest checked-safe pose on the straight line toward the target ({lo:.0%} of the way)",
            },
        )

    def describe(self) -> dict[str, Any]:
        env = self.envelope
        return {
            "frame": {
                "x_mm": "forward from the base axis",
                "y_mm": "left from the base axis",
                "z_mm": "height of the tool centre point above the table",
                "pitch_deg": "0 = tool horizontal, 90 = tool pointing straight down",
                "roll_deg": "wrist roll",
            },
            "tool_radius_mm": [env.min_radius_mm, env.max_radius_mm],
            "tool_height_mm": [env.planning_clearance_mm, env.max_height_mm],
            "min_clearance_mm": env.planning_clearance_mm,
            "max_joint_speed_deg_s": _round(math.degrees(env.max_joint_speed)),
            "joint_limits_deg": {
                name: [_round(math.degrees(self.lower[i])), _round(math.degrees(self.upper[i]))]
                for i, name in enumerate(JOINT_NAMES[:5])
            },
            "gripper_opening": [0.0, 1.0],
            "fixed_at_startup": True,
        }
