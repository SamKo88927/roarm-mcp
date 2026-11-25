"""MuJoCo kinematics, collision queries and the simulated arm."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import mujoco as _mujoco
import numpy as np
from numpy.typing import NDArray

from .model import GRIPPER_OPEN, HOME, JOINT_NAMES, SHOULDER_HEIGHT_M, Tool, build_mjcf

# MuJoCo's compiled bindings ship no type stubs; keep the untyped surface behind one explicit name.
mj: Any = _mujoco

Vec = NDArray[np.float64]
Joints = Sequence[float] | Vec
ARM_DOF = 5  # base, shoulder, elbow, wrist_pitch, wrist_roll; index 5 is the gripper
SHOULDER_HEIGHT_MM = SHOULDER_HEIGHT_M * 1000


@dataclass(frozen=True)
class Pose:
    """Tool pose. x, y: mm from the base axis. z: mm of the tool centre point above the table.

    pitch: 0 = tool horizontal, pi/2 = tool pointing straight down. roll: wrist roll. Radians.
    """

    x: float
    y: float
    z: float
    pitch: float
    roll: float

    @property
    def radius(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass(frozen=True)
class Contact:
    geom1: str
    geom2: str


class ArmModel:
    """Kinematic view of the arm: forward/inverse kinematics and collision queries. Never moves anything.

    Not thread-safe: give each thread its own instance.
    """

    def __init__(self, tool: Tool | None = None) -> None:
        self.model = mj.MjModel.from_xml_string(build_mjcf(tool))
        self.data = mj.MjData(self.model)
        self.tcp = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, "tcp")
        self.table = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "table")
        self.lower: Vec = self.model.jnt_range[:, 0].copy()
        self.upper: Vec = self.model.jnt_range[:, 1].copy()
        base_body = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "base_link")
        # Collision geoms that move with the arm. The base box sits on the table by design.
        # Visual-only geoms (no contype/conaffinity) are not part of any safety check.
        self.moving_geoms: set[int] = {
            g
            for g in range(self.model.ngeom)
            if int(self.model.geom_bodyid[g]) not in (0, base_body)
            and (int(self.model.geom_contype[g]) or int(self.model.geom_conaffinity[g]))
        }
        # The turret only spins about the vertical axis, so its height above the table never changes: leave it out of
        # the table clearance, or it would pin the reported minimum at its own constant height. It stays in contacts.
        turret = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "link1")
        self.clearance_geoms: set[int] = {g for g in self.moving_geoms if int(self.model.geom_bodyid[g]) != turret}

    def _set(self, q: Joints) -> None:
        self.data.qpos[:] = np.asarray(q, dtype=np.float64)
        mj.mj_forward(self.model, self.data)

    def _tcp_pitch(self) -> float:
        axis = self.data.site_xmat[self.tcp].reshape(3, 3)[:, 2]
        return math.asin(float(np.clip(-axis[2], -1.0, 1.0)))

    def pose(self, q: Joints) -> Pose:
        self._set(q)
        p = self.data.site_xpos[self.tcp]
        return Pose(
            x=float(p[0] * 1000),
            y=float(p[1] * 1000),
            z=float(p[2] * 1000),
            pitch=self._tcp_pitch(),
            roll=float(q[4]),
        )

    def contacts(self, q: Joints) -> list[Contact]:
        """Contacts that involve at least one moving geom (self-collision or arm-table)."""
        self._set(q)
        out: list[Contact] = []
        for c in self.data.contact[: self.data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.moving_geoms or g2 in self.moving_geoms:
                out.append(Contact(self.model.geom(g1).name, self.model.geom(g2).name))
        return out

    def table_clearance_mm(self, q: Joints) -> float:
        """Smallest height above the table of any moving geom or of the tool centre point, in mm."""
        self._set(q)
        best = float(self.data.site_xpos[self.tcp][2])
        fromto = np.zeros(6)
        for g in self.clearance_geoms:
            dist = mj.mj_geomDistance(self.model, self.data, g, self.table, 0.2, fromto)
            best = min(best, float(dist))
        return best * 1000

    def solve_ik(self, target: Pose, seed: Joints, iters: int = 300) -> tuple[Vec, float, float]:
        """Damped least squares on position (3) + tool pitch (1).

        Returns (joints, position residual in mm, pitch residual in rad). Joints stay inside the joint ranges.
        The caller decides whether the residual is acceptable.
        """
        q = np.asarray(seed, dtype=np.float64).copy()
        q[4] = target.roll
        if math.hypot(target.x, target.y) > 1.0:
            q[0] = math.atan2(target.y, target.x)  # the arm is planar: the base angle points at the target
        goal = np.array([target.x / 1000, target.y / 1000, target.z / 1000])
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        active = [0, 1, 2, 3]
        heading = math.atan2(target.y, target.x)
        cp, sp = math.cos(target.pitch), math.sin(target.pitch)
        goal_axis = np.array([cp * math.cos(heading), cp * math.sin(heading), -sp])
        w = 0.1  # metres of position error that weigh as much as 1 rad of tool-axis error
        best_q, best_cost = q.copy(), math.inf
        for _ in range(iters):
            self._set(q)
            pos = self.data.site_xpos[self.tcp]
            axis = self.data.site_xmat[self.tcp].reshape(3, 3)[:, 2]
            e_pos = goal - pos
            e_axis = goal_axis - axis
            cost = float(np.linalg.norm(e_pos) + w * np.linalg.norm(e_axis))
            if cost < best_cost:
                best_q, best_cost = q.copy(), cost
            if np.linalg.norm(e_pos) < 2e-4 and np.linalg.norm(e_axis) < 1e-3:
                break
            mj.mj_jacSite(self.model, self.data, jacp, jacr, self.tcp)
            # The tool axis a changes as da = w x a, so da/dq_i = J_r[:, i] x a.
            jaxis = np.cross(jacr.T, axis).T
            jac = np.vstack([jacp[:, active], w * jaxis[:, active]])
            err = np.concatenate([e_pos, w * e_axis])
            dq = np.linalg.solve(jac.T @ jac + 1e-5 * np.eye(4), jac.T @ err)
            q[active] = np.clip(q[active] + dq, self.lower[active], self.upper[active])
        q = best_q
        self._set(q)
        final_pos = self.data.site_xpos[self.tcp]
        err_mm = float(np.linalg.norm(goal - final_pos) * 1000)
        e_pitch = target.pitch - self._tcp_pitch()
        return q, err_mm, abs(e_pitch)


def interpolate(start: Joints, goal: Joints, max_step: float) -> list[Vec]:
    """Straight line in joint space, sampled so no joint moves more than max_step radians per sample."""
    a = np.asarray(start, dtype=np.float64)
    b = np.asarray(goal, dtype=np.float64)
    n = max(1, math.ceil(float(np.max(np.abs(b - a))) / max_step))
    return [a + (b - a) * (i / n) for i in range(1, n + 1)]


@dataclass(frozen=True)
class FollowResult:
    stopped: bool
    waypoints_done: int
    tracking_error_rad: float
    gripper_contact: bool = False  # the jaws met something while closing and now hold it lightly


class SimTransport:
    """Physics-simulated arm. A hardware transport provides the same three methods."""

    name = "sim"

    def __init__(self, realtime: bool = True, tool: Tool | None = None) -> None:
        self.realtime = realtime
        self.model = mj.MjModel.from_xml_string(build_mjcf(tool))
        self.data = mj.MjData(self.model)
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        mj.mj_forward(self.model, self.data)

    def read_joints(self) -> Vec:
        return self.data.qpos.copy()

    def now(self) -> float:
        """Simulated seconds: episodes recorded faster than real time still carry true motion timing."""
        return float(self.data.time)

    def _step(self, n: int) -> None:
        start = time.monotonic()
        for i in range(n):
            mj.mj_step(self.model, self.data)
            if self.realtime and i % 10 == 9:
                ahead = (i + 1) * self.model.opt.timestep - (time.monotonic() - start)
                if ahead > 0:
                    time.sleep(ahead)

    def follow(
        self,
        waypoints: Sequence[Joints],
        period_s: float,
        should_stop: Callable[[], bool] = lambda: False,
        on_waypoint: Callable[[Vec], None] = lambda _q: None,
    ) -> FollowResult:
        """Track waypoints with the position actuators, one waypoint every period_s seconds.

        should_stop is polled between waypoints. On stop the arm holds where it is (no snap-back).
        on_waypoint receives the measured joints after each waypoint, so callers can monitor what really ran.
        """
        steps = max(1, round(period_s / self.model.opt.timestep))
        done = 0
        stopped = False
        for wp in waypoints:
            if should_stop():
                stopped = True
                self.data.ctrl[:] = self.data.qpos
                break
            self.data.ctrl[:] = np.asarray(wp, dtype=np.float64)
            self._step(steps)
            done += 1
            on_waypoint(self.data.qpos.copy())
        for _ in range(5):  # settle in short slices so the monitor sees the tail of the motion
            self._step(50)
            on_waypoint(self.data.qpos.copy())
        return FollowResult(stopped, done, float(np.max(np.abs(self.data.qpos - self.data.ctrl))))


__all__ = [
    "ARM_DOF",
    "GRIPPER_OPEN",
    "HOME",
    "JOINT_NAMES",
    "SHOULDER_HEIGHT_MM",
    "ArmModel",
    "Contact",
    "FollowResult",
    "Joints",
    "Pose",
    "SimTransport",
    "interpolate",
]
