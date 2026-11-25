"""The arm controller: the only path from a tool call to a transport.

Each motion runs plan -> check every waypoint -> execute -> monitor what actually ran. One motion at a time;
`stop` may be called from any thread while a motion runs.
"""

from __future__ import annotations

import logging
import math
import threading
from collections.abc import Callable, Sequence
from typing import Any, Protocol

import numpy as np

from .model import GRIPPER_OPEN, HOME, JOINT_NAMES
from .safety import Envelope, Governor, Plan, Refusal
from .sim import ArmModel, FollowResult, Joints, Pose, Vec
from .trace import Trace

Result = dict[str, Any]

log = logging.getLogger("roarm_mcp")


class Transport(Protocol):
    name: str

    def read_joints(self) -> Vec: ...

    def follow(
        self,
        waypoints: Sequence[Joints],
        period_s: float,
        should_stop: Callable[[], bool] = ...,
        on_waypoint: Callable[[Vec], None] = ...,
    ) -> FollowResult: ...


def _deg(v: float) -> float:
    return round(math.degrees(v), 1)


class Arm:
    def __init__(self, transport: Transport, envelope: Envelope | None = None, trace: Trace | None = None):
        self.transport = transport
        self.governor = Governor(envelope)
        self.envelope = self.governor.envelope
        self.trace = trace or Trace()
        self._motion = threading.Lock()
        self._stop = threading.Event()
        self._monitor = ArmModel()  # used only inside the motion lock
        self._view = ArmModel()  # used only under _view_lock
        self._view_lock = threading.Lock()

    # ------------------------------------------------------------------ reading
    def _state(self, q: Joints) -> Result:
        with self._view_lock:
            pose = self._view.pose(q)
            clearance = self._view.table_clearance_mm(q)
        return {
            "backend": self.transport.name,
            "moving": self._motion.locked(),
            "tool": {
                "x_mm": round(pose.x, 1),
                "y_mm": round(pose.y, 1),
                "z_mm": round(pose.z, 1),
                "pitch_deg": _deg(pose.pitch),
                "roll_deg": _deg(pose.roll),
            },
            "joints_deg": {name: _deg(float(q[i])) for i, name in enumerate(JOINT_NAMES[:5])},
            "gripper_opening": round(float(np.clip(q[5] / GRIPPER_OPEN, 0.0, 1.0)), 3),
            "min_clearance_mm": round(clearance, 1),
        }

    def get_state(self) -> Result:
        return {"ok": True, "state": self._state(self.transport.read_joints())}

    def get_limits(self) -> Result:
        return {"ok": True, "limits": self.governor.describe()}

    # ------------------------------------------------------------------ acting
    def stop(self) -> Result:
        """Ask the running motion to halt at the next waypoint. The arm holds where it is."""
        was_moving = self._motion.locked()
        self._stop.set()
        result: Result = {
            "ok": True,
            "was_moving": was_moving,
            "note": "software stop; on real hardware the physical e-stop is the only power cut",
        }
        self.trace.record("stop", {}, result)
        return result

    def _run(
        self,
        tool: str,
        arguments: dict[str, Any],
        speed: float,
        planner: Callable[[Vec], Plan | Refusal],
    ) -> Result:
        if not self._motion.acquire(blocking=False):
            result: Result = {
                "ok": False,
                "refusal": Refusal("BUSY", "another motion is running; call stop or wait").to_dict(),
            }
            self.trace.record(tool, arguments, result)
            return result
        try:
            self._stop.clear()
            current = self.transport.read_joints()
            plan = planner(current)
            if isinstance(plan, Refusal):
                result = {"ok": False, "refusal": plan.to_dict(), "state": self._state(current)}
            else:
                result = self._execute(plan, speed)
        except Exception as exc:
            # A fault in planning or in the transport must not leave the model guessing or the lock held.
            log.exception("%s failed", tool)
            refusal = Refusal("INTERNAL_ERROR", f"{tool} failed inside the server: {type(exc).__name__}: {exc}")
            result = {"ok": False, "refusal": refusal.to_dict()}
        finally:
            self._motion.release()
        self.trace.record(tool, arguments, result)
        return result

    def _execute(self, plan: Plan, speed: float) -> Result:
        speed = float(np.clip(speed, 0.05, 1.0))
        period = self.envelope.max_waypoint_step / (self.envelope.max_joint_speed * speed)
        lowest = [math.inf]
        breach = threading.Event()

        def monitor(q: Vec) -> None:
            clearance = self._monitor.table_clearance_mm(q)
            lowest[0] = min(lowest[0], clearance)
            if clearance < self.envelope.min_clearance_mm or self._monitor.contacts(q):
                breach.set()  # what really ran left the envelope: halt now

        def should_stop() -> bool:
            return self._stop.is_set() or breach.is_set()

        followed = self.transport.follow(plan.waypoints, period, should_stop, monitor)
        final = self.transport.read_joints()
        if breach.is_set():
            refusal = Refusal(
                "ENVELOPE_BREACH",
                "the measured arm left the safety envelope during the motion and was halted",
                {"min_clearance_mm": round(lowest[0], 1)},
            )
        elif followed.stopped:
            refusal = Refusal("STOPPED", "motion was stopped before it finished")
        else:
            refusal = None
        return {
            "ok": refusal is None,
            "stopped": followed.stopped,
            "plan": {
                "waypoints": len(plan.waypoints),
                "duration_s": round(len(plan.waypoints) * period, 2),
                "planned_min_clearance_mm": round(plan.min_clearance_mm, 1),
            },
            "executed": {
                "waypoints": followed.waypoints_done,
                "min_clearance_mm": round(lowest[0], 1),
                "tracking_error_deg": _deg(followed.tracking_error_rad),
            },
            "state": self._state(final),
            **({"refusal": refusal.to_dict()} if refusal is not None else {}),
        }

    def move_to(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        pitch_deg: float = 90.0,
        roll_deg: float = 0.0,
        speed: float = 0.5,
    ) -> Result:
        target = Pose(x_mm, y_mm, z_mm, math.radians(pitch_deg), math.radians(roll_deg))
        args = {"x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm, "pitch_deg": pitch_deg, "roll_deg": roll_deg, "speed": speed}
        return self._run("move_to", args, speed, lambda q: self.governor.plan_pose_with_fallback(q, target))

    def move_joints(self, joints_deg: Sequence[float], speed: float = 0.5) -> Result:
        args = {"joints_deg": list(joints_deg), "speed": speed}
        if len(joints_deg) != 5:
            refusal = Refusal("BAD_JOINTS", "give 5 values: base, shoulder, elbow, wrist_pitch, wrist_roll")
            result: Result = {"ok": False, "refusal": refusal.to_dict()}
            self.trace.record("move_joints", args, result)
            return result

        def planner(q: Vec) -> Plan | Refusal:
            goal = np.array([math.radians(v) for v in joints_deg] + [float(self.governor.clip(q)[5])])
            return self.governor.check_path(q, goal)

        return self._run("move_joints", args, speed, planner)

    def gripper(self, opening: float, speed: float = 0.5) -> Result:
        args = {"opening": opening, "speed": speed}

        def planner(q: Vec) -> Plan | Refusal:
            if not (math.isfinite(opening) and 0.0 <= opening <= 1.0):
                return Refusal("BAD_GRIPPER", "opening must be between 0 (closed) and 1 (open)")
            current = float(np.clip(q[5] / GRIPPER_OPEN, 0.0, 1.0))
            if opening < current - 0.02 and not getattr(self.transport, "can_close_gripper", True):
                return Refusal(
                    "GRIPPER_CLOSE_UNSUPPORTED",
                    "this backend cannot close the gripper yet (no contact detection or torque limit); "
                    "opening is allowed",
                    {"current_opening": round(current, 3)},
                )
            goal = self.governor.clip(q)
            goal[5] = opening * GRIPPER_OPEN
            return self.governor.check_path(q, goal)

        return self._run("gripper", args, speed, planner)

    def rest(self, speed: float = 0.5) -> Result:
        return self._run("rest", {"speed": speed}, speed, lambda q: self.governor.check_path(q, np.array(HOME)))
