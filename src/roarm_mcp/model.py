"""Primitive MuJoCo model of the RoArm-M3, generated from kinematic numbers.

No vendor meshes are shipped. The kinematic tree (link offsets, joint axes, joint ranges) matches the arm, and every
link is approximated by a capsule, cylinder or box for collision. Those collision shapes are hidden (geom group 3).
What you see is a separate, visual-only shell (group 1, no contacts, no mass) built from boxes and cylinders in the
arm's colours: white aluminium brackets, black servos, a black forearm tube. The link offsets are taken
from a mesh-based reference model of the arm; tests/test_model.py pins the tool position at several joint sets to
that reference. On the author's arm, the reference reproduced the firmware's reported tool position within 3 mm at
three poses. Treat that as one measurement, not a guarantee for every unit.

Frames
------
World: z up, the table plane is z = 0, the arm base axis is the z axis.
Arm frame (used by the public API and by the firmware): origin on the shoulder axis, millimetres.
The shoulder axis is SHOULDER_HEIGHT_M above the table.

Gripper convention (this package only): joint value 0 = closed, GRIPPER_OPEN = fully open.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

SHOULDER_HEIGHT_M = 0.0701 + 0.051959
GRIPPER_OPEN = 1.2
TCP_OFFSET_M = 0.115428


@dataclass(frozen=True)
class JointSpec:
    name: str
    lower: float
    upper: float
    kp: float
    kv: float
    force: float


# Ranges are the intersection of the mechanical range and the range validated on hardware.
JOINTS: tuple[JointSpec, ...] = (
    JointSpec("base", -3.1416, 3.1416, 45, 7, 2.0),
    JointSpec("shoulder", -1.5708, 1.5708, 85, 10, 2.4),
    JointSpec("elbow", 0.0, 2.95, 65, 8, 2.0),
    JointSpec("wrist_pitch", -1.5708, 1.5708, 25, 3, 1.2),
    JointSpec("wrist_roll", -3.1416, 3.1416, 18, 2.5, 1.0),
    JointSpec("gripper", 0.0, GRIPPER_OPEN, 10, 1.2, 0.5),
)
JOINT_NAMES: tuple[str, ...] = tuple(j.name for j in JOINTS)
HOME = (0.0, 0.0, 1.5708, 0.0, 0.0, GRIPPER_OPEN)

VISUAL_GROUP = 1  # visual-only geoms: no contacts, no mass
COLLISION_GROUP = 3  # collision geoms: hidden in renders by default


@dataclass(frozen=True)
class ToolBox:
    """A collision box fixed to the wrist-roll link (link5). Millimetres, in the link5 frame:
    z runs along the tool axis toward the fingertips; the stock tool centre point is at z = 115.4."""

    name: str
    center_mm: tuple[float, float, float]
    half_size_mm: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.name.replace("_", "").isalnum():
            raise ValueError(f"tool box name {self.name!r}: use letters, digits and _")
        values = (*self.center_mm, *self.half_size_mm)
        if len(self.center_mm) != 3 or len(self.half_size_mm) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError(f"tool box {self.name!r}: center_mm and half_size_mm need 3 finite numbers each")
        if not all(0 < v <= 200 for v in self.half_size_mm):
            raise ValueError(f"tool box {self.name!r}: half sizes must be in (0, 200] mm")


@dataclass(frozen=True)
class Tool:
    """Extra collision geometry for what is fitted to the wrist: a custom claw, a camera, a cable clip.

    The boxes are added to the stock collision shapes, so every safety check also keeps them clear of the
    table and of the arm. They are hidden in renders.
    """

    name: str
    boxes: tuple[ToolBox, ...] = field(default=())

    @classmethod
    def load(cls, path: Path) -> Tool:
        data = json.loads(path.read_text())
        boxes = tuple(
            ToolBox(str(b["name"]), tuple(map(float, b["center_mm"])), tuple(map(float, b["half_size_mm"])))  # type: ignore[arg-type]
            for b in data["boxes"]
        )
        if not boxes:
            raise ValueError(f"{path}: a tool needs at least one box")
        return cls(str(data.get("name", path.stem)), boxes)

    def mjcf(self) -> str:
        return "".join(
            f'<geom name="tool_{b.name}" type="box" pos="{b.center_mm[0] / 1000} {b.center_mm[1] / 1000} '
            f'{b.center_mm[2] / 1000}" size="{b.half_size_mm[0] / 1000} {b.half_size_mm[1] / 1000} '
            f'{b.half_size_mm[2] / 1000}" density="0"/>'
            for b in self.boxes
        )


def _joint(name: str) -> str:
    spec = next(j for j in JOINTS if j.name == name)
    return f'<joint name="{name}" type="hinge" axis="0 0 1" range="{spec.lower} {spec.upper}"/>'


def build_mjcf(tool: Tool | None = None) -> str:
    """Return the MJCF XML for the arm on a table, with the tool's collision boxes on the wrist if given."""
    actuators = "\n    ".join(
        f'<position name="{j.name}" joint="{j.name}" kp="{j.kp}" kv="{j.kv}" forcerange="{-j.force} {j.force}"/>'
        for j in JOINTS
    )
    home = " ".join(str(v) for v in HOME)
    return f"""<mujoco model="roarm-m3 (primitive)">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <visual>
    <global azimuth="135" elevation="-22" offwidth="640" offheight="480"/>
    <headlight ambient="0.32 0.32 0.34" diffuse="0.35 0.35 0.36" specular="0.1 0.1 0.1"/>
    <quality shadowsize="4096" offsamples="8"/>
    <map znear="0.01" shadowclip="1.2"/>
  </visual>
  <default>
    <joint damping="0.18" armature="0.004" frictionloss="0.025"/>
    <geom contype="1" conaffinity="1" density="1200" friction="0.7 0.01 0.001" group="{COLLISION_GROUP}"/>
    <position ctrllimited="true" forcelimited="true" inheritrange="1"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" density="0" group="{VISUAL_GROUP}" material="white"/>
      <default class="servo"><geom material="servo"/></default>
      <default class="tube"><geom material="tube"/></default>
      <default class="metal"><geom material="metal"/></default>
    </default>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.86 0.88 0.91" rgb2="0.52 0.56 0.62"
             width="512" height="512"/>
    <texture name="desk" type="2d" builtin="checker" rgb1="0.46 0.44 0.42" rgb2="0.43 0.41 0.39"
             width="512" height="512"/>
    <material name="table" texture="desk" texrepeat="10 10" reflectance="0" specular="0.05"/>
    <material name="white" rgba="0.93 0.94 0.95 1" specular="0.35" shininess="0.5"/>
    <material name="servo" rgba="0.07 0.075 0.085 1" specular="0.25" shininess="0.4"/>
    <material name="tube" rgba="0.03 0.03 0.035 1" specular="0.7" shininess="0.9"/>
    <material name="metal" rgba="0.62 0.64 0.67 1" specular="0.8" shininess="0.8"/>
    <material name="pcb" rgba="0.08 0.36 0.2 1" specular="0.3"/>
  </asset>
  <worldbody>
    <light name="key" pos="0.6 -0.8 1.6" dir="-0.35 0.45 -1" directional="true" castshadow="true"
           diffuse="0.62 0.61 0.6" specular="0.25 0.25 0.25"/>
    <light name="fill" pos="-0.8 0.6 1.2" dir="0.5 -0.4 -1" directional="true" castshadow="false"
           diffuse="0.22 0.23 0.25" specular="0 0 0"/>
    <geom name="table" type="plane" size="1 1 0.02" material="table" group="0"/>
    <camera name="overview" pos="0.62 -0.62 0.48" xyaxes="0.707 0.707 0 -0.32 0.32 0.892"/>
    <body name="base_link" pos="0 0 0.0701">
      <geom name="base" type="box" size="0.072 0.072 0.034" pos="0 0 -0.036"/>
      <geom class="visual" type="box" size="0.068 0.068 0.003" pos="0 0 -0.0672"/>
      <geom class="visual" type="box" size="0.05 0.046 0.0008" pos="0 0 -0.0634" material="pcb"/>
      <geom class="servo" type="box" size="0.0226 0.0124 0.0175" pos="0 0 -0.035"/>
      <geom class="visual" type="box" size="0.03 0.0015 0.028" pos="0 0.0155 -0.034"/>
      <geom class="visual" type="box" size="0.03 0.0015 0.028" pos="0 -0.0155 -0.034"/>
      <geom class="visual" type="box" size="0.034 0.022 0.0015" pos="0 0 -0.0045"/>
      <body name="link1" gravcomp="1">
        {_joint("base")}
        <geom name="turret" type="cylinder" size="0.046 0.027" pos="0 0 0.020"/>
        <geom class="visual" type="cylinder" size="0.032 0.003" pos="0 0 0.002"/>
        <geom class="visual" type="box" size="0.024 0.0015 0.033" pos="0 0.031 0.028"/>
        <geom class="visual" type="box" size="0.024 0.0015 0.033" pos="0 -0.031 0.028"/>
        <geom class="visual" type="box" size="0.024 0.031 0.0015" pos="0 0 0.0055"/>
        <geom class="servo" type="box" size="0.0226 0.0175 0.0124" pos="0.006 0 0.04"/>
        <geom class="metal" type="cylinder" size="0.007 0.034" pos="0 0 0.051959" euler="1.5708 0 0"/>
        <body name="link2" gravcomp="1" pos="0 0 0.051959" quat="0.499998163 -0.5 -0.5 -0.500001837">
          {_joint("shoulder")}
          <geom name="upper_arm" type="capsule" size="0.026" fromto="0.015 0.008 0 0.220 0.028 0"/>
          <geom class="visual" type="box" size="0.112 0.016 0.0018" pos="0.118 0.016 0.0215" euler="0 0 0.0964"/>
          <geom class="visual" type="box" size="0.112 0.016 0.0018" pos="0.118 0.016 -0.0215" euler="0 0 0.0964"/>
          <geom class="visual" type="box" size="0.104 0.0018 0.0215" pos="0.118 0.002 0" euler="0 0 0.0964"/>
          <geom class="servo" type="box" size="0.0226 0.0124 0.0175" pos="0.226 0.03 0"/>
          <body name="link3" gravcomp="1" pos="0.236815 0.030002 0" quat="0.707105483 0 0 0.70710808">
            {_joint("elbow")}
            <geom name="forearm" type="capsule" size="0.023" fromto="0 -0.008 0 0 -0.132 0"/>
            <geom class="visual" type="box" size="0.016 0.012 0.0215" pos="0 -0.006 0"/>
            <geom class="tube" type="cylinder" size="0.0075" fromto="0 -0.016 0 0 -0.128 0"/>
            <geom class="visual" type="box" size="0.013 0.006 0.016" pos="0 -0.024 0"/>
            <geom class="visual" type="box" size="0.013 0.006 0.016" pos="0 -0.126 0"/>
            <geom class="metal" type="cylinder" size="0.006 0.024" pos="0 -0.1446 0"/>
            <body name="link4" gravcomp="1" pos="0 -0.144586 0">
              {_joint("wrist_pitch")}
              <geom name="wrist" type="capsule" size="0.021" fromto="0 -0.005 0 0.012 -0.047 0"/>
              <geom class="servo" type="box" size="0.0124 0.0226 0.0175" pos="0.004 -0.022 0" euler="0 0 0.27"/>
              <geom class="visual" type="box" size="0.016 0.006 0.02" pos="0.012 -0.046 0" euler="0 0 0.27"/>
              <body name="link5" gravcomp="1" pos="0.015147 -0.053653 0" quat="0.499998163 0.5 0.5 -0.500001837">
                {_joint("wrist_roll")}
                <geom name="hand" type="capsule" size="0.020" fromto="0 0 0.010 0 0 0.094"/>
                {tool.mjcf() if tool else ""}
                <geom class="visual" type="box" size="0.02 0.016 0.034" pos="0 0 0.044"/>
                <geom class="servo" type="box" size="0.0124 0.0085 0.0226" pos="0 -0.024 0.044"/>
                <geom class="visual" type="box" size="0.016 0.003 0.022" pos="0 0.013 0.098"/>
                <site name="tcp" pos="0 0 {TCP_OFFSET_M}" size="0.008" rgba="0.1 0.85 0.35 1" group="4"/>
                <body name="jaw" gravcomp="1" pos="0 0.018821 0.052035" quat="0.499998163 -0.5 -0.5 -0.500001837">
                  {_joint("gripper")}
                  <geom name="jaw" type="box" size="0.030 0.021 0.012" pos="0.022 0 -0.010"/>
                  <geom class="visual" type="box" size="0.032 0.016 0.003" pos="0.026 0 -0.006"/>
                  <geom class="metal" type="cylinder" size="0.004 0.012" pos="0 0 -0.006"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <contact>
    <exclude body1="base_link" body2="link1"/>
  </contact>
  <actuator>
    {actuators}
  </actuator>
  <keyframe>
    <key name="home" qpos="{home}" ctrl="{home}"/>
  </keyframe>
</mujoco>
"""
