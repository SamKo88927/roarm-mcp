"""Primitive MuJoCo model of the RoArm-M3, generated from kinematic numbers.

No vendor meshes are shipped. The kinematic tree (link offsets, joint axes, joint ranges) matches the arm, and every
link is approximated by a capsule, cylinder or box that doubles as its collision shape. The link offsets are taken
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

from dataclasses import dataclass

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

ARM_RGBA = "0.58 0.62 0.68 1"
SERVO_RGBA = "0.10 0.11 0.13 1"


def _joint(name: str) -> str:
    spec = next(j for j in JOINTS if j.name == name)
    return f'<joint name="{name}" type="hinge" axis="0 0 1" range="{spec.lower} {spec.upper}"/>'


def build_mjcf() -> str:
    """Return the MJCF XML for the arm on a table."""
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
    <headlight ambient="0.55 0.55 0.55" diffuse="0.6 0.6 0.6"/>
  </visual>
  <default>
    <joint damping="0.18" armature="0.004" frictionloss="0.025"/>
    <geom contype="1" conaffinity="1" density="1200" friction="0.7 0.01 0.001" rgba="{ARM_RGBA}"/>
    <position ctrllimited="true" forcelimited="true" inheritrange="1"/>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.29 0.34" rgb2="0.15 0.18 0.22"
             width="512" height="512"/>
    <material name="table" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0 -1.2 1.8" dir="0 0.5 -1" directional="true"/>
    <geom name="table" type="plane" size="1 1 0.02" material="table"/>
    <camera name="overview" pos="0.62 -0.62 0.48" xyaxes="0.707 0.707 0 -0.32 0.32 0.892"/>
    <body name="base_link" pos="0 0 0.0701">
      <geom name="base" type="box" size="0.072 0.072 0.034" pos="0 0 -0.036" rgba="{SERVO_RGBA}"/>
      <body name="link1" gravcomp="1">
        {_joint("base")}
        <geom name="turret" type="cylinder" size="0.046 0.027" pos="0 0 0.020"/>
        <body name="link2" gravcomp="1" pos="0 0 0.051959" quat="0.499998163 -0.5 -0.5 -0.500001837">
          {_joint("shoulder")}
          <geom name="upper_arm" type="capsule" size="0.026" fromto="0.015 0.008 0 0.220 0.028 0"/>
          <body name="link3" gravcomp="1" pos="0.236815 0.030002 0" quat="0.707105483 0 0 0.70710808">
            {_joint("elbow")}
            <geom name="forearm" type="capsule" size="0.023" fromto="0 -0.008 0 0 -0.132 0"/>
            <body name="link4" gravcomp="1" pos="0 -0.144586 0">
              {_joint("wrist_pitch")}
              <geom name="wrist" type="capsule" size="0.021" fromto="0 -0.005 0 0.012 -0.047 0"
                    rgba="{SERVO_RGBA}"/>
              <body name="link5" gravcomp="1" pos="0.015147 -0.053653 0" quat="0.499998163 0.5 0.5 -0.500001837">
                {_joint("wrist_roll")}
                <geom name="hand" type="capsule" size="0.020" fromto="0 0 0.010 0 0 0.094"/>
                <site name="tcp" pos="0 0 {TCP_OFFSET_M}" size="0.008" rgba="0.1 0.85 0.35 1"/>
                <body name="jaw" gravcomp="1" pos="0 0.018821 0.052035" quat="0.499998163 -0.5 -0.5 -0.500001837">
                  {_joint("gripper")}
                  <geom name="jaw" type="box" size="0.030 0.021 0.012" pos="0.022 0 -0.010"
                        rgba="{SERVO_RGBA}"/>
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
