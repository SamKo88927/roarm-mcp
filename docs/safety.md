# Safety model

roarm-mcp assumes the agent can be wrong, adversarial or confused. Safety is enforced below the MCP surface, in
code the agent cannot reach.

## Layers

1. **Schema.** Tool arguments have types and broad ranges (finite numbers, gripper opening 0 to 1, exactly five
   joint values). This only rejects malformed input; it is not the safety check.
2. **Target check.** `move_to` targets must lie inside the reach ring (100 to 450 mm from the base axis) and the
   height band (18 to 450 mm above the table, see margins below).
3. **Inverse kinematics with an explicit convergence check.** The solver searches only inside the margined joint
   limits. A solution that misses the target by more than 1 mm or 0.02 rad is refused as `UNREACHABLE`. It is never
   passed on as a plan.
4. **Waypoint check.** The plan is a straight line in joint space, sampled so no joint moves more than 0.05 rad
   between waypoints. Every waypoint is checked for joint limits, self-collision, arm-table collision and the
   clearance of every moving link and of the tool centre point. These waypoints are exactly what the transport
   follows.
5. **Runtime monitor.** While the transport runs, the measured joints are checked after every waypoint. If the
   measured arm goes below the hard clearance limit or touches anything, the motion halts with
   `ENVELOPE_BREACH`.
6. **One motion at a time.** A second motion while one runs is refused with `BUSY`. `stop()` takes effect at the
   next waypoint, and the arm holds where it is.
7. **Faults are refusals.** If planning or the transport raises, the call returns `INTERNAL_ERROR` with the
   exception text, the fault is logged to stderr and recorded in the trace, and the motion lock is released.

## Default envelope

| Setting | Default | Meaning |
| --- | --- | --- |
| `min_clearance_mm` | 15 | Hard limit for what actually runs (runtime monitor) |
| `tracking_margin_mm` | 3 | Extra clearance plans must keep, for tracking error. Plans need 18 mm |
| `min_radius_mm` / `max_radius_mm` | 100 / 450 | Reach ring for the tool centre point |
| `max_height_mm` | 450 | Highest tool centre point |
| `joint_margin_rad` | 0.02 | Kept away from each mechanical joint limit |
| `max_joint_speed` | 1.5 rad/s | At `speed=1.0` |
| `max_waypoint_step` | 0.05 rad | Largest joint change between checked waypoints |
| `ik_tolerance_mm` / `ik_tolerance_rad` | 1.0 / 0.02 | Largest accepted IK miss |

The envelope is a frozen dataclass that validates itself when it is built (for example, it rejects negative
clearances and waypoint steps over 0.15 rad). It is created once at startup.

## What this does not cover

- **Objects on the table.** Clearance is checked against the table plane only.
- **Power loss or firmware faults.** A software stop cannot cut power. Use the physical switch.
- **Sim-to-real gap.** The simulated servos are stiff. Real servos sag under load and lag at speed. The real-arm
  milestone measures this on hardware and sets margins from those measurements.

## How it is tested

- Unit tests for every refusal code, the IK convergence check, the joint limit check and the suggestion search.
  Each suggested pose is replanned to confirm it passes.
- A fixture case where both ends of a motion are safe but the path between them dips below the clearance limit.
  It is refused at the waypoint where the dip happens.
- Property tests (Hypothesis) with random targets and joint sets, both inside and far outside the workspace. After
  every command, the clearance measured on the stepped simulation must be at least 15 mm with no contacts. A guard
  test makes sure at least 100 of those random commands really moved the arm.
- A slipping-servo transport, which shows that the runtime monitor halts a motion that leaves the envelope.
- A concurrency test, which shows that `stop()` interrupts a running motion through the real MCP client/server path.
