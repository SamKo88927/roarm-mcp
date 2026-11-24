# Design

## Goal

An MCP server that lets an LLM agent operate a Waveshare RoArm-M3 safely. Every motion is checked in simulation
and by a safety layer the agent cannot bypass. The interface is built for models: table-frame millimetres,
degrees, structured results and refusals that explain how to recover.

## Layout

```
src/roarm_mcp/
  model.py    MJCF of the arm built from primitive shapes on the arm's kinematic tree
  sim.py      ArmModel (FK, IK, collision and clearance queries) and SimTransport (physics-simulated arm)
  safety.py   Envelope (frozen limits), Governor (planning + checking), Refusal
  arm.py      Arm: the only path from a tool call to a transport; motion lock, stop, runtime monitor
  trace.py    In-memory and JSONL trace of every tool call
  server.py   FastMCP tools and resources
  cli.py      `roarm-mcp` entry point
```

## Decisions

- **Table frame on the MCP surface.** z is the tool height above the table, not the firmware's shoulder-origin
  frame. Models reason about "40 mm above the table" more reliably. The transport layer converts frames.
- **Degrees on the MCP surface, radians inside.**
- **Joint-space waypoints.** The planner interpolates in joint space and checks every sample, because that is
  what the servos execute. Cartesian straight lines can be added later as a planner on top of the same check.
- **Refusal suggestions.** On refusal, the governor searches along the straight line from the current pose to the
  target (bisection, 12 steps) for the farthest pose that fully plans and checks. The suggestion is only returned
  if it passed the same checks.
- **Blocking motions in a worker thread.** Tool calls that move the arm run with `anyio.to_thread`. The event
  loop stays free, so `stop()` and `get_state()` answer during a motion.
- **No vendor code or assets.** The serial client will be written against Waveshare's public JSON command
  documentation. No meshes or CAD files are redistributed.

## Milestones

1. Simulation-only server, safety layer, tests, CI (this release).
2. Real arm over USB serial and Wi-Fi. Measured sim-to-real margins. Demo recorded from traces.
3. Camera tools (`look`, `locate`) for a multimodal model.
4. Pick and place skills and a reproducible benchmark across models.
