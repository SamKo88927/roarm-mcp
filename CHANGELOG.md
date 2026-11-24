# Changelog

## 0.1.0 (unreleased)

- MCP server with `get_state`, `get_limits`, `move_to`, `move_joints`, `gripper`, `rest`, `stop`, and the
  resources `roarm://limits` and `roarm://trace/latest`.
- MuJoCo-simulated arm built from primitive shapes. No vendor meshes.
- Safety layer: frozen envelope, IK convergence check, per-waypoint checks, runtime monitor, refusal codes with
  checked-safe suggestions.
- Tests: unit, property-based (Hypothesis), MCP client/server, concurrency (stop during motion).
- CI: ruff, pyright strict, pytest with coverage on Python 3.11 to 3.13.
