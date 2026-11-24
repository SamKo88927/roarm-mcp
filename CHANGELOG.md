# Changelog

## 0.1.0 (unreleased)

- MCP server with `get_state`, `get_limits`, `move_to`, `move_joints`, `gripper`, `rest`, `stop`, and the
  resources `roarm://limits` and `roarm://trace/latest`.
- MuJoCo-simulated arm built from primitive shapes. No vendor meshes.
- Safety layer: frozen envelope, IK convergence check, per-waypoint checks, runtime monitor, refusal codes with
  checked-safe suggestions.
- Tests: unit, property-based (Hypothesis), MCP client/server, concurrency (stop during motion).
- Refusal suggestions are re-checked as the exact rounded pose a model will send back, so following one is
  always accepted (regression test included).
- Visual-only shell in the arm's colours (white brackets, black servos, forearm tube), shadows and a neutral
  desk. Collision shapes are unchanged and hidden; safety checks use collision geoms only.
- README demo GIF, generated from the real code path by `scripts/record_demo.py`. `--meshes DIR` draws the arm
  with local per-link STL meshes (not shipped); the README GIF uses meshes from Waveshare's official STEP.
- CI: ruff, pyright strict, pytest with coverage on Python 3.11 to 3.13.
