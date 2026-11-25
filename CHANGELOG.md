# Changelog

## Unreleased

- `--live-port PORT`: a read-only Server-Sent Events stream of the measured joints and tool pose
  (`GET /state`, 20 Hz while moving, 2 Hz idle) on 127.0.0.1, for live 3D views.
- `--tool-geometry FILE`: collision boxes for a claw, camera or clip fitted to the wrist, in the wrist-roll link
  frame. They join every contact and table-clearance check; `get_limits` names the fitted tool.
- Real-arm transport over USB serial (`--real DEVICE`, extra `roarm-mcp[real]`): streams checked waypoints
  with T:102, polls measured joints with T:105, halts and holds on a tracking error, and locks the port so a
  second process cannot drive the same arm. `--dry-run` prints the commands instead. `--real` requires
  `--gripper-map` or `--stock-gripper`, defaults to 60 mm table clearance, streams at most 20 waypoints/s, and
  closes the gripper in 0.03 rad steps, stopping and backing off 0.03 rad at the first sign of contact
  (`executed.gripper_contact`). Tested against a fake firmware with 90 ms
  feedback latency only; not yet run on hardware.
- `serverInfo.version` reports the roarm-mcp version instead of the MCP SDK's.
- Dockerfile (simulated arm over stdio) and `glama.json`, so MCP directories can start and inspect the server.

## 0.1.1 (2026-11-24)

- Install from PyPI: `uvx roarm-mcp`. README badges.
- MCP Registry name `io.github.SamKo88927/roarm-mcp` (`server.json`).

## 0.1.0 (2026-11-24)

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
- Server faults (planning or transport exceptions) return an `INTERNAL_ERROR` refusal, are logged to stderr and
  traced, and always release the motion lock.
- CLI: `--host`, `--port`, `--log-level`; a non-loopback HTTP host needs `--allow-remote`, because the endpoint
  has no authentication. Logs never go to stdout.
- Install with `uvx --from git+https://github.com/SamKo88927/roarm-mcp roarm-mcp`; checked from a clean
  environment with a real stdio MCP client.
- CI: ruff, pyright strict, pytest with a 90% coverage floor on Python 3.11 to 3.13 (Linux) and 3.13 (macOS),
  package build. Dependabot for Actions and Python dependencies.
