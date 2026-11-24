# Security

roarm-mcp lets a language model move a physical robot arm. A bug that lets a tool call bypass the safety envelope
is a security issue, even in simulation.

## Reporting

Use GitHub's private vulnerability reporting: **Security → Report a vulnerability** on this repository. Do not
open a public issue. Include the tool calls (a `--trace` file is ideal) and what you expected the safety layer to
do. You should get a first reply within a week.

## In scope

- A sequence of MCP calls that moves the arm outside the envelope in `docs/safety.md`, or that executes a motion
  without every waypoint being checked.
- A way to change the envelope through the MCP surface.
- The HTTP transport being reachable from another host without `--allow-remote`.

## Out of scope

- The HTTP endpoint has no authentication by design; keep it on loopback or put it behind your own
  authenticating proxy.
- `stop()` is a software stop. On real hardware, the physical power switch is the only emergency stop.
