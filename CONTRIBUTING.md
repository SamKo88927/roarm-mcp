# Contributing

Issues and pull requests are welcome.

## Setup

```sh
uv sync
uv run ruff check . && uv run ruff format --check .
uv run pyright
uv run pytest -q --cov=roarm_mcp
```

CI runs the same commands on Linux and macOS and fails below 90% coverage.

## Rules for changes

- **Safety first.** Every path that moves the arm must go through `Arm`, which plans, checks every waypoint and
  monitors what ran. Do not add a tool that changes the envelope at runtime.
- **Tests with the change.** A new refusal code needs a test that triggers it. A safety fix needs a test that
  fails without the fix.
- **Models are the users.** Keep tool results structured; a refusal says what was wrong and, where possible, what
  would pass.
- **No vendor assets.** Do not commit Waveshare meshes, CAD files or firmware.
- Update `CHANGELOG.md`.
