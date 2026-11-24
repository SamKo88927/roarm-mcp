"""Command line entry point: `roarm-mcp [--trace PATH] [--http]`."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

from . import __version__


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="roarm-mcp",
        description="MCP server for the Waveshare RoArm-M3. Runs a MuJoCo-simulated arm.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--real", action="store_true", help="drive real hardware (not available in this release)")
    parser.add_argument("--trace", type=Path, help="append every tool call to this JSONL file")
    parser.add_argument("--http", action="store_true", help="serve streamable HTTP on 127.0.0.1:8000 instead of stdio")
    parser.add_argument("--fast", action="store_true", help="run the simulation faster than real time")
    args = parser.parse_args(argv)

    if args.real:
        print(
            "roarm-mcp: real hardware is not supported in this release; the server runs in simulation only.",
            file=sys.stderr,
        )
        return 2

    # Import late so `--help` and `--real` stay fast and never load MuJoCo.
    from .arm import Arm
    from .server import create_server
    from .sim import SimTransport
    from .trace import Trace

    arm = Arm(SimTransport(realtime=not args.fast), trace=Trace(args.trace))
    server = create_server(arm)
    server.run(transport="streamable-http" if args.http else "stdio")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
