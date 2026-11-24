"""Command line entry point: `roarm-mcp [--trace PATH] [--http [--host H] [--port P]]`."""

from __future__ import annotations

import argparse
import ipaddress
import logging
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import cast

from . import __version__


def _is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="roarm-mcp",
        description="MCP server for the Waveshare RoArm-M3. Runs a MuJoCo-simulated arm.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--real", action="store_true", help="drive real hardware (not available in this release)")
    parser.add_argument("--trace", type=Path, help="append every tool call to this JSONL file")
    parser.add_argument("--http", action="store_true", help="serve streamable HTTP instead of stdio")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default 8000)")
    parser.add_argument(
        "--allow-remote",
        action="store_true",
        help="allow a non-loopback --host. The HTTP endpoint has no authentication; anyone who can reach it can "
        "move the arm",
    )
    parser.add_argument("--fast", action="store_true", help="run the simulation faster than real time")
    parser.add_argument(
        "--log-level",
        default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="log level; logs always go to stderr, never stdout (default WARNING)",
    )
    args = parser.parse_args(argv)

    if args.real:
        print(
            "roarm-mcp: real hardware is not supported in this release; the server runs in simulation only.",
            file=sys.stderr,
        )
        return 2
    if not 0 < args.port < 65536:
        parser.error("--port must be between 1 and 65535")
    if args.http and not _is_loopback(args.host) and not args.allow_remote:
        parser.error(
            f"--host {args.host} is not a loopback address. The HTTP endpoint has no authentication; "
            "pass --allow-remote if you really want to expose it"
        )

    # In stdio mode stdout carries the protocol, so every log line goes to stderr.
    logging.basicConfig(
        level=args.log_level, stream=sys.stderr, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )

    # Import late so `--help` and `--real` stay fast and never load MuJoCo.
    from .arm import Arm
    from .server import LogLevel, create_server
    from .sim import SimTransport
    from .trace import Trace

    arm = Arm(SimTransport(realtime=not args.fast), trace=Trace(args.trace))
    server = create_server(arm, host=args.host, port=args.port, log_level=cast(LogLevel, args.log_level))
    try:
        server.run(transport="streamable-http" if args.http else "stdio")
    except KeyboardInterrupt:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
