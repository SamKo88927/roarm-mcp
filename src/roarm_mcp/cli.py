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
        description="MCP server for the Waveshare RoArm-M3. Runs a MuJoCo-simulated arm unless --real is given.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument(
        "--real",
        metavar="DEVICE",
        help="drive the real arm on this serial device, e.g. /dev/cu.usbserial-XXXX (needs roarm-mcp[real])",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="with --real: talk to a fake arm instead and log every command that would be sent",
    )
    parser.add_argument(
        "--gripper-map",
        type=Path,
        help='with --real: JSON {"raw_closed": ..., "raw_open": ...} measured on your claw, in firmware radians',
    )
    parser.add_argument(
        "--stock-gripper",
        action="store_true",
        help="with --real: the arm has the stock claw (1.57 rad open, 3.13 rad closed)",
    )
    parser.add_argument(
        "--live-port",
        type=int,
        help="also stream the arm's measured joints as Server-Sent Events on http://127.0.0.1:PORT/state, for a 3D "
        "view (read only)",
    )
    parser.add_argument(
        "--record",
        type=Path,
        metavar="DIR",
        help="save every motion as an episode JSON (plan, measured joints, result) in DIR; replay them in the "
        "simulator with `python -m roarm_mcp.replay`",
    )
    parser.add_argument(
        "--tool-geometry",
        type=Path,
        help='JSON {"name": ..., "boxes": [{"name", "center_mm", "half_size_mm"}]}: collision boxes for a claw or '
        "camera fitted to the wrist, in the wrist-roll link frame",
    )
    parser.add_argument(
        "--min-clearance-mm",
        type=float,
        help="smallest allowed distance between the arm and the table (default 15 in simulation, 60 with --real: "
        "the model does not know about tools or cameras you fitted)",
    )
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

    if (args.dry_run or args.gripper_map or args.stock_gripper) and not args.real:
        parser.error("--dry-run, --gripper-map and --stock-gripper need --real")
    if args.real and not (args.gripper_map or args.stock_gripper):
        parser.error(
            "--real needs --gripper-map FILE (measured on your claw) or --stock-gripper: claws differ in range and "
            "direction, and a wrong map drives the jaws into their stops"
        )
    if args.gripper_map and args.stock_gripper:
        parser.error("give --gripper-map or --stock-gripper, not both")
    if args.live_port is not None and not 0 < args.live_port < 65536:
        parser.error("--live-port must be between 1 and 65535")
    if args.min_clearance_mm is not None and not 0 <= args.min_clearance_mm <= 200:
        parser.error("--min-clearance-mm must be between 0 and 200")
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
    from .arm import Arm, Transport
    from .model import Tool
    from .safety import Envelope
    from .server import LogLevel, create_server
    from .sim import SimTransport
    from .trace import Trace

    try:
        tool = Tool.load(args.tool_geometry) if args.tool_geometry else None
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"roarm-mcp: --tool-geometry: {exc}", file=sys.stderr)
        return 2
    transport: Transport
    if args.real:
        from .hardware import STOCK_GRIPPER, FakeFirmware, GripperMap, SerialTransport, TransportError, open_serial
        from .model import HOME

        try:
            gripper = GripperMap.load(args.gripper_map) if args.gripper_map else STOCK_GRIPPER
            if args.dry_run:
                port = FakeFirmware([*HOME[:5], gripper.to_raw(HOME[5])], log=sys.stderr)
            else:
                port = open_serial(args.real)
            transport = SerialTransport(port, gripper)
            transport.read_joints()  # fail now, not on the first tool call, if the arm does not answer
        except (TransportError, OSError, ValueError, KeyError) as exc:
            print(f"roarm-mcp: {exc}", file=sys.stderr)
            return 2
    else:
        transport = SimTransport(realtime=not args.fast, tool=tool)
    clearance = args.min_clearance_mm if args.min_clearance_mm is not None else (60.0 if args.real else 15.0)
    from .episodes import EpisodeRecorder

    recorder = EpisodeRecorder(args.record) if args.record else None
    arm = Arm(
        transport,
        envelope=Envelope(min_clearance_mm=clearance),
        trace=Trace(args.trace),
        tool=tool,
        recorder=recorder,
    )
    live = None
    if args.live_port is not None:
        from .live import LiveState

        try:
            live = LiveState(arm, args.live_port).start()
        except OSError as exc:
            print(f"roarm-mcp: --live-port {args.live_port}: {exc}", file=sys.stderr)
            return 2
    server = create_server(arm, host=args.host, port=args.port, log_level=cast(LogLevel, args.log_level))
    try:
        server.run(transport="streamable-http" if args.http else "stdio")
    except KeyboardInterrupt:
        arm.stop()
    finally:
        if live is not None:
            live.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
