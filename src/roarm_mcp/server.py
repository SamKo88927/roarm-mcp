"""MCP surface. Every acting tool goes through Arm, which plans and checks before anything moves."""

from __future__ import annotations

import json
from typing import Annotated, Any, Literal

import anyio
from mcp.server.fastmcp import FastMCP
from mcp.types import CallToolResult, TextContent, ToolAnnotations
from pydantic import Field

from .arm import Arm

INSTRUCTIONS = """\
Controls a Waveshare RoArm-M3 desk arm (5 joints + gripper). Coordinates are millimetres in the table frame:
x forward, y left, z = height of the tool centre point above the table. Angles are degrees.
Call get_limits first and plan inside them. Every motion is checked before it runs; a refused motion returns
ok=false with a reason code, a message and, when one exists, a nearby pose that would pass. Use that to correct
yourself instead of retrying the same command. stop() halts the current motion; it is a software stop.
"""

Mm = Annotated[float, Field(allow_inf_nan=False, ge=-2000, le=2000)]
Deg = Annotated[float, Field(allow_inf_nan=False, ge=-360, le=360)]
Speed = Annotated[
    float, Field(allow_inf_nan=False, ge=0.05, le=1.0, description="Fraction of the maximum joint speed.")
]


def tool_result(payload: dict[str, Any]) -> CallToolResult:
    """Structured result; refusals set isError so clients surface them to the model."""
    return CallToolResult(
        content=[TextContent(type="text", text=json.dumps(payload, ensure_ascii=False))],
        structuredContent=payload,
        isError=payload.get("ok") is not True,
    )


LogLevel = Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]


def create_server(arm: Arm, host: str = "127.0.0.1", port: int = 8000, log_level: LogLevel = "WARNING") -> FastMCP:
    server = FastMCP("roarm-mcp", instructions=INSTRUCTIONS, host=host, port=port, log_level=log_level)
    read = ToolAnnotations(readOnlyHint=True, idempotentHint=True, openWorldHint=False)
    act = ToolAnnotations(readOnlyHint=False, destructiveHint=False, idempotentHint=False, openWorldHint=True)

    @server.tool(annotations=read)
    def get_state() -> CallToolResult:
        """Tool pose, joint angles, gripper opening, clearance above the table, and whether a motion is running."""
        return tool_result(arm.get_state())

    @server.tool(annotations=read)
    def get_limits() -> CallToolResult:
        """The safety envelope (reach, height, joint limits, speed). Fixed at startup; no tool can change it."""
        return tool_result(arm.get_limits())

    @server.tool(annotations=act)
    async def move_to(
        x_mm: Mm,
        y_mm: Mm,
        z_mm: Annotated[Mm, Field(description="Height of the tool centre point above the table.")],
        pitch_deg: Annotated[Deg, Field(description="0 = tool horizontal, 90 = pointing straight down.")] = 90.0,
        roll_deg: Deg = 0.0,
        speed: Speed = 0.5,
    ) -> CallToolResult:
        """Move the tool centre point to (x, y, z) with the given pitch and roll. Returns when the motion ends."""
        return tool_result(
            await anyio.to_thread.run_sync(lambda: arm.move_to(x_mm, y_mm, z_mm, pitch_deg, roll_deg, speed))
        )

    @server.tool(annotations=act)
    async def move_joints(
        joints_deg: Annotated[
            list[Deg],
            Field(min_length=5, max_length=5, description="base, shoulder, elbow, wrist_pitch, wrist_roll"),
        ],
        speed: Speed = 0.5,
    ) -> CallToolResult:
        """Move the five arm joints to the given angles in degrees. The gripper keeps its opening."""
        return tool_result(await anyio.to_thread.run_sync(lambda: arm.move_joints(joints_deg, speed)))

    @server.tool(annotations=act)
    async def gripper(
        opening: Annotated[
            float, Field(allow_inf_nan=False, ge=0.0, le=1.0, description="0 = closed, 1 = fully open.")
        ],
        speed: Speed = 0.5,
    ) -> CallToolResult:
        """Open or close the gripper."""
        return tool_result(await anyio.to_thread.run_sync(lambda: arm.gripper(opening, speed)))

    @server.tool(annotations=act)
    async def rest(speed: Speed = 0.5) -> CallToolResult:
        """Return to the rest pose (arm raised, gripper open)."""
        return tool_result(await anyio.to_thread.run_sync(lambda: arm.rest(speed)))

    @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False, openWorldHint=True))
    def stop() -> CallToolResult:
        """Halt the running motion at the next waypoint; the arm holds where it is. Software stop only."""
        return tool_result(arm.stop())

    @server.resource("roarm://limits", mime_type="application/json")
    def limits_resource() -> str:
        """The active safety envelope."""
        return json.dumps(arm.get_limits()["limits"])

    @server.resource("roarm://trace/latest", mime_type="application/json")
    def trace_resource() -> str:
        """The last 50 tool calls with their verdicts."""
        return json.dumps(arm.trace.latest(50), ensure_ascii=False)

    return server
