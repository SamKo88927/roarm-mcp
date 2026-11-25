from __future__ import annotations

import json

import anyio
import pytest
from mcp.shared.memory import create_connected_server_and_client_session
from mcp.types import TextContent

from roarm_mcp.arm import Arm
from roarm_mcp.cli import main
from roarm_mcp.server import create_server
from roarm_mcp.sim import SimTransport

EXPECTED_TOOLS = {"get_state", "get_limits", "move_to", "move_joints", "gripper", "rest", "stop"}


async def test_tool_surface() -> None:
    server = create_server(Arm(SimTransport(realtime=False)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        tools = (await client.list_tools()).tools
        assert {t.name for t in tools} == EXPECTED_TOOLS  # nothing can change the envelope
        for t in tools:
            assert t.description


async def test_refusal_is_an_error_with_a_reason() -> None:
    server = create_server(Arm(SimTransport(realtime=False)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        ok = await client.call_tool("move_to", {"x_mm": 250, "y_mm": 0, "z_mm": 60})
        assert ok.isError is False
        bad = await client.call_tool("move_to", {"x_mm": 250, "y_mm": 0, "z_mm": 2})
        assert bad.isError is True
        assert bad.structuredContent is not None
        assert bad.structuredContent["refusal"]["code"] == "TABLE_CLEARANCE"
        assert "suggestion" in bad.structuredContent["refusal"]


async def test_schema_rejects_out_of_range_arguments() -> None:
    server = create_server(Arm(SimTransport(realtime=False)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        r = await client.call_tool("gripper", {"opening": 3})
        assert r.isError is True
        r = await client.call_tool("move_joints", {"joints_deg": [0, 0, 0]})
        assert r.isError is True


async def test_stop_arrives_during_a_motion() -> None:
    server = create_server(Arm(SimTransport(realtime=True)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        results: dict[str, object] = {}

        async def slow_move() -> None:
            results["move"] = await client.call_tool("move_to", {"x_mm": -200, "y_mm": 100, "z_mm": 60, "speed": 0.1})

        async with anyio.create_task_group() as tg:
            tg.start_soon(slow_move)
            await anyio.sleep(0.4)
            stop = await client.call_tool("stop", {})
            assert stop.structuredContent is not None
            assert stop.structuredContent["was_moving"] is True
        move = results["move"]
        content = move.content[0]  # type: ignore[attr-defined]
        assert isinstance(content, TextContent)
        assert json.loads(content.text)["stopped"] is True


async def test_resources() -> None:
    server = create_server(Arm(SimTransport(realtime=False)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        uris = {str(r.uri) for r in (await client.list_resources()).resources}
        assert uris == {"roarm://limits", "roarm://trace/latest"}


def test_real_flag_refuses_cleanly(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--real"]) == 2
    assert "simulation only" in capsys.readouterr().err


def test_http_refuses_non_loopback_without_opt_in(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as e:
        main(["--http", "--host", "0.0.0.0"])
    assert e.value.code == 2
    assert "no authentication" in capsys.readouterr().err


def test_bad_port_is_rejected() -> None:
    with pytest.raises(SystemExit):
        main(["--port", "70000"])


async def test_server_reports_package_version() -> None:
    from roarm_mcp import __version__

    server = create_server(Arm(SimTransport(realtime=False)))
    async with create_connected_server_and_client_session(server._mcp_server) as client:  # pyright: ignore[reportPrivateUsage]
        result = await client.initialize()
        assert result.serverInfo.version == __version__
