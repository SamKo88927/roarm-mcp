"""Collision boxes for a claw or camera fitted to the wrist."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from roarm_mcp.arm import Arm
from roarm_mcp.model import HOME, Tool
from roarm_mcp.sim import ArmModel, SimTransport

EXAMPLE = Path(__file__).parent / "data" / "example_tool.json"


def test_tool_boxes_join_the_safety_checks() -> None:
    tool = Tool.load(EXAMPLE)
    model = ArmModel(tool)
    names = {model.model.geom(g).name for g in model.moving_geoms}
    assert {"tool_claw_body", "tool_camera"} <= names
    assert model.contacts(HOME) == []


def test_a_bulky_tool_is_kept_off_the_table() -> None:
    # Tool tilted 30 degrees, 30 mm up: the stock hand clears the table, the bulkier fitted claw body does not.
    tool = Tool.load(EXAMPLE)
    stock = Arm(SimTransport(realtime=False)).move_to(300, 0, 30, pitch_deg=30, speed=1.0)
    fitted = Arm(SimTransport(realtime=False, tool=tool), tool=tool).move_to(300, 0, 30, pitch_deg=30, speed=1.0)
    assert stock["ok"], stock
    assert not fitted["ok"]
    assert fitted["refusal"]["code"] == "TABLE_CLEARANCE"


def test_limits_name_the_fitted_tool() -> None:
    tool = Tool.load(EXAMPLE)
    limits = Arm(SimTransport(realtime=False), tool=tool).get_limits()["limits"]
    assert limits["fitted_tool"] == "example claw with a camera on top"


@pytest.mark.parametrize(
    "box",
    [
        {"name": "a b", "center_mm": [0, 0, 0], "half_size_mm": [1, 1, 1]},
        {"name": "a", "center_mm": [0, 0], "half_size_mm": [1, 1, 1]},
        {"name": "a", "center_mm": [0, 0, 0], "half_size_mm": [1, 0, 1]},
        {"name": "a", "center_mm": [0, 0, "nan"], "half_size_mm": [1, 1, 1]},
    ],
)
def test_bad_tool_files_are_rejected(tmp_path: Path, box: dict[str, object]) -> None:
    path = tmp_path / "tool.json"
    path.write_text(json.dumps({"name": "bad", "boxes": [box]}))
    with pytest.raises(ValueError):
        Tool.load(path)
