"""Replay a recorded episode in the simulator and measure how far reality and the model drifted apart.

    python -m roarm_mcp.replay EPISODE.json [...] [--tool-geometry FILE]

For each episode the simulator starts from the recorded start joints and follows the same planned waypoints at
the same period. The measured samples are then compared with the simulated trajectory at the same moments,
after shifting time by the lag that fits best (command and feedback latency on a real arm). Reported: that lag,
the joint error per joint (deg, RMS and max), and the tool position error (mm, RMS and max).

On a simulated episode the errors are near zero; that is the self-check. On a real one they are the sim-to-real
gap to fix (servo lag, sag under gravity, backlash), and the numbers to beat after each fix.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from .episodes import load
from .model import JOINT_NAMES, Tool
from .sim import ArmModel, SimTransport, mj

MAX_LAG_S = 1.0
LAG_STEP_S = 0.01


def simulate(episode: dict[str, Any], tool: Tool | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Times (s) and joints (n x 6) of the planned motion run in the simulator from the recorded start."""
    sim = SimTransport(realtime=False, tool=tool)
    start = np.asarray(episode["start_joints"], dtype=np.float64)
    sim.data.qpos[:] = start
    sim.data.qvel[:] = 0
    sim.data.ctrl[:] = start
    sim.data.time = 0.0
    mj.mj_forward(sim.model, sim.data)
    times, joints = [0.0], [start.copy()]

    def sample(q: np.ndarray) -> None:
        times.append(float(sim.data.time))
        joints.append(np.asarray(q, dtype=np.float64).copy())

    sim.follow([np.asarray(w) for w in episode["planned"]], float(episode["period_s"]), on_waypoint=sample)
    return np.asarray(times), np.asarray(joints)


def _interp(times: np.ndarray, joints: np.ndarray, at: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(at, times, joints[:, j]) for j in range(joints.shape[1])], axis=1)


def compare(episode: dict[str, Any], tool: Tool | None = None) -> dict[str, Any]:
    if not episode["planned"]:
        return {"tool": episode["tool"], "skipped": "refused before moving; nothing to replay"}
    sim_t, sim_q = simulate(episode, tool)
    measured = np.asarray(episode["measured"], dtype=np.float64)
    real_t, real_q = measured[:, 0] - measured[0, 0], measured[:, 1:]
    best: tuple[float, float] | None = None
    for lag in np.arange(0.0, MAX_LAG_S + 1e-9, LAG_STEP_S):
        err = float(np.sqrt(np.mean((real_q[:, :5] - _interp(sim_t, sim_q, real_t - lag)[:, :5]) ** 2)))
        if best is None or err < best[1]:
            best = (float(lag), err)
    assert best is not None
    lag = best[0]
    expected = _interp(sim_t, sim_q, real_t - lag)
    diff = np.degrees(real_q - expected)
    model = ArmModel(tool)
    tool_err = np.array(
        [
            np.linalg.norm(
                np.subtract(
                    [model.pose(a).x, model.pose(a).y, model.pose(a).z],
                    [model.pose(b).x, model.pose(b).y, model.pose(b).z],
                )
            )
            for a, b in zip(real_q, expected, strict=True)
        ]
    )
    return {
        "tool": episode["tool"],
        "backend": episode["backend"],
        "samples": len(real_t),
        "lag_s": round(lag, 3),
        "joint_error_deg": {
            name: {
                "rms": round(float(np.sqrt(np.mean(diff[:, i] ** 2))), 2),
                "max": round(float(np.max(np.abs(diff[:, i]))), 2),
            }
            for i, name in enumerate(JOINT_NAMES)
        },
        "tool_error_mm": {
            "rms": round(float(np.sqrt(np.mean(tool_err**2))), 2),
            "max": round(float(tool_err.max()), 2),
        },
        "final_tool_error_mm": round(float(tool_err[-1]), 2),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m roarm_mcp.replay", description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--tool-geometry", type=Path)
    args = parser.parse_args(argv)
    tool = Tool.load(args.tool_geometry) if args.tool_geometry else None
    for path in args.episodes:
        print(json.dumps({"episode": path.name, **compare(load(path), tool)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
