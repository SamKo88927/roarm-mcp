"""Record the README demo GIF from the simulated arm.

Every frame comes from the real code path: the same Arm, governor and SimTransport the MCP server uses.
The captions show the tool call and the result it returned.

    uv run --extra render python scripts/record_demo.py docs/media/demo.gif [--meshes DIR]

With --meshes, the arm is drawn with per-link STL meshes instead of the built-in box-and-cylinder shell.
The README GIF was rendered this way from meshes split out of Waveshare's official RoArm-M3 STEP file.
The meshes are not part of this repository. Name them <body>_<part>_<nn>.stl, where body is one of
base_link, link1, link2, link3, link4, link5, gripper_link and part is structure, servo, hardware, detail or
electronics; each mesh must be in its body's frame. Only the drawing changes: kinematics, collision shapes and
every safety check are the same as without --meshes.
"""

from __future__ import annotations

import argparse
import re
import sys
import textwrap
from collections.abc import Callable
from pathlib import Path
from typing import Any

import imageio.v3 as iio
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from roarm_mcp.arm import Arm
from roarm_mcp.model import build_mjcf
from roarm_mcp.sim import SimTransport

WIDTH, HEIGHT = 640, 480
FPS = 20
GREEN = (0.15, 0.85, 0.35, 0.55)
RED = (0.95, 0.2, 0.2, 0.55)
Result = dict[str, Any]


# STL body prefix -> body name in the MJCF, and STL part -> material.
MESH_BODIES = {"gripper_link": "jaw"}
MESH_MATERIALS = {"structure": "white", "servo": "servo", "hardware": "servo", "detail": "tube", "electronics": "pcb"}
# The base and turret meshes sit 7 mm low against the joint axes in the source model; measured against the STEP.
MESH_OFFSETS = {"base_link": "0 0 0.007", "link1": "0 0 0.007"}
# Shell parts kept with meshes: the source meshes have no turret U-bracket, so the shell's bracket stands in.
KEEP_SHELL = re.compile(r'pos="0 (-?0\.031 0\.028|0 0\.0055|0 0\.002)"')


def mesh_mjcf(mesh_dir: Path) -> str:
    """The arm's MJCF with the visual shell replaced by STL meshes. Collision geoms are untouched."""
    xml = build_mjcf()
    lines = [
        line
        for line in xml.splitlines()
        if not ("<geom class=" in line and not KEEP_SHELL.search(line)) or "<default" in line
    ]
    xml = "\n".join(lines).replace("<compiler ", f'<compiler meshdir="{mesh_dir.resolve()}" ', 1)
    assets: list[str] = []
    per_body: dict[str, list[str]] = {}
    for stl in sorted(mesh_dir.glob("*.stl")):
        match = re.fullmatch(r"(.+)_(structure|servo|hardware|detail|electronics)_\d+", stl.stem)
        if match is None:
            continue
        source, part = match.groups()
        assets.append(f'<mesh name="m_{stl.stem}" file="{stl.name}"/>')
        offset = MESH_OFFSETS.get(source, "0 0 0")
        per_body.setdefault(MESH_BODIES.get(source, source), []).append(
            f'<geom class="visual" type="mesh" mesh="m_{stl.stem}" material="{MESH_MATERIALS[part]}" pos="{offset}"/>'
        )
    if not assets:
        raise SystemExit(f"no <body>_<part>_<nn>.stl meshes in {mesh_dir}")
    xml = xml.replace("</asset>", "  " + "\n    ".join(assets) + "\n  </asset>", 1)
    for body, geoms in per_body.items():
        inserted = "".join(geoms).replace("\\", "\\\\")
        xml, count = re.subn(rf'(<body name="{body}"[^>]*>)', rf"\g<1>{inserted}", xml, count=1)
        if count != 1:
            raise SystemExit(f"meshes for unknown body {body!r}")
    return xml


class RecordingSim(SimTransport):
    """SimTransport that renders a frame every 1/FPS seconds of simulated time."""

    def __init__(self, mesh_dir: Path | None = None) -> None:
        super().__init__(realtime=False)
        if mesh_dir is not None:
            self.model = mj.MjModel.from_xml_string(mesh_mjcf(mesh_dir))
            self.data = mj.MjData(self.model)
            mj.mj_resetDataKeyframe(self.model, self.data, 0)
            mj.mj_forward(self.model, self.data)
        self.renderer = mj.Renderer(self.model, HEIGHT, WIDTH)
        self.camera = mj.MjvCamera()
        self.camera.lookat[:] = (0.25, 0.0, 0.14)
        self.camera.distance = 0.9
        self.camera.azimuth = 120
        self.camera.elevation = -20
        self.every = round(1 / FPS / self.model.opt.timestep)
        self.since = 0
        self.frames: list[np.ndarray] = []
        self.caption: tuple[str, str, tuple[int, int, int]] = ("", "", (255, 255, 255))
        self.marker: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None

    def _step(self, n: int) -> None:
        for _ in range(n):
            mj.mj_step(self.model, self.data)
            self.since += 1
            if self.since >= self.every:
                self.since = 0
                self.snap()

    def snap(self, count: int = 1) -> None:
        self.renderer.update_scene(self.data, self.camera)
        if self.marker is not None:
            scene = self.renderer.scene
            if scene.ngeom < scene.maxgeom:
                pos, rgba = self.marker
                mj.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    mj.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.012, 0, 0]),
                    np.array(pos),
                    np.eye(3).flatten(),
                    np.array(rgba, dtype=np.float32),
                )
                scene.ngeom += 1
        frame = annotate(self.renderer.render(), *self.caption)
        self.frames.extend([frame] * count)

    def hold(self, seconds: float) -> None:
        self.snap(max(1, round(seconds * FPS)))


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in ("/System/Library/Fonts/Menlo.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size)


FONT, SMALL = _font(17), _font(14)


def annotate(rgb: np.ndarray, call: str, result: str, colour: tuple[int, int, int]) -> np.ndarray:
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image, "RGBA")
    if call:
        lines = textwrap.wrap(result, 72) or [""]
        draw.rectangle((0, 0, WIDTH, 40 + 19 * len(lines)), fill=(10, 12, 16, 200))
        draw.text((14, 8), call, font=FONT, fill=(235, 238, 242))
        for i, line in enumerate(lines):
            draw.text((14, 34 + 19 * i), line, font=SMALL, fill=colour)
    draw.text((WIDTH - 214, HEIGHT - 24), "roarm-mcp · MuJoCo sim", font=SMALL, fill=(220, 224, 230, 190))
    return np.asarray(image)


def target(x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
    # Targets below the table are drawn on the table surface so the marker stays visible.
    return (x_mm / 1000, y_mm / 1000, max(z_mm, 12.0) / 1000)


def step(sim: RecordingSim, call: str, run: Callable[[], Result], where: tuple[float, float, float] | None) -> Result:
    sim.caption = (call, "checking every waypoint, then moving ...", (200, 205, 212))
    sim.marker = (where, GREEN) if where else None
    sim.hold(0.6)
    result = run()
    if result["ok"]:
        sim.caption = (call, "ok", (110, 230, 140))
        sim.hold(0.7)
    else:
        refusal = result["refusal"]
        sim.marker = (where, RED) if where else None
        sim.caption = (call, f"refused {refusal['code']}: {refusal['message']}", (255, 120, 110))
        sim.hold(2.4)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--meshes", type=Path, help="directory of per-link STL meshes (see the module docstring)")
    args = parser.parse_args(argv)

    sim = RecordingSim(args.meshes)
    arm = Arm(sim)
    sim.hold(0.5)

    step(sim, "move_to(x=220, y=80, z=40)", lambda: arm.move_to(220, 80, 40, speed=0.6), target(220, 80, 40))
    step(sim, "gripper(opening=0.2)", lambda: arm.gripper(0.2, speed=0.8), None)
    step(sim, "move_to(x=260, y=-70, z=25)", lambda: arm.move_to(260, -70, 25, speed=0.6), target(260, -70, 25))
    step(
        sim, "move_to(x=200, y=90, z=-30)   # below the table", lambda: arm.move_to(200, 90, -30), target(200, 90, -30)
    )
    far = step(sim, "move_to(x=600, y=0, z=50)   # out of reach", lambda: arm.move_to(600, 0, 50), target(600, 0, 50))
    s = far["refusal"]["suggestion"]
    step(
        sim,
        f"move_to(x={s['x_mm']:.0f}, y={s['y_mm']:.0f}, z={s['z_mm']:.0f})   # the suggested safe pose",
        lambda: arm.move_to(s["x_mm"], s["y_mm"], s["z_mm"], speed=0.6),
        target(s["x_mm"], s["y_mm"], s["z_mm"]),
    )
    step(sim, "gripper(opening=1.0)", lambda: arm.gripper(1.0, speed=0.8), None)
    step(sim, "rest()", lambda: arm.rest(speed=0.6), None)
    sim.caption = ("", "", (255, 255, 255))
    sim.hold(0.6)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(args.output, np.stack(sim.frames), duration=1000 / FPS, loop=0)
    size = args.output.stat().st_size / 1e6
    print(f"{args.output}: {len(sim.frames)} frames, {len(sim.frames) / FPS:.1f} s, {size:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
