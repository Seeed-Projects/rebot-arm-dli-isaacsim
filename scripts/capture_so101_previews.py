#!/usr/bin/env python3
"""Capture headless still frames from the SO-101 vials scene.

Renders the wrist front, side, and external cameras plus one extra
overview frame that shows the whole white SO-ARM101 inside the enclosure.
Useful for documentation and for checking the scene without the GUI; the
scene is opened read-only and is never saved back.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "media" / "so101_camera_previews"
DEFAULT_SCENE = PROJECT_ROOT / "scenes" / "so101_vials_sorting.usda"
DEFAULT_DR_CONFIG = PROJECT_ROOT / "config" / "domain_randomization_so101.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Directory that receives the PNG stills",
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--dr-config", type=Path, default=DEFAULT_DR_CONFIG)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-dr", action="store_true")
    parser.add_argument("--settle-steps", type=int, default=120)
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=40,
        help="Rendered frames before the capture; with --pt-samples these are 128+ sample frames",
    )
    parser.add_argument(
        "--pt-samples",
        type=int,
        default=128,
        help="Path-traced samples per pixel per frame on each render product; 1 (the default) reads as snow when the accumulation resets",
    )
    parser.add_argument(
        "--realtime-renderer",
        action="store_true",
        help="Render with Real-Time instead of Path Tracing",
    )
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="Regenerate the scene before capturing",
    )
    parser.add_argument(
        "--overview",
        action="store_true",
        default=True,
        help="Also render one whole-workspace overview frame (default on)",
    )
    return parser.parse_args()


ARGS = _parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import (  # noqa: E402
    get_current_stage,
    is_stage_loading,
    open_stage,
)
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402

from build_scene import _look_at_camera  # noqa: E402
from build_scene_so101 import (  # noqa: E402
    build_scene_so101,
    so101_scene_is_configured,
)
from domain_randomization import DomainRandomizer  # noqa: E402


def _boost_path_tracing_samples(camera, samples: int) -> None:
    """Force Path Tracing on a sensor render product and raise its spp.

    New render products inherit the stage's render settings; without the
    /Render settings stack (which the reBot scene carries from its GUI era)
    they fall back to Real-Time, which on GPUs without DLSS-RR renders as
    one-sample speckle — the "snow".  Authoring the mode and the sample
    count directly on the product makes each frame self-contained and
    noise-free regardless of accumulation resets.
    """
    from pxr import Sdf, Usd

    stage = get_current_stage()
    product_path = camera.get_render_product_path()
    prim = stage.GetPrimAtPath(product_path)
    if not prim.IsValid():
        raise RuntimeError(f"Render product is missing: {product_path}")
    prim.CreateAttribute(
        "omni:rtx:rendermode", Sdf.ValueTypeNames.Token
    ).Set("PathTracing")
    prim.CreateAttribute(
        "omni:rtx:pt:samplesPerPixel", Sdf.ValueTypeNames.UInt
    ).Set(int(samples))
    print(
        f"[capture] {camera.name}: PathTracing, "
        f"{samples} samples/pixel/frame"
    )

if not ARGS.realtime_renderer:
    # Stills converge in a handful of frames and Real-Time 2.0 speckles on
    # GPUs without DLSS-RR; Path Tracing gives clean stills.  This is a
    # per-run setting and is not written into the scene file.
    carb.settings.get_settings().set_string("/rtx/rendermode", "PathTracing")


def _wait_for_stage() -> None:
    while is_stage_loading():
        simulation_app.update()
    for _ in range(3):
        simulation_app.update()


def _frame_to_uint8(frame) -> np.ndarray | None:
    array = np.asarray(frame)
    if array.size == 0:
        return None
    if array.dtype != np.uint8:
        array = np.asarray(array, dtype=np.float64)
        if float(np.nanmax(array)) <= 1.0 + 1e-6:
            array = array * 255.0
        array = np.clip(array, 0.0, 255.0).astype(np.uint8)
    return array[:, :, :3]


def main() -> int:
    dr_config = json.loads(ARGS.dr_config.read_text(encoding="utf-8"))
    if ARGS.no_dr:
        dr_config["enabled"] = False
    if ARGS.seed is not None:
        dr_config["seed"] = int(ARGS.seed)

    if ARGS.rebuild or not ARGS.scene.exists():
        build_scene_so101(
            ARGS.scene,
            render_mode="PathTracing" if not ARGS.realtime_renderer else None,
        )
    if not open_stage(str(ARGS.scene)):
        raise RuntimeError(f"Unable to open scene: {ARGS.scene}")
    _wait_for_stage()
    stage = get_current_stage()
    if not so101_scene_is_configured(stage):
        print("[scene] The SO-101 scene is outdated; rebuilding it")
        build_scene_so101(
            ARGS.scene,
            render_mode="PathTracing" if not ARGS.realtime_renderer else None,
        )
        if not open_stage(str(ARGS.scene)):
            raise RuntimeError(f"Unable to reopen scene: {ARGS.scene}")
        _wait_for_stage()

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 120.0,
    )
    robot = world.scene.add(
        SingleArticulation(prim_path="/World/Robot/base", name="so101")
    )
    targets = {
        name: SingleRigidPrim(prim_path=path, name=f"target_{name}")
        for name, path in (
            ("vial_1", "/World/Objects/Vial1"),
            ("vial_2", "/World/Objects/Vial2"),
            ("vial_3", "/World/Objects/Vial3"),
        )
    }
    world.reset()
    robot.initialize()
    for target in targets.values():
        target.initialize()

    side_camera = Camera(
        prim_path=dr_config["side_camera"]["prim_path"],
        name="side_camera",
        resolution=(640, 480),
    )
    front_camera = Camera(
        prim_path=dr_config["front_camera"]["prim_path"],
        name="front_camera",
        resolution=(640, 480),
    )
    external_camera = Camera(
        prim_path=dr_config["external_camera"]["prim_path"],
        name="external_camera",
        resolution=(640, 480),
    )
    side_camera.initialize()
    front_camera.initialize()
    external_camera.initialize()
    for camera in (side_camera, front_camera, external_camera):
        _boost_path_tracing_samples(camera, ARGS.pt_samples)

    if ARGS.overview:
        _look_at_camera(
            stage,
            "/World/Cameras/PreviewOverview",
            eye=Gf.Vec3d(0.85, -0.72, 0.72),
            target=Gf.Vec3d(-0.05, 0.16, 0.10),
            up=Gf.Vec3d(0.0, 0.0, 1.0),
            focal_length=24.0,
            clipping_range=Gf.Vec2f(0.01, 100.0),
        )
        overview_camera = Camera(
            prim_path="/World/Cameras/PreviewOverview",
            name="overview_camera",
            resolution=(640, 480),
        )
        overview_camera.initialize()
        _boost_path_tracing_samples(overview_camera, ARGS.pt_samples)
    else:
        overview_camera = None

    # Operator's over-the-shoulder view (owner-specified framing): 2 m back
    # (the robot faces -Y), 30 cm above the work surface, 30 cm to the right
    # (world -X = the robot's right hand side), looking into the enclosure.
    _look_at_camera(
        stage,
        "/World/Cameras/PreviewBack",
        eye=Gf.Vec3d(-0.30, 2.0, 0.30),
        target=Gf.Vec3d(0.0, 0.08, 0.08),
        up=Gf.Vec3d(0.0, 0.0, 1.0),
        focal_length=18.0,
        clipping_range=Gf.Vec2f(0.01, 100.0),
    )
    back_camera = Camera(
        prim_path="/World/Cameras/PreviewBack",
        name="back_camera",
        resolution=(640, 480),
    )
    back_camera.initialize()
    _boost_path_tracing_samples(back_camera, ARGS.pt_samples)

    randomizer = DomainRandomizer(
        stage=stage,
        targets=targets,
        side_camera=side_camera,
        front_camera=front_camera,
        config=dr_config,
        enabled=bool(dr_config["enabled"]) and not ARGS.no_dr,
        seed=int(dr_config["seed"]),
    )
    randomizer.reset()

    for _ in range(ARGS.settle_steps):
        world.step(render=False)

    ARGS.output.mkdir(parents=True, exist_ok=True)
    cameras = (
        ("front", front_camera),
        ("side", side_camera),
        ("external", external_camera),
        ("back", back_camera),
    )
    if overview_camera is not None:
        cameras = cameras + (("overview", overview_camera),)

    # Rendered warmup: every frame already carries --pt-samples samples per
    # pixel on every render product, so the captured buffer is clean even
    # though physics keeps stepping (which resets path-tracing accumulation).
    for _ in range(ARGS.warmup_frames):
        world.step(render=True)

    failures = []
    for name, camera in cameras:
        frame = _frame_to_uint8(camera.get_rgba())
        if frame is None:
            failures.append(name)
            continue
        image_path = ARGS.output / f"so101_{name}.png"
        Image.fromarray(frame).save(image_path)
        print(f"[preview] {image_path}")
    if failures:
        print(f"[preview-error] No image from: {', '.join(failures)}")
        return 1
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception as exc:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        print(f"[error] {type(exc).__name__}: {exc}")
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
