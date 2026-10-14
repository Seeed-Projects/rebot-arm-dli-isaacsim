#!/usr/bin/env python3
"""Capture headless still frames from the scene's front/side/external cameras.

The script mirrors the teleoperation runtime (leader_teleop.py): it opens the
generated scene, initializes the articulation and the three cameras, applies
domain-randomization resets, lets the vials settle, then writes one PNG per
camera and round.  Useful for docs and for checking the camera framing without
launching the GUI.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = PROJECT_ROOT / "scenes" / "rebot_stationery_sorting.usda"
DEFAULT_GRASP_CONFIG = PROJECT_ROOT / "config" / "grasp_config.json"
DEFAULT_DR_CONFIG = PROJECT_ROOT / "config" / "domain_randomization.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "media" / "camera_previews",
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--grasp-config", type=Path, default=DEFAULT_GRASP_CONFIG)
    parser.add_argument("--dr-config", type=Path, default=DEFAULT_DR_CONFIG)
    parser.add_argument(
        "--rounds",
        type=int,
        default=2,
        help="Number of reset/capture rounds (each round advances the DR seed)",
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-dr", action="store_true")
    parser.add_argument("--settle-steps", type=int, default=120)
    parser.add_argument("--warmup-frames", type=int, default=60)
    parser.add_argument(
        "--classic-renderer",
        action="store_true",
        help=(
            "Use the real-time renderer with the OptiX denoiser "
            "(/rtx/modes/rt) instead of RTX Real-Time 2.0; useful on GPUs "
            "where DLSS Ray Reconstruction fails and images look noisy"
        ),
    )
    parser.add_argument("--rebuild", action="store_true")
    return parser.parse_args()


ARGS = _parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import (  # noqa: E402
    get_current_stage,
    is_stage_loading,
    open_stage,
)
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402

from build_scene import (  # noqa: E402
    build_scene,
    camera_rig_is_configured,
    gripper_physics_is_fixed,
    robot_appearance_is_configured,
)
from domain_randomization import DomainRandomizer  # noqa: E402


if ARGS.classic_renderer:
    _settings = carb.settings.get_settings()
    _settings.set_bool("/rtx/modes/rt2/enabled", False)
    _settings.set_bool("/rtx/modes/rt/enabled", True)


def _wait_for_stage() -> None:
    while is_stage_loading():
        simulation_app.update()
    for _ in range(3):
        simulation_app.update()


def _scene_is_current(stage, required_paths: list[str]) -> bool:
    if any(not stage.GetPrimAtPath(path).IsValid() for path in required_paths):
        return False
    return (
        gripper_physics_is_fixed(stage)
        and robot_appearance_is_configured(stage)
        and camera_rig_is_configured(stage)
    )


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
    grasp_config = json.loads(ARGS.grasp_config.read_text())
    dr_config = json.loads(ARGS.dr_config.read_text())

    required_paths = [
        dr_config["side_camera"]["prim_path"],
        dr_config["front_camera"]["prim_path"],
        dr_config["front_camera"]["mount_prim_path"],
        dr_config["external_camera"]["prim_path"],
        dr_config["external_camera"]["mount_prim_path"],
        "/World/Looks/StationeryPhysics",
        "/World/Environment/Table/Top",
        grasp_config["vial_rack_prim_path"],
        grasp_config["workspace_prim_path"],
        *grasp_config["target_prim_paths"].values(),
    ]

    needs_rebuild = ARGS.rebuild or not ARGS.scene.exists()
    if not needs_rebuild:
        if not open_stage(str(ARGS.scene)):
            raise RuntimeError(f"Unable to open scene: {ARGS.scene}")
        _wait_for_stage()
        if not _scene_is_current(get_current_stage(), required_paths):
            print("[scene] Stale scene detected; rebuilding")
            needs_rebuild = True
    if needs_rebuild:
        build_scene(ARGS.scene)
        if not open_stage(str(ARGS.scene)):
            raise RuntimeError(f"Unable to reopen scene: {ARGS.scene}")
        _wait_for_stage()

    stage = get_current_stage()

    # Match leader_teleop's runtime physics configuration.
    joint_prim = stage.GetPrimAtPath(grasp_config["assist_joint_path"])
    if joint_prim.IsValid() and joint_prim.IsActive():
        UsdPhysics.Joint(joint_prim).GetJointEnabledAttr().Set(False)
        joint_prim.SetActive(False)
    target_paths = {
        name: str(path)
        for name, path in grasp_config["target_prim_paths"].items()
    }
    for path in target_paths.values():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Vial prim is missing: {path}")
        UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=float(grasp_config["physics_dt"]),
        rendering_dt=float(grasp_config["rendering_dt"]),
    )
    robot = world.scene.add(
        SingleArticulation(
            prim_path=grasp_config["robot_prim_path"], name="rebot_preview"
        )
    )
    targets = {
        name: world.scene.add(
            SingleRigidPrim(prim_path=path, name=f"preview_{name}")
        )
        for name, path in target_paths.items()
    }
    world.reset()
    robot.initialize()
    for target in targets.values():
        target.initialize()

    resolution = tuple(int(value) for value in dr_config["recording"]["resolution"])
    cameras = {
        "front": Camera(
            prim_path=dr_config["front_camera"]["prim_path"],
            name="front_camera",
            resolution=resolution,
        ),
        "side": Camera(
            prim_path=dr_config["side_camera"]["prim_path"],
            name="side_camera",
            resolution=resolution,
        ),
        "external": Camera(
            prim_path=dr_config["external_camera"]["prim_path"],
            name="external_camera",
            resolution=resolution,
        ),
    }
    for camera in cameras.values():
        camera.initialize()

    randomizer = DomainRandomizer(
        stage=stage,
        targets=targets,
        side_camera=cameras["side"],
        front_camera=cameras["front"],
        config=dr_config,
        enabled=not ARGS.no_dr,
        seed=ARGS.seed if ARGS.seed is not None else int(dr_config["seed"]),
    )

    dof_names = list(robot.dof_names)
    arm_indices = np.asarray(
        [dof_names.index(name) for name in grasp_config["arm_joint_names"]],
        dtype=np.int64,
    )
    gripper_indices = np.asarray(
        [dof_names.index(name) for name in grasp_config["gripper_joint_names"]],
        dtype=np.int64,
    )
    hold_arm = np.zeros(len(arm_indices), dtype=np.float64)
    hold_gripper = np.zeros(len(gripper_indices), dtype=np.float64)
    controller = robot.get_articulation_controller()

    def hold_pose(render: bool) -> None:
        # Hold the USD zero pose while the vials fall and settle, exactly as
        # the teleop loop does before any leader packet arrives.
        robot.set_joint_positions(hold_arm, joint_indices=arm_indices)
        robot.set_joint_velocities(
            np.zeros(len(arm_indices), dtype=np.float64),
            joint_indices=arm_indices,
        )
        controller.apply_action(
            ArticulationAction(
                joint_positions=np.concatenate((hold_arm, hold_gripper)),
                joint_indices=np.concatenate((arm_indices, gripper_indices)),
            )
        )
        world.step(render=render)

    ARGS.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[capture] Output directory: {ARGS.output_dir}")
    written = 0
    rounds = max(1, ARGS.rounds)
    for round_index in range(rounds):
        randomization = randomizer.reset()
        seed = int(randomization["seed"])
        for step in range(ARGS.settle_steps + ARGS.warmup_frames):
            hold_pose(render=True)
        summary = ", ".join(
            f"{name}=({item['position_m'][0]:.3f},{item['position_m'][1]:.3f})"
            for name, item in randomization["objects"].items()
        )
        print(f"[capture] Round {round_index + 1}/{rounds} seed={seed} objects: {summary}")
        for name, camera in cameras.items():
            frame = None
            for _attempt in range(60):
                frame = _frame_to_uint8(camera.get_rgba())
                if frame is not None and int(frame.max()) > 0:
                    break
                frame = None
                hold_pose(render=True)
            if frame is None:
                print(f"[capture-warn] {name}: camera produced no image")
                continue
            output_path = ARGS.output_dir / f"preview_{name}_seed{seed}.png"
            Image.fromarray(frame).save(output_path)
            written += 1
            print(
                f"[capture] {output_path.name} "
                f"({frame.shape[1]}x{frame.shape[0]}, mean={frame.mean():.1f})"
            )

    print(f"[capture] Wrote {written} image(s) to {ARGS.output_dir}")
    return 0 if written else 1


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception as exc:
        print(f"[error] {type(exc).__name__}: {exc}")
        raise
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
