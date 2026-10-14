#!/usr/bin/env python3
"""Mirror a reBot Arm 102 leader into the stationery-sorting scene."""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TELEOP_CONFIG = PROJECT_ROOT / "config" / "teleop_config.json"
DEFAULT_GRASP_CONFIG = PROJECT_ROOT / "config" / "grasp_config.json"
DEFAULT_DR_CONFIG = PROJECT_ROOT / "config" / "domain_randomization.json"
DEFAULT_SCENE = PROJECT_ROOT / "scenes" / "rebot_stationery_sorting.usda"
BRIDGE_SCRIPT = PROJECT_ROOT / "scripts" / "leader_bridge.py"
DATASET_WRITER_SCRIPT = PROJECT_ROOT / "scripts" / "dataset_writer.py"
# The vial asset ships its own collider mesh, and its shape is expressed as a
# box so the proximity heuristic does not have to assume an axis.
TARGET_COLLIDER_SUFFIXES = {
    "vials": "/collider",
    # The stationery targets get an authored convex-hull override prim.
    "stationery": "/Mesh",
}
TARGET_GRASP_GEOMETRIES = {
    "vials": {
        "vial_1": {"half_extents": (0.017, 0.017, 0.058)},
        "vial_2": {"half_extents": (0.017, 0.017, 0.058)},
        "vial_3": {"half_extents": (0.017, 0.017, 0.058)},
    },
    "stationery": {
        "pen_red": {"half_length": 0.068, "radius": 0.0040},
        "pen_blue": {"half_length": 0.078, "radius": 0.0047},
        "eraser": {"half_extents": (0.0223, 0.0093, 0.0053)},
    },
}
VARIANT_DEFAULTS = {
    "vials": {
        "scene": DEFAULT_SCENE,
        "grasp_config": DEFAULT_GRASP_CONFIG,
        "dr_config": DEFAULT_DR_CONFIG,
    },
    "stationery": {
        "scene": PROJECT_ROOT / "scenes" / "rebot_stationery_pencil_cup.usda",
        "grasp_config": PROJECT_ROOT / "config" / "grasp_config_stationery.json",
        "dr_config": PROJECT_ROOT
        / "config"
        / "domain_randomization_stationery.json",
    },
}


def _resolve_leader_python(value: str | Path) -> Path:
    """Resolve configured paths relative to the project, not the launch shell."""
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    # Do not use Path.resolve() here. A virtual environment's python executable
    # is commonly a symlink; resolving it selects the base interpreter and
    # prevents Python from finding the venv's pyvenv.cfg and installed packages.
    return Path(os.path.abspath(path))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI")
    parser.add_argument(
        "--variant",
        choices=sorted(VARIANT_DEFAULTS),
        default="vials",
        help="Task scene: vials into the vial rack (default), or pens and "
        "eraser into the fixed pencil cup",
    )
    parser.add_argument("--leader-port", help="Leader serial port, for example /dev/ttyUSB0")
    parser.add_argument("--leader-python", type=Path)
    parser.add_argument(
        "--no-start-bridge",
        action="store_true",
        help="Start only the Isaac receiver; run leader_bridge.py in another terminal",
    )
    parser.add_argument(
        "--strict-physics",
        action="store_true",
        help="Disable near-field grasp assist and rely only on collision and friction",
    )
    parser.add_argument("--rebuild-scene", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_TELEOP_CONFIG)
    parser.add_argument("--grasp-config", type=Path, default=None)
    parser.add_argument("--dr-config", type=Path, default=None)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument(
        "--no-dr",
        action="store_true",
        help="Use a fixed scene; R still resets the three objects without randomization",
    )
    parser.add_argument(
        "--no-recording",
        action="store_true",
        help="Disable S/C dataset recording controls",
    )
    parser.add_argument(
        "--single-viewport",
        action="store_true",
        help="Keep only the main debug viewport; front/external capture still runs in the background",
    )
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--repo-id")
    parser.add_argument("--task-name")
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--test",
        action="store_true",
        help="Explicit offline test using a mock leader; exits automatically in headless mode",
    )
    parser.add_argument(
        "--test-recording",
        action="store_true",
        help="Also validate front/external cameras and LeRobot dataset writing during --test",
    )
    parser.add_argument("--test-packets", type=int, default=330)
    args = parser.parse_args()
    if args.test_recording:
        args.test = True
    # Unspecified scene/config paths follow the selected task variant.
    defaults = VARIANT_DEFAULTS[args.variant]
    if args.scene is None:
        args.scene = defaults["scene"]
    if args.grasp_config is None:
        args.grasp_config = defaults["grasp_config"]
    if args.dr_config is None:
        args.dr_config = defaults["dr_config"]
    return args


ARGS = _parse_args()

# Variant-selected proximity geometry and collider naming; rebound here so
# the rest of the module can keep referencing the singular names.
TARGET_COLLIDER_SUFFIX = TARGET_COLLIDER_SUFFIXES[ARGS.variant]
TARGET_GRASP_GEOMETRY = TARGET_GRASP_GEOMETRIES[ARGS.variant]

try:
    from isaacsim import SimulationApp
except ImportError as exc:
    raise RuntimeError(
        "Run with /home/seeed/isaacsim/python.sh or use the project's ./run.sh"
    ) from exc

simulation_app = SimulationApp({"headless": ARGS.headless or ARGS.test})

if not (ARGS.headless or ARGS.test):
    # The default main-loop cap is 120 Hz, which on laptop GPUs keeps the
    # renderer pinned at its maximum frame rate; the desktop compositor then
    # starves and the whole machine appears to freeze.  Throttle the loop to
    # the 60 Hz leader stream so the GPU keeps headroom for the desktop.
    import carb

    _settings = carb.settings.get_settings()
    _settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
    _settings.set_int("/app/runLoops/main/rateLimitFrequency", 60)

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import (
    get_current_stage,
    is_stage_loading,
    open_stage,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, UsdPhysics

from build_scene import (
    GRIPPER_COLLIDER_PATHS,
    build_scene,
    camera_rig_is_configured,
    gripper_physics_is_fixed,
    robot_appearance_is_configured,
)
from domain_randomization import DomainRandomizer
from episode_recorder import EpisodeRecorder
from teleop_protocol import decode_payload

import carb
import omni.appwindow


class TeleopKeyboard:
    """Collect S/R/C key presses from the focused Isaac Sim viewport."""

    def __init__(self, enabled: bool) -> None:
        self.events: list[str] = []
        self._input = None
        self._keyboard = None
        self._subscription = None
        if not enabled:
            return
        window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = window.get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_event
        )

    def _on_event(self, event, *_args, **_kwargs) -> bool:
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return False
        key = event.input.name
        if key in {"S", "R", "C"}:
            self.events.append(key)
            return True
        return False

    def pop_events(self) -> list[str]:
        result = self.events
        self.events = []
        return result

    def cleanup(self) -> None:
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._subscription
            )
            self._subscription = None


def _quaternion_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a vector by a [w, x, y, z] quaternion."""
    w = float(quaternion[0])
    xyz = np.asarray(quaternion[1:4], dtype=np.float64)
    vector = np.asarray(vector, dtype=np.float64)
    return (
        vector
        + 2.0 * np.cross(xyz, np.cross(xyz, vector) + w * vector)
    )


def _quaternion_normalize(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("Zero-length quaternion")
    return quaternion / norm


def _quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    result = _quaternion_normalize(quaternion).copy()
    result[1:4] *= -1.0
    return result


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Multiply scalar-first [w, x, y, z] quaternions."""
    lw, lx, ly, lz = _quaternion_normalize(left)
    rw, rx, ry, rz = _quaternion_normalize(right)
    return _quaternion_normalize(
        np.asarray(
            [
                lw * rw - lx * rx - ly * ry - lz * rz,
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
            ],
            dtype=np.float64,
        )
    )


class LeaderTeleop:
    def __init__(self) -> None:
        self.config_path = ARGS.config.resolve()
        self.grasp_config_path = ARGS.grasp_config.resolve()
        self.dr_config_path = ARGS.dr_config.resolve()
        self.scene_path = ARGS.scene.resolve()
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.grasp_config = json.loads(
            self.grasp_config_path.read_text(encoding="utf-8")
        )
        self.dr_config = json.loads(
            self.dr_config_path.read_text(encoding="utf-8")
        )

        udp = self.config["udp"]
        self.host = str(udp["host"])
        self.port = int(udp["port"])
        self.watchdog_timeout = float(udp["watchdog_timeout_s"])
        command_filter = self.config["command_filter"]
        self.arm_control_mode = str(command_filter.get("mode", "drive"))
        self.filter_alpha = float(command_filter["alpha"])
        self.fast_filter_alpha = float(
            command_filter.get("fast_alpha", self.filter_alpha)
        )
        self.fast_filter_threshold = np.radians(
            float(command_filter.get("fast_threshold_deg", 1.0))
        )
        self.arm_deadband = np.radians(
            float(command_filter.get("deadband_deg", 0.0))
        )
        self.max_arm_step = np.radians(
            float(command_filter["max_arm_step_deg"])
        )
        self.gripper_alpha = float(
            command_filter.get("gripper_alpha", 1.0)
        )
        self.max_gripper_step = float(command_filter["max_gripper_step_m"])
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("command_filter.alpha must be in (0, 1]")
        if self.arm_control_mode not in {"direct", "drive"}:
            raise ValueError("command_filter.mode must be either direct or drive")
        if not 0.0 < self.fast_filter_alpha <= 1.0:
            raise ValueError("command_filter.fast_alpha must be in (0, 1]")
        if self.fast_filter_threshold < 0.0 or self.arm_deadband < 0.0:
            raise ValueError("Filter thresholds and deadbands cannot be negative")
        if not 0.0 < self.gripper_alpha <= 1.0:
            raise ValueError(
                "command_filter.gripper_alpha must be in (0, 1]"
            )

        self.physics_dt = float(self.grasp_config["physics_dt"])
        self.rendering_dt = float(self.grasp_config["rendering_dt"])
        self.robot_path = str(self.grasp_config["robot_prim_path"])
        self.target_paths = {
            str(name): str(path)
            for name, path in self.grasp_config["target_prim_paths"].items()
        }
        # The vial config names the rack "vial_rack_*"; the stationery
        # config uses the generic receptacle keys.
        if "receptacle_prim_path" in self.grasp_config:
            self.vial_rack_path = str(
                self.grasp_config["receptacle_prim_path"]
            )
        else:
            self.vial_rack_path = str(
                self.grasp_config["vial_rack_prim_path"]
            )
        self.receptacle_label = str(
            self.grasp_config.get("receptacle_label", "vial rack")
        )
        self.workspace_path = str(
            self.grasp_config["workspace_prim_path"]
        )
        self.assist_joint_path = str(self.grasp_config["assist_joint_path"])
        self.assist_body_path = str(self.grasp_config["assist_body_path"])
        self.arm_names = [
            item["sim_joint"] for item in self.config["arm_mapping"]
        ]
        self.gripper_names = list(self.grasp_config["gripper_joint_names"])
        recording_cfg = self.dr_config["recording"]
        self.recording_enabled = not ARGS.no_recording
        self.record_fps = int(recording_cfg["fps"])
        self.record_resolution = tuple(
            int(value) for value in recording_cfg["resolution"]
        )
        configured_dataset_root = Path(recording_cfg["dataset_root"])
        if not configured_dataset_root.is_absolute():
            configured_dataset_root = PROJECT_ROOT / configured_dataset_root
        self.dataset_root = (
            ARGS.dataset_root.resolve()
            if ARGS.dataset_root
            else configured_dataset_root.resolve()
        )
        if ARGS.test_recording:
            test_parent = Path(
                tempfile.mkdtemp(prefix="rebot_stationery_dataset_", dir="/tmp")
            )
            self.dataset_root = test_parent / "dataset"
        self.repo_id = ARGS.repo_id or str(recording_cfg["repo_id"])
        self.task_name = ARGS.task_name or str(recording_cfg["task_name"])
        self.dr_enabled = bool(self.dr_config["enabled"]) and not ARGS.no_dr
        self.dr_seed = (
            int(ARGS.seed)
            if ARGS.seed is not None
            else (
                227
                if ARGS.test
                else int(self.dr_config["seed"])
            )
        )
        self.render_frames = (
            not (ARGS.headless or ARGS.test) or ARGS.test_recording
        )

        assist = self.config["grasp_assist"]
        self.assist_enabled = bool(assist["enabled"]) and not ARGS.strict_physics
        self.assist_close = float(assist["close_threshold_m"])
        self.assist_release = float(assist["release_threshold_m"])
        self.assist_distance = float(assist["max_grasp_distance_m"])
        self.assist_region_half_length = max(
            0.0, float(assist.get("grasp_region_half_length_m", 0.0))
        )
        self.assist_region_samples = max(
            1, int(assist.get("grasp_region_samples", 1))
        )
        self.assist_activation_opening = float(
            assist.get("activation_opening_m", 0.028)
        )
        self.allow_contact_lock_fallback = bool(
            assist.get("allow_contact_lock_fallback", True)
        )
        self.contact_lock_enabled = bool(
            assist.get("contact_lock_enabled", True)
        )
        self.contact_lock_opening = float(
            assist.get("contact_lock_opening_m", 0.018)
        )
        self.contact_lock_release = float(
            assist.get("contact_lock_release_m", self.assist_release)
        )
        self.release_collision_delay_steps = max(
            1, int(assist.get("release_collision_delay_steps", 3))
        )
        self.release_collision_max_steps = max(
            self.release_collision_delay_steps,
            int(assist.get("release_collision_max_steps", 24)),
        )
        self.release_contact_restore_opening = float(
            assist.get("release_contact_restore_opening_m", 0.038)
        )
        self.assist_offset = np.asarray(
            self.grasp_config["assist_grasp_offset_m"], dtype=np.float64
        )
        self.assist_orientation = np.asarray(
            self.grasp_config["assist_grasp_orientation_wxyz"],
            dtype=np.float64,
        )
        if "receptacle" in self.grasp_config:
            rack_cfg = self.grasp_config["receptacle"]
        else:
            rack_cfg = self.grasp_config["vial_rack"]
        self.rack_center_xy = np.asarray(
            rack_cfg["center_xy_m"], dtype=np.float64
        )
        self.rack_success_radius = float(rack_cfg["success_radius_m"])
        self.rack_success_min_z = float(rack_cfg["success_min_z_m"])
        self.rack_success_max_z = float(rack_cfg["success_max_z_m"])
        self.rack_success_max_speed = float(
            rack_cfg["success_max_speed_mps"]
        )
        self.rack_success_settle_time = float(
            rack_cfg["success_settle_time_s"]
        )

        self.world: World | None = None
        self.robot: SingleArticulation | None = None
        self.targets: dict[str, SingleRigidPrim] = {}
        self.active_target_name: str | None = None
        self.active_target: SingleRigidPrim | None = None
        self.active_target_path: str | None = None
        self.assist_link_index: int | None = None
        self.controller = None
        self.arm_indices: np.ndarray | None = None
        self.gripper_indices: np.ndarray | None = None
        self.arm_lower = np.zeros(6, dtype=np.float64)
        self.arm_upper = np.zeros(6, dtype=np.float64)
        self.gripper_lower = np.zeros(2, dtype=np.float64)
        self.gripper_upper = np.full(2, 0.05, dtype=np.float64)
        self.command_arm = np.zeros(6, dtype=np.float64)
        self.command_gripper = 0.0
        self.latest_received_arm = self.command_arm.copy()
        self.latest_received_gripper = self.command_gripper
        self.max_observed_gripper = np.zeros(2, dtype=np.float64)

        self.input_socket: socket.socket | None = None
        self.bridge_process: subprocess.Popen | None = None
        self.side_camera: Camera | None = None
        self.front_camera: Camera | None = None
        self.external_camera: Camera | None = None
        self.side_viewport = None
        self.front_viewport = None
        self.external_viewport = None
        self.randomizer: DomainRandomizer | None = None
        self.recorder: EpisodeRecorder | None = None
        self.keyboard = TeleopKeyboard(
            enabled=not (ARGS.headless or ARGS.test)
        )
        self.last_sequence = -1
        self.last_packet_monotonic = 0.0
        self.packet_count = 0
        self._test_packet_queue: deque[tuple[np.ndarray, float, int]] = deque()
        self._test_consume_countdown = 2
        self.grasp_attached = False
        self.gripper_contact_locked = False
        self.gripper_contact_target = np.zeros(2, dtype=np.float64)
        self.release_collision_pending_steps = 0
        self.attached_relative_position = np.zeros(3, dtype=np.float64)
        self.attached_relative_orientation = np.asarray(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )
        self.min_grasp_distance = float("inf")
        self._watchdog_reported = False
        self.initial_target_z: dict[str, float] = {}
        self.max_object_lift = 0.0
        self.episode_grasped = False
        self.episode_success = False
        self.rack_settle_elapsed = 0.0
        self.objects_in_rack: set[str] = set()
        self.current_randomization: dict = {}
        self.sim_elapsed = 0.0
        self.next_record_time = 0.0
        self.test_record_started = False
        self.test_recording_ok = not ARGS.test_recording
        self.test_gripper_hold_error = float("inf") if ARGS.test else 0.0
        self.test_gripper_asymmetry = float("inf") if ARGS.test else 0.0
        self.test_attached_gripper_error = float("inf") if ARGS.test else 0.0
        self.test_grasp_attached = False
        self.test_object_lift = float("-inf") if ARGS.test else 0.0

    @staticmethod
    def _wait_for_stage() -> None:
        while is_stage_loading():
            simulation_app.update()
        for _ in range(3):
            simulation_app.update()

    def _configure_stage_physics(self) -> None:
        stage = get_current_stage()
        joint_prim = stage.GetPrimAtPath(self.assist_joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(f"Grasp-assist joint is missing: {self.assist_joint_path}")
        for name, path in self.target_paths.items():
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Stationery prim is missing: {name}={path}")
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)
        if joint_prim.IsActive():
            UsdPhysics.Joint(joint_prim).GetJointEnabledAttr().Set(False)
            joint_prim.SetActive(False)

    def setup(self) -> None:
        if ARGS.rebuild_scene or not self.scene_path.exists():
            build_scene(self.scene_path, variant=ARGS.variant)
        if not open_stage(str(self.scene_path)):
            raise RuntimeError(f"Unable to open scene: {self.scene_path}")
        self._wait_for_stage()
        stage = get_current_stage()
        required_scene_prims = [
            self.dr_config["side_camera"]["prim_path"],
            self.dr_config["front_camera"]["prim_path"],
            self.dr_config["front_camera"]["mount_prim_path"],
            self.dr_config["external_camera"]["prim_path"],
            self.dr_config["external_camera"]["mount_prim_path"],
            "/World/Looks/StationeryPhysics",
            "/World/Environment/Table/Top",
            self.vial_rack_path,
            self.workspace_path,
            *self.target_paths.values(),
        ]
        missing_scene_content = any(
            not stage.GetPrimAtPath(path).IsValid()
            for path in required_scene_prims
        )
        missing_gripper_fix = not gripper_physics_is_fixed(stage)
        missing_robot_appearance = not robot_appearance_is_configured(stage)
        missing_camera_rig = not camera_rig_is_configured(stage)
        if (
            missing_scene_content
            or missing_gripper_fix
            or missing_robot_appearance
            or missing_camera_rig
        ):
            reasons = []
            if missing_scene_content:
                reasons.append(
                    f"cameras, table, objects, or fixed {self.receptacle_label}"
                )
            if missing_gripper_fix:
                reasons.append("PR #21 gravity-resistant gripper drives and 1:1 linkage")
            if missing_robot_appearance:
                reasons.append("official Seeed robot color scheme")
            if missing_camera_rig:
                reasons.append("Seeed UVC32 physical mount and dual 4:3 cameras")
            print(
                "[scene] An outdated scene was detected; rebuilding "
                + "、".join(reasons)
            )
            build_scene(self.scene_path, variant=ARGS.variant)
            if not open_stage(str(self.scene_path)):
                raise RuntimeError(f"Unable to reopen scene: {self.scene_path}")
            self._wait_for_stage()
        self._configure_stage_physics()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=self.physics_dt,
            rendering_dt=self.rendering_dt,
        )
        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=self.robot_path, name="rebot_teleop")
        )
        self.targets = {
            name: SingleRigidPrim(prim_path=path, name=f"target_{name}")
            for name, path in self.target_paths.items()
        }
        self.world.reset()
        self.robot.initialize()
        for target in self.targets.values():
            target.initialize()
        default_target = str(self.grasp_config.get("test_object_name", "vial_1"))
        self._set_active_target(default_target)
        self.side_camera = Camera(
            prim_path=self.dr_config["side_camera"]["prim_path"],
            name="side_camera",
            resolution=self.record_resolution,
        )
        self.front_camera = Camera(
            prim_path=self.dr_config["front_camera"]["prim_path"],
            name="front_camera",
            resolution=self.record_resolution,
        )
        self.external_camera = Camera(
            prim_path=self.dr_config["external_camera"]["prim_path"],
            name="external_camera",
            resolution=self.record_resolution,
        )
        self.side_camera.initialize()
        self.front_camera.initialize()
        self.external_camera.initialize()
        assist_link_name = self.assist_body_path.rsplit("/", 1)[-1]
        self.assist_link_index = self.robot._articulation_view.get_link_index(
            assist_link_name
        )

        dof_names = list(self.robot.dof_names)
        missing = [
            name
            for name in self.arm_names + self.gripper_names
            if name not in dof_names
        ]
        if missing:
            raise RuntimeError(
                f"Robot DOF mismatch; missing {missing}; available DOFs: {dof_names}"
            )
        self.arm_indices = np.asarray(
            [dof_names.index(name) for name in self.arm_names], dtype=np.int64
        )
        self.gripper_indices = np.asarray(
            [dof_names.index(name) for name in self.gripper_names],
            dtype=np.int64,
        )
        properties = self.robot.dof_properties
        lower = np.asarray(properties["lower"], dtype=np.float64)
        upper = np.asarray(properties["upper"], dtype=np.float64)
        self.arm_lower = lower[self.arm_indices]
        self.arm_upper = upper[self.arm_indices]
        self.gripper_lower = lower[self.gripper_indices]
        self.gripper_upper = upper[self.gripper_indices]
        self.controller = self.robot.get_articulation_controller()

        if ARGS.test:
            available_properties = set(properties.dtype.names or ())
            diagnostic_names = (
                "stiffness",
                "damping",
                "maxEffort",
                "maxVelocity",
            )
            diagnostics = {
                name: np.asarray(properties[name])[self.gripper_indices]
                .round(6)
                .tolist()
                for name in diagnostic_names
                if name in available_properties
            }
            print(f"[test-gripper-dof-properties] {diagnostics}")

        # No autonomous home trajectory: hold the USD zero pose until leader data arrives.
        self._apply_targets()

        self.randomizer = DomainRandomizer(
            stage=get_current_stage(),
            targets=self.targets,
            side_camera=self.side_camera,
            front_camera=self.front_camera,
            config=self.dr_config,
            enabled=self.dr_enabled,
            seed=self.dr_seed,
        )
        if self.recording_enabled:
            leader_python = _resolve_leader_python(
                ARGS.leader_python
                if ARGS.leader_python
                else self.config["leader"]["python"]
            )
            self.recorder = EpisodeRecorder(
                dataset_root=self.dataset_root,
                repo_id=self.repo_id,
                task_name=self.task_name,
                fps=self.record_fps,
                resolution=self.record_resolution,
                jpeg_quality=int(
                    self.dr_config["recording"]["jpeg_quality"]
                ),
                writer_python=leader_python,
                writer_script=DATASET_WRITER_SCRIPT,
            )

        self.input_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.input_socket.bind((self.host, self.port))
        self.input_socket.setblocking(False)

        if not (ARGS.headless or ARGS.test):
            try:
                if not ARGS.single_viewport:
                    from isaacsim.core.utils.viewports import (
                        create_viewport_for_camera,
                    )

                    self.side_viewport = create_viewport_for_camera(
                        "Side Camera",
                        self.dr_config["side_camera"]["prim_path"],
                        width=640,
                        height=480,
                        position_x=30,
                        position_y=90,
                    )
                    self.front_viewport = create_viewport_for_camera(
                        "Front Camera (Wrist UVC32)",
                        self.dr_config["front_camera"]["prim_path"],
                        width=640,
                        height=480,
                        position_x=700,
                        position_y=90,
                    )
                    self.external_viewport = create_viewport_for_camera(
                        "External Camera (Overhead)",
                        self.dr_config["external_camera"]["prim_path"],
                        width=640,
                        height=480,
                        position_x=1370,
                        position_y=90,
                    )
                    print(
                        "[camera] The main viewport remains an unrecorded debug view; "
                        "opened the side, wrist-mounted front and overhead external "
                        "camera viewports"
                    )
            except Exception as exc:
                print(f"[camera-warn] Unable to configure camera viewports: {exc}")

        self._reset_episode()

        self._start_bridge()
        mode = "contact physics only" if ARGS.strict_physics else "near-field reliable grasp assist"
        dr_mode = "enabled" if self.dr_enabled else "disabled (fixed scene)"
        print("=" * 76)
        print("  reBot 102 Leader -> Isaac Sim B601 real-time teleoperation")
        print(f"  Receiver: udp://{self.host}:{self.port}")
        print(f"  Grasp mode: {mode}")
        print(
            f"  Task: {self.dr_config['recording']['task_name']} "
            f"({self.rack_center_xy[0]:.2f}, "
            f"{self.rack_center_xy[1]:.2f})m"
        )
        print(f"  Domain randomization: {dr_mode}; R=reset/randomize")
        if self.recording_enabled:
            print(
                f"  Recording: S=start/stop, C=cancel; "
                f"{self.record_fps} FPS with front/external cameras"
            )
            print(f"  Dataset: {self.dataset_root}")
        else:
            print("  Recording: disabled")
        print("  Safety: holds the current joint targets before data arrives and after a timeout")
        print(f"  DOF : {dof_names}")
        print(
            "  Gripper: "
            + "  ".join(
                f"{name}[{low:.4f}, {high:.4f}]m"
                for name, low, high in zip(
                    self.gripper_names,
                    self.gripper_lower,
                    self.gripper_upper,
                )
            )
            + "; low-force physical drives, shared 1:1 finger target, and stationery contact lock"
        )
        print("=" * 76)

    def _set_active_target(self, name: str) -> None:
        if name not in self.targets:
            raise KeyError(f"Unknown stationery target: {name}")
        self.active_target_name = name
        self.active_target = self.targets[name]
        self.active_target_path = self.target_paths[name]

    def _set_target_collision_enabled(self, enabled: bool) -> None:
        if self.active_target_path is None:
            return
        collider = get_current_stage().GetPrimAtPath(
            f"{self.active_target_path}{TARGET_COLLIDER_SUFFIX}"
        )
        if collider.IsValid():
            UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Set(
                enabled
            )

    def _set_target_kinematic(self, enabled: bool) -> None:
        if self.active_target_path is None:
            return
        target_prim = get_current_stage().GetPrimAtPath(
            self.active_target_path
        )
        if target_prim.IsValid():
            UsdPhysics.RigidBodyAPI(
                target_prim
            ).GetKinematicEnabledAttr().Set(enabled)

    def _set_target_gripper_contacts_enabled(self, enabled: bool) -> None:
        """Filter finger contacts while retaining table/rack contacts."""
        if self.active_target_path is None:
            return
        collider = get_current_stage().GetPrimAtPath(
            f"{self.active_target_path}{TARGET_COLLIDER_SUFFIX}"
        )
        if not collider.IsValid():
            return
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(collider)
        relationship = filtered_pairs.CreateFilteredPairsRel()
        targets = [] if enabled else [
            Sdf.Path(path) for path in GRIPPER_COLLIDER_PATHS
        ]
        relationship.SetTargets(targets)

    def _detach_target(
        self, *, silent: bool, defer_collision: bool = False
    ) -> None:
        joint_prim = get_current_stage().GetPrimAtPath(
            self.assist_joint_path
        )
        if joint_prim.IsValid() and joint_prim.IsActive():
            UsdPhysics.Joint(joint_prim).GetJointEnabledAttr().Set(False)
            joint_prim.SetActive(False)
        was_attached = self.grasp_attached
        self.grasp_attached = False
        self.gripper_contact_locked = False
        self._set_target_kinematic(False)
        if was_attached and defer_collision:
            # A fixed joint can leave a large solver velocity on the released
            # body. Start release from rest, then give the fingers a few
            # physics frames to move clear before contacts are restored.
            self.release_collision_pending_steps = 1
            self._set_target_collision_enabled(True)
            self._set_target_gripper_contacts_enabled(False)
            self.active_target.set_linear_velocity(
                np.zeros(3, dtype=np.float64)
            )
            self.active_target.set_angular_velocity(
                np.zeros(3, dtype=np.float64)
            )
        else:
            self.release_collision_pending_steps = 0
            self._set_target_collision_enabled(True)
            self._set_target_gripper_contacts_enabled(True)
        if was_attached and not silent:
            print(
                "[RELEASE] The grasp constraint was removed; finger collisions "
                f"will be restored after separation and {self.active_target_name} is in free fall"
            )

    def _update_release_transition(self) -> None:
        """Restore contacts only after the opening fingers have moved clear."""
        if self.release_collision_pending_steps <= 0:
            return
        actual = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        opening = float(np.mean(actual))
        distance = self._grasp_distance()
        minimum_wait_done = (
            self.release_collision_pending_steps
            >= self.release_collision_delay_steps
        )
        safely_separated = (
            opening >= self.release_contact_restore_opening
            or distance >= self.assist_distance + 0.02
        )
        timed_out = (
            self.release_collision_pending_steps
            >= self.release_collision_max_steps
        )
        if minimum_wait_done and (safely_separated or timed_out):
            self._set_target_gripper_contacts_enabled(True)
            self.release_collision_pending_steps = 0
            print(
                f"[RELEASE-CONTACT] {self.active_target_name} left the grasp region; restoring finger contacts "
                f"(opening={opening:.3f}m, distance={distance:.3f}m)"
            )
        else:
            self.release_collision_pending_steps += 1

    def _reset_episode(self) -> None:
        assert self.randomizer is not None
        assert self.robot is not None
        assert self.arm_indices is not None
        assert self.gripper_indices is not None
        if self.recorder is not None and self.recorder.recording:
            self._stop_recording()
        self._detach_target(silent=True)
        for name in self.targets:
            self._set_active_target(name)
            self._set_target_collision_enabled(True)
            self._set_target_gripper_contacts_enabled(True)
        self._apply_targets()
        self.current_randomization = self.randomizer.reset()
        if ARGS.test:
            # Keep the mock-leader grasp regression deterministic.
            test_name = str(self.grasp_config["test_object_name"])
            self._set_active_target(test_name)
            test_pose = self.grasp_config["test_object_pose"]
            test_position = np.asarray(
                test_pose["position"], dtype=np.float64
            )
            test_orientation = np.asarray(
                test_pose["orientation_wxyz"], dtype=np.float64
            )
            self.active_target.set_world_pose(
                position=test_position,
                orientation=test_orientation,
            )
            self.active_target.set_linear_velocity(np.zeros(3, dtype=np.float64))
            self.active_target.set_angular_velocity(np.zeros(3, dtype=np.float64))
            self.current_randomization["objects"][test_name]["position_m"] = (
                test_position.tolist()
            )
            # The mock-leader regression grabs the target while it is held at
            # the calibrated pose. Keep the body kinematic so it does not fall
            # before the scripted grasp arrives.
            self._set_target_kinematic(True)
        # Let all stationery fall onto the work surface and warm up cameras.
        for settle_step in range(60):
            self._apply_targets()
            self.world.step(
                render=(
                    settle_step < 12
                    or not (ARGS.headless or ARGS.test)
                )
            )
            self.sim_elapsed += self.physics_dt
        self.initial_target_z = {
            name: float(target.get_world_pose()[0][2])
            for name, target in self.targets.items()
        }
        self.max_object_lift = 0.0
        self.episode_grasped = False
        self.episode_success = False
        self.rack_settle_elapsed = 0.0
        self.objects_in_rack = set()
        self.min_grasp_distance = float("inf")
        objects = self.current_randomization["objects"]
        summary = ", ".join(
            f"{name}=({item['position_m'][0]:.3f},{item['position_m'][1]:.3f})"
            for name, item in objects.items()
        )
        print(
            f"[RESET] seed={self.current_randomization['seed']} "
            f"objects: {summary}"
        )

    def _leader_action_vector(self) -> np.ndarray:
        values: list[float] = []
        for index, mapping in enumerate(self.config["arm_mapping"]):
            sim_deg = float(np.degrees(self.latest_received_arm[index]))
            scale = float(mapping["scale"])
            offset = float(mapping.get("offset_deg", 0.0))
            values.append((sim_deg - offset) / scale)
        gripper = self.config["gripper_mapping"]
        sim_closed = float(gripper["sim_closed_m"])
        sim_open = float(gripper["sim_open_m"])
        openness = (
            (self.latest_received_gripper - sim_closed)
            / (sim_open - sim_closed)
        )
        openness = float(np.clip(openness, 0.0, 1.0))
        values.append(
            float(gripper["leader_closed_deg"])
            + openness
            * (
                float(gripper["leader_open_deg"])
                - float(gripper["leader_closed_deg"])
            )
        )
        return np.asarray(values, dtype=np.float32)

    def _follower_state_vector(self) -> np.ndarray:
        assert self.robot is not None
        assert self.arm_indices is not None
        assert self.gripper_indices is not None
        sim_arm = np.asarray(
            self.robot.get_joint_positions(joint_indices=self.arm_indices),
            dtype=np.float64,
        )
        sim_gripper = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        directions = self.config["dataset_mapping"][
            "follower_joint_directions"
        ]
        follower_arm: list[float] = []
        for index, mapping in enumerate(self.config["arm_mapping"]):
            sim_deg = float(np.degrees(sim_arm[index]))
            leader_scale = float(mapping["scale"])
            follower_direction = float(
                directions[mapping["leader_joint"]]
            )
            follower_arm.append(
                sim_deg * follower_direction / leader_scale
            )

        gripper = self.config["gripper_mapping"]
        sim_closed = float(gripper["sim_closed_m"])
        sim_open = float(gripper["sim_open_m"])
        mean_gripper = float(np.mean(sim_gripper))
        openness = float(
            np.clip(
                (mean_gripper - sim_closed) / (sim_open - sim_closed),
                0.0,
                1.0,
            )
        )
        leader_gripper = float(gripper["leader_closed_deg"]) + openness * (
            float(gripper["leader_open_deg"])
            - float(gripper["leader_closed_deg"])
        )
        follower_gripper = leader_gripper * float(
            self.config["dataset_mapping"]["follower_gripper_scale"]
        )
        return np.asarray(
            follower_arm + [follower_gripper], dtype=np.float32
        )

    def _start_recording(self) -> None:
        if self.recorder is None:
            print("[record-warn] Dataset recording is disabled")
            return
        if (
            self.last_packet_monotonic <= 0.0
            or time.monotonic() - self.last_packet_monotonic
            > self.watchdog_timeout
        ):
            print("[record-warn] The leader is offline; refusing to start an empty episode")
            return
        self.recorder.start(self.current_randomization)
        self.next_record_time = self.sim_elapsed

    def _stop_recording(self) -> None:
        if self.recorder is None:
            return
        self.recorder.stop(
            success=self.episode_success,
            grasped=self.episode_grasped,
            max_object_lift_m=self.max_object_lift,
        )

    def _capture_record_frame(self) -> None:
        if self.recorder is None or not self.recorder.recording:
            return
        assert self.front_camera is not None
        assert self.external_camera is not None
        if self.sim_elapsed + 1e-9 < self.next_record_time:
            return
        front = np.asarray(self.front_camera.get_rgba())
        external = np.asarray(self.external_camera.get_rgba())
        if front.size == 0 or external.size == 0:
            print("[camera-warn] RTX cameras have not produced images yet; skipping this frame")
            return
        self.recorder.capture(
            action=self._leader_action_vector(),
            state=self._follower_state_vector(),
            images={
                "front": front[:, :, :3],
                "external": external[:, :, :3],
            },
        )
        self.next_record_time += 1.0 / self.record_fps

    def _handle_keyboard(self) -> None:
        for key in self.keyboard.pop_events():
            if key == "S":
                if self.recorder is not None and self.recorder.recording:
                    self._stop_recording()
                else:
                    self._start_recording()
            elif key == "C":
                if self.recorder is not None:
                    self.recorder.cancel()
            elif key == "R":
                self._reset_episode()

    def _update_episode_metrics(self) -> None:
        gripper_positions = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        self.max_observed_gripper = np.maximum(
            self.max_observed_gripper, gripper_positions
        )
        in_rack: set[str] = set()
        all_stable = True
        distances: dict[str, float] = {}
        for name, target in self.targets.items():
            position, orientation = target.get_world_pose()
            position = np.asarray(position, dtype=np.float64)
            lift = float(position[2]) - self.initial_target_z.get(name, 0.0)
            self.max_object_lift = max(self.max_object_lift, lift)
            velocity = np.asarray(
                target.get_linear_velocity(), dtype=np.float64
            )
            all_stable = all_stable and (
                float(np.linalg.norm(velocity)) <= self.rack_success_max_speed
            )
            distance = float(
                np.linalg.norm(position[:2] - self.rack_center_xy)
            )
            distances[name] = distance
            height_ok = (
                self.rack_success_min_z
                <= float(position[2])
                <= self.rack_success_max_z
            )
            orientation_ok = True
            if name.startswith("pen_"):
                pen_axis = _quaternion_rotate(
                    _quaternion_normalize(orientation),
                    np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
                )
                orientation_ok = abs(float(pen_axis[2])) >= 0.55
            if (
                distance <= self.rack_success_radius
                and height_ok
                and orientation_ok
            ):
                in_rack.add(name)
        if in_rack != self.objects_in_rack:
            self.objects_in_rack = in_rack
            print(
                f"[SORT] In {self.receptacle_label}: "
                + (", ".join(sorted(in_rack)) if in_rack else "none")
                + f" ({len(in_rack)}/{len(self.targets)})"
            )
        released = (
            not self.grasp_attached
            and float(np.mean(gripper_positions))
            >= self.contact_lock_release
        )
        all_in_rack = len(in_rack) == len(self.targets)
        if all_in_rack and released and all_stable:
            self.rack_settle_elapsed += self.physics_dt
        else:
            self.rack_settle_elapsed = 0.0

        if (
            not self.episode_success
            and self.rack_settle_elapsed >= self.rack_success_settle_time
        ):
            self.episode_success = True
            print(
                f"[SUCCESS] All objects are stable in the "
                f"{self.receptacle_label}: "
                f"center distances={distances}, "
                f"settle time={self.rack_settle_elapsed:.2f}s"
            )

    def _start_bridge(self) -> None:
        if ARGS.no_start_bridge:
            print("[teleop] Waiting for an external leader_bridge.py process")
            return
        leader_python = _resolve_leader_python(
            ARGS.leader_python
            if ARGS.leader_python
            else self.config["leader"]["python"]
        )
        if not leader_python.is_file():
            raise FileNotFoundError(f"LeRobot Python was not found: {leader_python}")
        command = [
            str(leader_python),
            str(BRIDGE_SCRIPT),
            "--config",
            str(self.config_path),
            "--host",
            self.host,
            "--udp-port",
            str(self.port),
        ]
        if ARGS.leader_port:
            command.extend(["--port", ARGS.leader_port])
        if ARGS.test:
            command.extend(["--mock", "--mock-profile", "grasp"])
        bridge_environment = os.environ.copy()
        # Isaac's python.sh injects its Python 3.10 standard library into
        # PYTHONPATH/PYTHONHOME. The leader runs in LeRobot's Python 3.12 venv,
        # so inheriting those variables causes an immediate SRE ABI mismatch.
        bridge_environment.pop("PYTHONHOME", None)
        bridge_environment.pop("PYTHONPATH", None)
        bridge_environment["VIRTUAL_ENV"] = str(leader_python.parent.parent)
        self.bridge_process = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
            env=bridge_environment,
        )

    def _recv_latest(self) -> tuple[np.ndarray, float] | None:
        assert self.input_socket is not None
        latest = None
        while True:
            try:
                packet, _address = self.input_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                joint_positions, gripper, sequence, _timestamp = decode_payload(
                    packet
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                print(f"[udp-warn] Dropping an invalid packet: {exc}")
                continue

            if ARGS.test:
                # The mock bridge emits a wall-clock scripted timeline. Queue
                # every packet and replay it in order at the nominal 60 Hz
                # simulation rate below, so a render-limited loop cannot
                # compress the timeline into slow motion.
                self._test_packet_queue.append(
                    (
                        np.asarray(joint_positions, dtype=np.float64),
                        float(gripper),
                        int(sequence),
                    )
                )
                continue

            now = time.monotonic()
            sequence_restarted = (
                self.last_packet_monotonic > 0.0
                and now - self.last_packet_monotonic > self.watchdog_timeout
            )
            if sequence <= self.last_sequence and not sequence_restarted:
                continue
            self.last_sequence = sequence
            self.last_packet_monotonic = now
            self.packet_count += 1
            latest = (
                np.asarray(joint_positions, dtype=np.float64),
                float(gripper),
            )
        if ARGS.test:
            # One queued packet every two physics steps: 60 Hz of scripted
            # leader data on the 120 Hz simulation clock.
            self._test_consume_countdown -= 1
            if self._test_consume_countdown <= 0 and self._test_packet_queue:
                joint_positions, gripper, sequence = (
                    self._test_packet_queue.popleft()
                )
                self._test_consume_countdown = 2
                self.last_sequence = sequence
                self.last_packet_monotonic = time.monotonic()
                self.packet_count += 1
                latest = (joint_positions, gripper)
        return latest

    def _update_command(self, arm_target: np.ndarray, gripper_target: float) -> None:
        arm_target = np.clip(arm_target, self.arm_lower, self.arm_upper)
        gripper_target = float(
            np.clip(
                gripper_target,
                float(np.min(self.gripper_lower)),
                float(np.max(self.gripper_upper)),
            )
        )
        arm_error = arm_target - self.command_arm
        arm_delta = np.clip(
            arm_error,
            -self.max_arm_step,
            self.max_arm_step,
        )
        # Suppress encoder noise near a held pose, but respond quickly when
        # the leader actually moves.  This avoids choosing between a stable
        # yet sluggish fixed low-pass and a fast but visibly jittery one.
        arm_delta = np.where(
            np.abs(arm_error) <= self.arm_deadband,
            0.0,
            arm_delta,
        )
        arm_alpha = np.where(
            np.abs(arm_error) >= self.fast_filter_threshold,
            self.fast_filter_alpha,
            self.filter_alpha,
        )
        gripper_delta = float(
            np.clip(
                gripper_target - self.command_gripper,
                -self.max_gripper_step,
                self.max_gripper_step,
            )
        )
        if self.arm_control_mode == "direct":
            # Match Seeed's Real-to-Sim receiver: the six arm joints mirror
            # the newest leader sample immediately.  The position drives are
            # still given the same targets below, so they hold the pose after
            # a watchdog timeout instead of chasing a stale trajectory.
            self.command_arm = arm_target.copy()
        else:
            self.command_arm += arm_alpha * arm_delta
        self.command_gripper += self.gripper_alpha * gripper_delta
        self.latest_received_arm = arm_target
        self.latest_received_gripper = gripper_target

    def _gripper_target_positions(self) -> np.ndarray:
        """Match Seeed's receiver: one command, per-finger USD clipping."""
        if self.gripper_contact_locked:
            return np.clip(
                self.gripper_contact_target,
                self.gripper_lower,
                self.gripper_upper,
            )
        return np.clip(
            np.full(2, self.command_gripper, dtype=np.float64),
            self.gripper_lower,
            self.gripper_upper,
        )

    def _apply_targets(self) -> None:
        assert self.robot is not None
        assert self.controller is not None
        assert self.arm_indices is not None
        assert self.gripper_indices is not None
        # In direct Real-to-Sim mode, mirror the six arm joints immediately,
        # as Seeed's official receiver does.  Also update their drive targets
        # to the same pose so a packet timeout holds rather than rebounds.
        # The gripper is intentionally never teleported: its PR #21-derived
        # drives continue to hold both fingers physically against gravity.
        if self.arm_control_mode == "direct":
            self.robot.set_joint_positions(
                self.command_arm,
                joint_indices=self.arm_indices,
            )
            self.robot.set_joint_velocities(
                np.zeros(len(self.arm_indices), dtype=np.float64),
                joint_indices=self.arm_indices,
            )
        positions = np.concatenate(
            (self.command_arm, self._gripper_target_positions())
        )
        joint_indices = np.concatenate(
            (self.arm_indices, self.gripper_indices)
        )
        self.controller.apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_indices=joint_indices,
            )
        )

    def _update_gripper_contact_lock(self) -> None:
        """Stop both fingers at the first safe stationery obstruction."""
        if not self.contact_lock_enabled:
            return
        assert self.robot is not None
        assert self.gripper_indices is not None

        actual = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        actual_mean = float(np.mean(actual))

        if self.gripper_contact_locked:
            # Opening the leader is an explicit request to release the object.
            if self.command_gripper >= self.contact_lock_release:
                self.gripper_contact_locked = False
                print("[CONTACT-UNLOCK] The leader is opening the gripper; contact lock released")
            return

        # Only lock during closing, after the fingers have entered the known
        # stationery contact band. Empty-air closing remains unrestricted.
        if self.command_gripper >= actual_mean - 1e-5:
            return
        if actual_mean > self.contact_lock_opening:
            return
        distance = self._grasp_distance()
        self.min_grasp_distance = min(self.min_grasp_distance, distance)
        if distance > self.assist_distance:
            return

        self.gripper_contact_target = actual.copy()
        self.gripper_contact_locked = True
        print(
            f"[CONTACT-LOCK] {self.active_target_name} is between the fingers; locking at "
            f"{actual.round(5).tolist()}m. Open the leader gripper to release"
        )

    def _run_gripper_hold_test(self) -> None:
        """Measure steady-state gravity drift without direct state writes."""
        assert self.world is not None
        assert self.robot is not None
        assert self.active_target is not None
        assert self.gripper_indices is not None

        # Preserve the completed grasp result, then remove the object contact
        # load. Leaving its collider between the fingers would measure an
        # artificial penetration force rather than gravity-induced jaw drift.
        self.test_grasp_attached = self.grasp_attached
        object_position, object_orientation = self.active_target.get_world_pose()
        self.test_object_lift = (
            float(object_position[2])
            - self.initial_target_z.get(self.active_target_name, 0.0)
        )
        attached_actual = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        self.test_attached_gripper_error = float(
            np.max(
                np.abs(
                    attached_actual - self._gripper_target_positions()
                )
            )
        )
        print(
            "[test-gripper-attached] "
            f"target={self._gripper_target_positions().round(6).tolist()}m, "
            f"actual={attached_actual.round(6).tolist()}m, "
            f"max_error={self.test_attached_gripper_error * 1000:.4f}mm"
        )
        self._detach_target(silent=True)
        self.active_target.set_world_pose(
            position=np.asarray([1.5, 0.0, 0.5], dtype=np.float32),
            orientation=object_orientation,
        )

        # Test mid-stroke so either direction of gravity-induced motion is
        # observable instead of being hidden by an end stop.
        self.command_gripper = 0.5 * float(np.min(self.gripper_upper))
        target = self._gripper_target_positions()
        errors: list[float] = []
        asymmetries: list[float] = []
        actual = np.full(2, np.nan, dtype=np.float64)
        for step in range(240):
            self._apply_targets()
            self.world.step(render=False)
            if step < 120:
                continue
            actual = np.asarray(
                self.robot.get_joint_positions(
                    joint_indices=self.gripper_indices
                ),
                dtype=np.float64,
            )
            errors.append(float(np.max(np.abs(actual - target))))
            asymmetries.append(float(abs(actual[0] - actual[1])))

        self.test_gripper_hold_error = max(errors)
        self.test_gripper_asymmetry = max(asymmetries)
        print(
            "[test-gripper-hold] "
            f"target={target.round(6).tolist()}m, "
            f"actual={actual.round(6).tolist()}m, "
            f"max_gravity_drift={self.test_gripper_hold_error * 1000:.4f}mm, "
            f"max_left_right_error={self.test_gripper_asymmetry * 1000:.4f}mm"
        )

    def _assist_body_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        assert self.robot is not None
        assert self.assist_link_index is not None
        physics_view = self.robot._articulation_view._physics_view
        link_transforms = physics_view.get_link_transforms()
        if hasattr(link_transforms, "numpy"):
            link_transforms = link_transforms.numpy()
        link_pose = np.asarray(link_transforms, dtype=np.float64)[
            0, self.assist_link_index
        ]
        body_position = link_pose[:3]
        # PhysX tensors store quaternions as [x, y, z, w].
        body_orientation = np.asarray(
            [link_pose[6], link_pose[3], link_pose[4], link_pose[5]],
            dtype=np.float64,
        )
        return body_position, _quaternion_normalize(body_orientation)

    def _grasp_distance(self) -> float:
        body_position, body_orientation = self._assist_body_world_pose()
        # A single point at link6 is too sensitive to CAD and fingertip-length
        # calibration errors. Sample a short segment along the gripper's local
        # forward axis so the assist volume covers the space between the finger
        # roots and tips without extending sideways across the workspace.
        axial_offsets = np.linspace(
            -self.assist_region_half_length,
            self.assist_region_half_length,
            self.assist_region_samples,
            dtype=np.float64,
        )
        grasp_positions = [
            body_position
            + _quaternion_rotate(
                body_orientation,
                self.assist_offset
                + np.asarray([0.0, 0.0, axial_offset], dtype=np.float64),
            )
            for axial_offset in axial_offsets
        ]

        def target_distance(name: str) -> float:
            target_position, target_orientation = self.targets[
                name
            ].get_world_pose()
            target_position = np.asarray(target_position, dtype=np.float64)
            orientation = _quaternion_normalize(target_orientation)
            geometry = TARGET_GRASP_GEOMETRY[name]
            distances: list[float] = []
            for grasp_position in grasp_positions:
                local_point = _quaternion_rotate(
                    _quaternion_conjugate(orientation),
                    grasp_position - target_position,
                )
                if "half_extents" in geometry:
                    half_extents = np.asarray(
                        geometry["half_extents"], dtype=np.float64
                    )
                    outside = np.maximum(
                        np.abs(local_point) - half_extents, 0.0
                    )
                    distances.append(float(np.linalg.norm(outside)))
                    continue
                half_length = float(geometry["half_length"])
                radius = float(geometry["radius"])
                axial_excess = max(
                    abs(float(local_point[0])) - half_length, 0.0
                )
                radial_excess = max(
                    float(np.linalg.norm(local_point[1:])) - radius, 0.0
                )
                distances.append(
                    float(np.hypot(axial_excess, radial_excess))
                )
            return min(distances)

        if not self.grasp_attached and self.release_collision_pending_steps <= 0:
            nearest_name = min(self.targets, key=target_distance)
            self._set_active_target(nearest_name)
        assert self.active_target_name is not None
        return target_distance(self.active_target_name)

    def _attach_target(self, distance: float) -> None:
        # Preserve the exact pose at the instant of grasp. The previous fixed
        # local frame forced every randomized object into one preset rotation,
        # which created hidden angular constraint stress and a release impulse.
        body_position, body_orientation = self._assist_body_world_pose()
        assert self.active_target is not None
        assert self.active_target_path is not None
        target_position, target_orientation = self.active_target.get_world_pose()
        target_position = np.asarray(target_position, dtype=np.float64)
        target_orientation = _quaternion_normalize(target_orientation)
        body_inverse = _quaternion_conjugate(body_orientation)
        relative_position = _quaternion_rotate(
            body_inverse, target_position - body_position
        )
        relative_orientation = _quaternion_multiply(
            body_inverse, target_orientation
        )
        self.attached_relative_position = relative_position.copy()
        self.attached_relative_orientation = relative_orientation.copy()

        stage = get_current_stage()
        joint_prim = stage.GetPrimAtPath(self.assist_joint_path)
        fixed_joint = UsdPhysics.FixedJoint(joint_prim)
        fixed_joint.GetBody1Rel().SetTargets(
            [Sdf.Path(self.active_target_path)]
        )
        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*relative_position))
        fixed_joint.GetLocalRot0Attr().Set(
            Gf.Quatf(
                float(relative_orientation[0]),
                Gf.Vec3f(*relative_orientation[1:]),
            )
        )
        fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0))
        # Filter finger contacts while assisted, but retain table/rack
        # contact. The joint frame above is kept as diagnostic scene state;
        # it is intentionally not enabled in direct arm-control mode.
        self._set_target_collision_enabled(True)
        self._set_target_gripper_contacts_enabled(False)
        # A dynamic fixed joint would repeatedly chase the directly mirrored
        # arm links and accumulate solver velocity. Pose carry below writes
        # the measured relative transform directly and stores no constraint
        # stress. Make the body dynamic before touching velocities: PhysX
        # rejects velocity writes on a kinematic body.
        self._set_target_kinematic(False)
        self.active_target.set_linear_velocity(np.zeros(3, dtype=np.float64))
        self.active_target.set_angular_velocity(np.zeros(3, dtype=np.float64))
        self.grasp_attached = True
        self.release_collision_pending_steps = 0
        self.episode_grasped = True
        print(
            f"[GRASP] Stress-free attachment created for {self.active_target_name}: "
            f"gripper distance {distance:.3f}m; preserving the current randomized pose. "
            "Open the leader gripper to release"
        )

    def _update_attached_target_pose(self) -> None:
        """Carry an assisted grasp without a stressed solver constraint."""
        if not self.grasp_attached:
            return
        body_position, body_orientation = self._assist_body_world_pose()
        target_position = body_position + _quaternion_rotate(
            body_orientation, self.attached_relative_position
        )
        target_orientation = _quaternion_multiply(
            body_orientation, self.attached_relative_orientation
        )
        self.active_target.set_world_pose(
            position=target_position,
            orientation=target_orientation,
        )
        self.active_target.set_linear_velocity(np.zeros(3, dtype=np.float64))
        self.active_target.set_angular_velocity(np.zeros(3, dtype=np.float64))

    def _release_target(self) -> None:
        self._detach_target(silent=False, defer_collision=True)

    def _update_grasp_assist(self) -> None:
        if not self.assist_enabled:
            return
        if self.grasp_attached:
            actual = np.asarray(
                self.robot.get_joint_positions(
                    joint_indices=self.gripper_indices
                ),
                dtype=np.float64,
            )
            if (
                self.command_gripper >= self.assist_release
                and float(np.mean(actual)) >= self.assist_release
            ):
                self._release_target()
            return
        if self.command_gripper > self.assist_close:
            return
        distance = self._grasp_distance()
        self.min_grasp_distance = min(self.min_grasp_distance, distance)
        if distance > self.assist_distance:
            return

        actual = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        actual_mean = float(np.mean(actual))
        if actual_mean > self.assist_activation_opening:
            return

        if self.contact_lock_enabled and not self.gripper_contact_locked:
            if not self.allow_contact_lock_fallback:
                return
            # Thin objects may not create enough drive lag to satisfy the
            # directional contact-lock detector. Once the leader command and
            # the measured fingers are both closed in the near-field region,
            # freeze the measured opening and attach directly.
            self.gripper_contact_target = actual.copy()
            self.gripper_contact_locked = True
            print(
                f"[CONTACT-LOCK-FALLBACK] {self.active_target_name} is inside "
                f"the grasp corridor; locking at {actual.round(5).tolist()}m"
            )

        self._attach_target(distance)

    def _validate_test(self) -> bool:
        assert self.robot is not None
        assert self.arm_indices is not None
        actual = self.robot.get_joint_positions(joint_indices=self.arm_indices)
        command_error = float(
            np.max(np.abs(self.command_arm - self.latest_received_arm))
        )
        tracking_error = float(np.max(np.abs(actual - self.command_arm)))
        actual_gripper = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=self.gripper_indices
            ),
            dtype=np.float64,
        )
        gripper_error = float(
            np.max(
                np.abs(
                    actual_gripper - self._gripper_target_positions()
                )
            )
        )
        gripper_opened = bool(
            np.all(self.max_observed_gripper >= np.asarray([0.045, 0.045]))
        )
        object_lift = self.test_object_lift
        success = (
            self.packet_count >= ARGS.test_packets
            and command_error < np.radians(2.0)
            and tracking_error < np.radians(20.0)
            and gripper_error < 0.001
            and gripper_opened
            and self.test_gripper_hold_error < 0.0005
            and self.test_gripper_asymmetry < 0.0001
            and self.test_attached_gripper_error < 0.0005
            and self.test_grasp_attached
            and object_lift >= 0.06
            and self.test_recording_ok
        )
        label = "SUCCESS" if success else "FAILED"
        print(
            f"[{label}] packets={self.packet_count}, "
            f"command_error={np.degrees(command_error):.2f}deg, "
            f"tracking_error={np.degrees(tracking_error):.2f}deg, "
            f"gripper_error={gripper_error:.5f}m, "
            f"gripper_opened={gripper_opened}, "
            f"gravity_drift={self.test_gripper_hold_error * 1000:.4f}mm, "
            f"finger_asymmetry={self.test_gripper_asymmetry * 1000:.4f}mm, "
            f"attached_gripper_error="
            f"{self.test_attached_gripper_error * 1000:.4f}mm, "
            f"grasp_attached={self.test_grasp_attached}, "
            f"object_lift={object_lift:.4f}m, "
            f"min_grasp_distance={self.min_grasp_distance:.4f}m, "
            f"dataset_ok={self.test_recording_ok}"
        )
        return success

    def run(self) -> bool:
        assert self.world is not None
        last_report = time.monotonic()
        while simulation_app.is_running():
            loop_start = time.monotonic()
            if self.recorder is not None:
                self.recorder.poll()
            if self.bridge_process is not None:
                return_code = self.bridge_process.poll()
                if return_code is not None:
                    if ARGS.test and self.packet_count >= ARGS.test_packets:
                        break
                    raise RuntimeError(
                        f"Leader bridge exited with code {return_code}"
                    )

            latest = self._recv_latest()
            if latest is not None:
                self._update_command(*latest)
                self._watchdog_reported = False
            elif (
                self.last_packet_monotonic > 0.0
                and time.monotonic() - self.last_packet_monotonic
                > self.watchdog_timeout
                and not self._watchdog_reported
            ):
                print("[watchdog] Leader data timed out; the simulated arm is holding its last target")
                self._watchdog_reported = True

            self._handle_keyboard()
            if (
                ARGS.test_recording
                and not self.test_record_started
                and self.packet_count >= 20
            ):
                self._start_recording()
                self.test_record_started = bool(
                    self.recorder is not None
                    and self.recorder.recording
                )

            self._update_gripper_contact_lock()
            self._apply_targets()
            self._update_grasp_assist()
            self._update_attached_target_pose()
            render = self.render_frames or bool(
                self.recorder is not None and self.recorder.recording
            )
            self.world.step(render=render)
            # A dynamic body receives gravity during world.step. Re-apply the
            # carried pose afterward so rendering/recording and the next step
            # see the exact grasp transform. On release grasp_attached is
            # already false, so this stops immediately and gravity takes over.
            self._update_attached_target_pose()
            self.sim_elapsed += self.physics_dt
            self._update_release_transition()
            self._update_episode_metrics()
            self._capture_record_frame()

            if (
                ARGS.test_recording
                and self.recorder is not None
                and self.recorder.recording
                and self.packet_count >= 190
            ):
                self._stop_recording()

            now = time.monotonic()
            if now - last_report >= 1.0:
                status = (
                    "LIVE"
                    if self.last_packet_monotonic > 0.0
                    and now - self.last_packet_monotonic
                    <= self.watchdog_timeout
                    else "HOLD"
                )
                q_deg = np.degrees(self.command_arm).round(1).tolist()
                actual_gripper = np.asarray(
                    self.robot.get_joint_positions(
                        joint_indices=self.gripper_indices
                    ),
                    dtype=np.float64,
                )
                grasp_debug = ""
                if self.assist_enabled and not self.grasp_attached:
                    nearest_distance = self._grasp_distance()
                    grasp_debug = (
                        f" nearest={self.active_target_name} "
                        f"distance={nearest_distance:.3f}m"
                    )
                record_status = (
                    " REC"
                    if self.recorder is not None
                    and self.recorder.recording
                    else ""
                )
                print(
                    f"[teleop:{status}{record_status}] "
                    f"packets={self.packet_count} "
                    f"q_deg={q_deg} "
                    f"gripper_cmd={self.command_gripper:.4f}m "
                    f"gripper_actual={float(np.mean(actual_gripper)):.4f}m "
                    f"lock={self.gripper_contact_locked} "
                    f"attached={self.grasp_attached}"
                    f"{grasp_debug}"
                )
                last_report = now

            if ARGS.test and self.packet_count >= ARGS.test_packets:
                for _ in range(30):
                    self._apply_targets()
                    self._update_attached_target_pose()
                    self.world.step(render=False)
                    self._update_attached_target_pose()
                self._run_gripper_hold_test()
                break

            elapsed = time.monotonic() - loop_start
            if not render:
                time.sleep(max(0.0, self.physics_dt - elapsed))

        if ARGS.test_recording and self.recorder is not None:
            if self.recorder.recording:
                self._stop_recording()
            writer_ok = self.recorder.finish()
            videos_path = self.dataset_root / "videos"
            dataset_ok = (
                (self.dataset_root / "meta" / "info.json").is_file()
                and videos_path.is_dir()
                and len(list(videos_path.rglob("*.mp4"))) >= 2
            )
            camera_quality = self.recorder.last_image_std
            camera_edges = self.recorder.last_image_edge_std
            images_ok = (
                camera_edges["front"] >= 8.0
                and camera_edges["external"] >= 8.0
            )
            self.test_recording_ok = writer_ok and dataset_ok and images_ok
            print(
                f"[test-dataset] writer_ok={writer_ok}, "
                f"dataset_ok={dataset_ok}, images_ok={images_ok}, "
                f"rgb_std={camera_quality}, edge_std={camera_edges}, "
                f"root={self.dataset_root}"
            )
        return self._validate_test() if ARGS.test else True

    def shutdown(self) -> None:
        self.keyboard.cleanup()
        if self.side_viewport is not None:
            try:
                self.side_viewport.destroy()
            except Exception:
                pass
            self.side_viewport = None
        if self.front_viewport is not None:
            try:
                self.front_viewport.destroy()
            except Exception:
                pass
            self.front_viewport = None
        if self.external_viewport is not None:
            try:
                self.external_viewport.destroy()
            except Exception:
                pass
            self.external_viewport = None
        if self.recorder is not None:
            if self.recorder.recording:
                self.recorder.cancel()
            if self.recorder.pending or self.recorder.active_process is not None:
                print("[dataset] Waiting for background episode writes to finish...")
                if not self.recorder.finish():
                    print(
                        f"[dataset-error] {self.recorder.last_error}; "
                        "Temporary files were preserved"
                    )
        if self.input_socket is not None:
            self.input_socket.close()
            self.input_socket = None
        if self.bridge_process is not None and self.bridge_process.poll() is None:
            self.bridge_process.terminate()
            try:
                self.bridge_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.bridge_process.kill()
                self.bridge_process.wait(timeout=2.0)
        self.bridge_process = None


def main() -> int:
    teleop = LeaderTeleop()
    try:
        teleop.setup()
        return 0 if teleop.run() else 2
    finally:
        teleop.shutdown()


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("\n[stop] Interrupted by user")
        exit_code = 130
    except Exception as exc:
        print(f"[error] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
