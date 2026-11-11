#!/usr/bin/env python3
"""Mirror an SO-101 leader (and, when linked, the real SO-101 follower) into the white SO-ARM101 vials scene.

This is the SO-101 counterpart of ``leader_teleop.py`` and shares its
architecture: a LeRobot bridge process reads the leader (and optionally
drives the real follower arm) and streams joint targets over UDP; this
application mirrors them onto the simulated SO-ARM101 inside the same
enclosure and task as the reBot scene.  The reBot files are untouched.
"""

from __future__ import annotations

import argparse
import json
import math
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
DEFAULT_TELEOP_CONFIG = PROJECT_ROOT / "config" / "teleop_config_so101.json"
DEFAULT_GRASP_CONFIG = PROJECT_ROOT / "config" / "grasp_config_so101.json"
DEFAULT_DR_CONFIG = PROJECT_ROOT / "config" / "domain_randomization_so101.json"
DEFAULT_SCENE = PROJECT_ROOT / "scenes" / "so101_vials_sorting.usda"
BRIDGE_SCRIPT = PROJECT_ROOT / "scripts" / "so101_leader_bridge.py"
DATASET_WRITER_SCRIPT = PROJECT_ROOT / "scripts" / "dataset_writer_so101.py"
# Jaw openness (raw leader units, 0..100) above which the gripper counts as
# released for the episode success detector.
RELEASE_OPENNESS_RAW = 40.0

# Keyboard teleoperation: press K to toggle, then drive the arm from the
# viewport keyboard.  Held keys auto-repeat, so each press nudges one step.
KEYBOARD_TOGGLE_KEY = "K"
KEYBOARD_SPEED_KEY = "B"
# Keyboard teleoperation (inverse kinematics): press K to toggle, then
# move the END EFFECTOR in Cartesian space — WASD forward/back/left/right,
# arrow Up/Down raises/lowers it.  The four arm joints (Rotation, Pitch,
# Elbow, Wrist_Pitch) are solved per press with damped least squares while
# holding the tool tilt; Q/E roll the wrist and F toggles the jaw.
# S means "backward" in keyboard mode, so recording uses T there
# (S records in leader mode, as in the reBot app).
# Y toggles keyboard mode: plain K collides with an Isaac Sim GUI hotkey,
# and W/E/R are the viewport gizmo tools (F frames the selection), so none
# of those are safe as the mode switch.
KEYBOARD_TOGGLE_KEY = "Y"
KEYBOARD_SPEED_KEY = "B"
KEYBOARD_RECORD_KEY = "T"
KEYBOARD_CARTESIAN_KEYS = {
    "W": (0.0, -1.0, 0.0),   # forward (into the workspace)
    "S": (0.0, +1.0, 0.0),   # backward (toward the robot)
    "A": (+1.0, 0.0, 0.0),   # left  (+X: the robot's and the viewer's left)
    "D": (-1.0, 0.0, 0.0),   # right (-X)
    "UP": (0.0, 0.0, +1.0),
    "DOWN": (0.0, 0.0, -1.0),
}
KEYBOARD_ROLL_STEP_DEG = 3.0
KEYBOARD_PITCH_STEP_DEG = 3.0
KEYBOARD_SLOW_STEP_M = 0.005
KEYBOARD_FAST_STEP_M = 0.015
KEYBOARD_JAW_OPEN_DEG = 70.0
KEYBOARD_JAW_CLOSED_DEG = 0.0
# G toggles the jaw (not F: Isaac Sim's F frames the selected prim, which
# yanks the viewport camera whenever anything is selected).
KEYBOARD_JAW_TOGGLE_KEY = "G"
KEYBOARD_CONTROL_KEYS = {
    "S", "R",
    KEYBOARD_TOGGLE_KEY,
    KEYBOARD_SPEED_KEY,
    KEYBOARD_RECORD_KEY,
    *KEYBOARD_CARTESIAN_KEYS.keys(),
    "Q", "E", "Z", "C",
    KEYBOARD_JAW_TOGGLE_KEY,
}
# FK chain: the five joints from base to gripper (Wrist_Roll spins about
# the tool axis and is excluded from the position IK).
IK_JOINT_INDICES = (0, 1, 2, 3)
IK_TOOL_TCP_OFFSET_M = 0.07

# Near-field grasp assist, modelled on the reBot application's assist: when
# the leader closes the gripper with a vial inside the finger corridor, the
# vial is attached to the gripper link with its measured relative pose and
# carried kinematically (pure contact alone lets the vial tunnel through
# the fingers and fly away); opening the gripper releases it.  The corridor
# runs along the gripper link's local -Z, which points from the wrist
# toward the fingers.
ASSIST_CORRIDOR_OFFSETS_M = (0.03, 0.055, 0.08, 0.105)
ASSIST_CORRIDOR_RADIUS_M = 0.045
ASSIST_ATTACH_BELOW_RAW = 30.0
ASSIST_RELEASE_ABOVE_RAW = 60.0
GRIPPER_COLLIDER_PATHS = (
    "/World/Robot/gripper/collisions",
    "/World/Robot/jaw/collisions",
)


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
# Mock-regression tolerances after the settle window, in degrees.
TEST_TRACKING_TOLERANCE_DEG = 4.0


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
    parser.add_argument("--leader-port", help="SO-101 leader serial port, for example /dev/ttyUSB0")
    parser.add_argument(
        "--follower-port",
        help="Real SO-101 follower serial port; enables real+sim linked teleoperation",
    )
    parser.add_argument(
        "--no-follower",
        action="store_true",
        help="Sim-only teleoperation even when a follower port is configured",
    )
    parser.add_argument("--leader-python", type=Path)
    parser.add_argument(
        "--no-start-bridge",
        action="store_true",
        help="Start only the Isaac receiver; run so101_leader_bridge.py in another terminal",
    )
    parser.add_argument("--rebuild-scene", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_TELEOP_CONFIG)
    parser.add_argument("--grasp-config", type=Path, default=DEFAULT_GRASP_CONFIG)
    parser.add_argument("--dr-config", type=Path, default=DEFAULT_DR_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--no-dr",
        action="store_true",
        help="Use a fixed scene; R still resets the three vials without randomization",
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
    parser.add_argument(
        "--realtime-renderer",
        action="store_true",
        help="Render with Real-Time instead of the scene's Path Tracing: much lighter GPU load, the built-in freeze mitigation for 8 GB laptop GPUs (TUTORIAL section 10)",
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
    parser.add_argument(
        "--auto-record",
        type=int,
        default=0,
        metavar="EPISODES",
        help="Headless scripted collection: record EPISODES episodes of the "
        "mock timeline into the configured dataset root, then exit (use "
        "with --headless; no keyboard or leader needed)",
    )
    args = parser.parse_args()
    if args.test_recording:
        args.test = True
    return args


ARGS = _parse_args()

try:
    from isaacsim import SimulationApp
except ImportError as exc:
    raise RuntimeError(
        "Run with the Isaac Sim python.sh or use the project's ./run_so101.sh"
    ) from exc

simulation_app = SimulationApp({"headless": ARGS.headless or ARGS.test})

if not (ARGS.headless or ARGS.test):
    # Throttle the loop to the 60 Hz leader stream so laptop GPUs keep
    # headroom for the desktop compositor (see TUTORIAL §10).
    import carb

    _settings = carb.settings.get_settings()
    _settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
    _settings.set_int("/app/runLoops/main/rateLimitFrequency", 30)
    # Real-Time (not Path Tracing) on 8 GB laptop GPUs
    _settings.set_string("/rtx/rendermode", "RealTimePathTracing")
    # Aggressive quality reduction: the desktop compositor already takes
    # ~2.8 GB of the 8 GB, so the renderer gets minimal resources.
    _settings.set_int("/rtx/directLighting/sampledLighting/samplesPerPixel", 1)
    _settings.set_int("/rtx/reflections/sampledLighting/samplesPerPixel", 1)
    _settings.set_int("/rtx/ambientOcclusion/enabled", 0)
    _settings.set_bool("/rtx/post/aa/op", False)
    _settings.set_int("/rtx/post/bloom/enabled", 0)
    _settings.set_int("/rtx/post/dof/enabled", 0)
    _settings.set_int("/rtx/post/motionBlur/enabled", 0)
    _settings.set_int("/rtx/post/chromaticAberration/enabled", 0)
    _settings.set_int("/app/renderer/resolution/width", 640)
    _settings.set_int("/app/renderer/resolution/height", 360)

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import (
    get_current_stage,
    is_stage_loading,
    open_stage,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera

from build_scene_so101 import (
    SO101_POSITION,
    build_scene_so101,
    so101_scene_is_configured,
)
from domain_randomization import DomainRandomizer
from episode_recorder import EpisodeRecorder
from so101_protocol import (
    SO101_LEADER_JOINTS,
    SO101_SIM_JOINTS,
    decode_payload,
    jaw_openness,
    rad_to_raw,
)

import carb
import omni.appwindow


def _preview_guide_material(stage, path: str, color) -> object:
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        __import__("pxr").Gf.Vec3f(*color)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _set_guide_transform(prim, position, orientation, length) -> None:
    from pxr import Gf, Usd, UsdGeom

    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    if not ops or len(ops) < 2:
        xform.ClearXformOpOrder()
        translate = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        orient = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        scale = xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
    else:
        translate, orient, scale = ops[0], ops[1], ops[-1]
    translate.Set(Gf.Vec3d(*position))
    orient.Set(
        Gf.Quatd(
            float(orientation[0]),
            Gf.Vec3d(*[float(v) for v in orientation[1:4]]),
        )
    )
    scale.Set(Gf.Vec3d(1.0, 1.0, float(length)))


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
        if key in KEYBOARD_CONTROL_KEYS:
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


class SO101Teleop:
    def __init__(self) -> None:
        self.config_path = ARGS.config.resolve()
        self.grasp_config_path = ARGS.grasp_config.resolve()
        self.dr_config_path = ARGS.dr_config.resolve()
        self.scene_path = ARGS.scene.resolve()
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.grasp_config = json.loads(
            self.grasp_config_path.read_text(encoding="utf-8")
        )
        self.dr_config = json.loads(self.dr_config_path.read_text(encoding="utf-8"))

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
        self.jaw_alpha = float(command_filter.get("jaw_alpha", 1.0))
        self.max_jaw_step = np.radians(
            float(command_filter.get("max_jaw_step_deg", 25.0))
        )
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("command_filter.alpha must be in (0, 1]")
        if self.arm_control_mode not in {"direct", "drive"}:
            raise ValueError("command_filter.mode must be either direct or drive")
        if not 0.0 < self.fast_filter_alpha <= 1.0:
            raise ValueError("command_filter.fast_alpha must be in (0, 1]")
        if not 0.0 < self.jaw_alpha <= 1.0:
            raise ValueError("command_filter.jaw_alpha must be in (0, 1]")

        self.physics_dt = float(self.grasp_config["physics_dt"])
        self.rendering_dt = float(self.grasp_config["rendering_dt"])
        self.robot_path = str(self.grasp_config["robot_prim_path"])
        self.target_paths = {
            str(name): str(path)
            for name, path in self.grasp_config["target_prim_paths"].items()
        }
        self.rack_path = str(self.grasp_config["vial_rack_prim_path"])
        rack_cfg = self.grasp_config["vial_rack"]
        self.rack_center_xy = np.asarray(
            rack_cfg["center_xy_m"], dtype=np.float64
        )
        self.rack_success_radius = float(rack_cfg["success_radius_m"])
        self.rack_success_min_z = float(rack_cfg["success_min_z_m"])
        self.rack_success_max_z = float(rack_cfg["success_max_z_m"])
        self.rack_success_max_speed = float(rack_cfg["success_max_speed_mps"])
        self.rack_success_settle_time = float(rack_cfg["success_settle_time_s"])

        self.sim_source = str(
            self.config.get("follower", {}).get("sim_source", "follower")
        )
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
                tempfile.mkdtemp(prefix="so101_vials_dataset_", dir="/tmp")
            )
            self.dataset_root = test_parent / "dataset"
        self.repo_id = ARGS.repo_id or str(recording_cfg["repo_id"])
        self.task_name = ARGS.task_name or str(recording_cfg["task_name"])
        self.dr_enabled = bool(self.dr_config["enabled"]) and not ARGS.no_dr
        self.dr_seed = (
            int(ARGS.seed)
            if ARGS.seed is not None
            else (227 if ARGS.test else int(self.dr_config["seed"]))
        )
        self.render_frames = (
            not (ARGS.headless or ARGS.test) or ARGS.test_recording
        )

        self.world: World | None = None
        self.robot: SingleArticulation | None = None
        self.targets: dict[str, SingleRigidPrim] = {}
        self.controller = None
        # Five arm joints plus the Jaw, all in radians, ordered like
        # SO101_SIM_JOINTS.
        self.arm_indices: np.ndarray | None = None
        self.jaw_index: int | None = None
        self.arm_lower = np.zeros(5, dtype=np.float64)
        self.arm_upper = np.zeros(5, dtype=np.float64)
        self.jaw_lower = 0.0
        self.jaw_upper = 0.0
        # Commands start at the USD zero pose, matching the scene's
        # authored drive targets, so the arm waits flat until the first
        # packet arrives.
        self.command_joints = np.zeros(len(SO101_SIM_JOINTS), dtype=np.float64)
        self.latest_received_joints = self.command_joints.copy()
        self.latest_leader_raw = np.asarray(
            [rad_to_raw(name, value) for name, value in zip(
                SO101_LEADER_JOINTS, self.command_joints
            )],
            dtype=np.float64,
        )
        self.latest_source = "init"
        self.assist_link_index: int | None = None
        self.attached_target: SingleRigidPrim | None = None
        self.attached_target_name: str | None = None
        self.attached_target_path: str | None = None
        self.attached_relative_position = np.zeros(3, dtype=np.float64)
        self.attached_relative_orientation = np.asarray(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )

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
        self._test_packet_queue: deque[
            tuple[np.ndarray, np.ndarray | None, np.ndarray, int]
        ] = deque()
        self._test_consume_countdown = 2
        self._watchdog_reported = False
        self.initial_target_z: dict[str, float] = {}
        self.max_object_lift = 0.0
        self.episode_success = False
        self.rack_settle_elapsed = 0.0
        self.objects_in_rack: set[str] = set()
        self.current_randomization: dict = {}
        self.sim_elapsed = 0.0
        self.next_record_time = 0.0
        self._physics_broken_reported = False
        self._auto_record_remaining = int(ARGS.auto_record or 0)
        self._auto_record_phase = "reset" if self._auto_record_remaining else "off"
        self._auto_record_deadline = 0.0
        self.keyboard_mode = False
        self.keyboard_fast = False
        self._keyboard_jaw_open = True
        self._ik_frames = []
        self._ik_base_pos = np.zeros(3)
        self._ik_base_quat = np.asarray([1.0, 0.0, 0.0, 0.0])
        self._ik_gripper_local = np.zeros(3)
        self._ik_validated = False
        self._guide_line_path: str | None = None
        self._guide_marker_path: str | None = None
        self._guide_green = None
        self._guide_orange = None
        self._nearest_vial_name: str | None = None
        self._nearest_vial_distance = float("inf")
        self.test_record_started = False
        self.test_recording_ok = not ARGS.test_recording
        self.test_tracking_error_deg = float("inf")

    @staticmethod
    def _wait_for_stage() -> None:
        while is_stage_loading():
            simulation_app.update()
        for _ in range(3):
            simulation_app.update()

    def setup(self) -> None:
        if ARGS.rebuild_scene or not self.scene_path.exists():
            # Path Tracing keeps the recorded camera images free of the
            # Real-Time speckle on GPUs without DLSS-RR; see TUTORIAL §10.
            build_scene_so101(self.scene_path, render_mode="RealTimePathTracing")
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
            self.rack_path,
            str(self.grasp_config["workspace_prim_path"]),
            *self.target_paths.values(),
        ]
        missing_scene_content = any(
            not stage.GetPrimAtPath(path).IsValid()
            for path in required_scene_prims
        )
        missing_so101_setup = not so101_scene_is_configured(stage)
        if missing_scene_content or missing_so101_setup:
            reasons = []
            if missing_scene_content:
                reasons.append("cameras, table, vials, or the fixed vial rack")
            if missing_so101_setup:
                reasons.append(
                    "the white SO-ARM101 articulation, drives, and wrist camera"
                )
            print(
                "[scene] An outdated scene was detected; rebuilding "
                + "、".join(reasons)
            )
            build_scene_so101(self.scene_path, render_mode="RealTimePathTracing")
            if not open_stage(str(self.scene_path)):
                raise RuntimeError(f"Unable to reopen scene: {self.scene_path}")
            self._wait_for_stage()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=self.physics_dt,
            rendering_dt=self.rendering_dt,
        )
        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=self.robot_path, name="so101_teleop")
        )
        self.targets = {
            name: SingleRigidPrim(prim_path=path, name=f"target_{name}")
            for name, path in self.target_paths.items()
        }
        self.world.reset()
        self.robot.initialize()
        for target in self.targets.values():
            target.initialize()
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
        # Sensor render products default to Real-Time when the stage lacks
        # the /Render settings stack; without DLSS-RR that renders as
        # one-sample speckle.  Force the renderer mode on the products
        # themselves: PathTracing normally, RayTracedLighting when the
        # freeze-mitigation flag is passed.
        from pxr import Sdf

        # "RealTimePathTracing" is this build's RTX Realtime 2.0 token;
        # "RayTracedLighting" is the unsupported RT1 name and logs a
        # fallback warning on every launch.
        product_mode = (
            "RealTimePathTracing" if ARGS.realtime_renderer else "PathTracing"
        )
        for camera in (self.side_camera, self.front_camera, self.external_camera):
            product = get_current_stage().GetPrimAtPath(
                camera.get_render_product_path()
            )
            product.CreateAttribute(
                "omni:rtx:rendermode", Sdf.ValueTypeNames.Token
            ).Set(product_mode)

        dof_names = list(self.robot.dof_names)
        arm_names = list(self.grasp_config["arm_joint_names"])
        jaw_name = str(self.grasp_config["gripper_joint_names"][0])
        missing = [
            name for name in arm_names + [jaw_name] if name not in dof_names
        ]
        if missing:
            raise RuntimeError(
                f"Robot DOF mismatch; missing {missing}; available DOFs: {dof_names}"
            )
        self.arm_indices = np.asarray(
            [dof_names.index(name) for name in arm_names], dtype=np.int64
        )
        self.jaw_index = dof_names.index(jaw_name)
        properties = self.robot.dof_properties
        lower = np.asarray(properties["lower"], dtype=np.float64)
        upper = np.asarray(properties["upper"], dtype=np.float64)
        self.arm_lower = lower[self.arm_indices]
        self.arm_upper = upper[self.arm_indices]
        self.jaw_lower = float(lower[self.jaw_index])
        self.jaw_upper = float(upper[self.jaw_index])
        self.controller = self.robot.get_articulation_controller()
        self.assist_link_index = self.robot._articulation_view.get_link_index(
            "gripper"
        )

        # No autonomous home trajectory: hold the USD zero pose until
        # leader data arrives, exactly as the reBot application does.
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

        self._build_kinematics()
        self._validate_fk()

        if not (ARGS.headless or ARGS.test):
            # Depth/occlusion aid: a 3D guide line plus terminal distances.
            self._create_grasp_guide()

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
                        "Front Camera (SO-101 Wrist)",
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
                        "[camera] The main viewport remains an unrecorded debug "
                        "view; opened the side, wrist front, and overhead "
                        "external camera viewports"
                    )
            except Exception as exc:
                print(f"[camera-warn] Unable to configure camera viewports: {exc}")

        self._reset_episode()

        self._start_bridge()
        linkage = (
            "real follower + simulation (linked)"
            if self._follower_enabled()
            else "simulation only"
        )
        dr_mode = "enabled" if self.dr_enabled else "disabled (fixed scene)"
        print("=" * 76)
        print("  SO-101 Leader -> Isaac Sim SO-ARM101 (white) teleoperation")
        print(f"  Receiver: udp://{self.host}:{self.port}")
        print(f"  Linkage: {linkage}")
        print(f"  Mirror source: {self.sim_source} measurements when present")
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
        print(
            "  Safety: holds the current joint targets before data arrives "
            "and after a timeout"
        )
        print(
            "  Keyboard: CLICK THE 3D VIEWPORT once, then press Y to toggle"
            " leader/keyboard control; in keyboard mode WASD moves the"
            " gripper, arrows raise/lower it, Q/E roll the wrist,"
            " Z/C tilt the gripper (俯仰), G toggles the jaw, B toggles the"
            " step size (5/15 mm), T starts/stops recording (S is backward)."
        )
        print(f"  DOF : {dof_names}")
        print(
            f"  Jaw : {jaw_name} [{self.jaw_lower:.2f}, "
            f"{self.jaw_upper:.2f}] rad; position drive from the workshop USD"
        )
        print("=" * 76)

    def _follower_enabled(self) -> bool:
        follower_cfg = self.config.get("follower", {})
        if ARGS.no_follower or ARGS.no_start_bridge or ARGS.test:
            return False
        if ARGS.follower_port:
            return True
        return bool(follower_cfg.get("enabled", False))

    def _reset_episode(self) -> None:
        assert self.randomizer is not None
        assert self.robot is not None
        if self.recorder is not None and self.recorder.recording:
            self._stop_recording()
        self._detach_assist()
        self._apply_targets()
        self.current_randomization = self.randomizer.reset()
        # Let the vials fall onto the work surface and warm up cameras.
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
        self.episode_success = False
        self.rack_settle_elapsed = 0.0
        self.objects_in_rack = set()
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
        """Dataset action vector: the raw leader values, as lerobot records them."""
        return np.asarray(self.latest_leader_raw, dtype=np.float32)

    def _follower_state_vector(self) -> np.ndarray:
        """Dataset state vector: simulated joints expressed as leader values."""
        assert self.robot is not None
        joints = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=np.concatenate(
                    (self.arm_indices, [self.jaw_index])
                )
            ),
            dtype=np.float64,
        )
        return np.asarray(
            [
                rad_to_raw(name, value)
                for name, value in zip(SO101_LEADER_JOINTS, joints)
            ],
            dtype=np.float32,
        )

    def _start_recording(self) -> None:
        if self.recorder is None:
            print("[record-warn] Dataset recording is disabled")
            return
        if (
            not self.keyboard_mode
            and not ARGS.auto_record
            and (
                self.last_packet_monotonic <= 0.0
                or time.monotonic() - self.last_packet_monotonic
                > self.watchdog_timeout
            )
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
            grasped=self.max_object_lift > 0.02,
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
            if key == KEYBOARD_TOGGLE_KEY:
                self.keyboard_mode = not self.keyboard_mode
                state = (
                    "KEYBOARD (IK: WASD move the gripper, arrows up/down, "
                    "Q/E wrist roll, Z/C tool pitch, G jaw toggle, "
                    "B step size, T record; Y toggles back to leader)"
                    if self.keyboard_mode
                    else "LEADER"
                )
                print(f"[teleop] Control source: {state}")
                continue
            if key == KEYBOARD_SPEED_KEY:
                self.keyboard_fast = not self.keyboard_fast
                print(
                    f"[teleop] Cartesian step: "
                    f"{KEYBOARD_FAST_STEP_M * 1000 if self.keyboard_fast else KEYBOARD_SLOW_STEP_M * 1000:.0f} mm"
                )
                continue
            if key == "S" and not self.keyboard_mode:
                if self.recorder is not None and self.recorder.recording:
                    self._stop_recording()
                else:
                    self._start_recording()
            elif key == KEYBOARD_RECORD_KEY and self.keyboard_mode:
                if self.recorder is not None and self.recorder.recording:
                    self._stop_recording()
                else:
                    self._start_recording()
            elif key == "C" and not self.keyboard_mode:
                # Cancel recording (leader mode only; C is tool pitch
                # in keyboard mode and was being swallowed here).
                if self.recorder is not None:
                    self.recorder.cancel()
            elif key == "R":
                self._reset_episode()
            elif not self.keyboard_mode:
                continue
            elif key in KEYBOARD_CARTESIAN_KEYS and self._ik_validated:
                step = (
                    KEYBOARD_FAST_STEP_M
                    if self.keyboard_fast
                    else KEYBOARD_SLOW_STEP_M
                )
                self._apply_cartesian_step(
                    np.asarray(
                        KEYBOARD_CARTESIAN_KEYS[key], dtype=np.float64
                    ),
                    step,
                )
            elif key in ("Q", "E"):
                direction = -1.0 if key == "Q" else 1.0
                value = self.command_joints[4] + math.radians(
                    direction * KEYBOARD_ROLL_STEP_DEG
                )
                self.command_joints[4] = min(
                    float(self.arm_upper[4]),
                    max(float(self.arm_lower[4]), value),
                )
                self._refresh_leader_raw_from_command()
            elif key in ("Z", "C"):
                # Tool pitch (俯仰): C tilts the gripper down (低头),
                # Z tilts it up (抬头).  Positive Wrist_Pitch raises the
                # tool, so C subtracts and Z adds.
                direction = 1.0 if key == "Z" else -1.0
                value = self.command_joints[3] + math.radians(
                    direction * KEYBOARD_PITCH_STEP_DEG
                )
                self.command_joints[3] = min(
                    float(self.arm_upper[3]),
                    max(float(self.arm_lower[3]), value),
                )
                self._refresh_leader_raw_from_command()
            elif key == KEYBOARD_JAW_TOGGLE_KEY:
                self._keyboard_jaw_open = not self._keyboard_jaw_open
                target_deg = (
                    KEYBOARD_JAW_OPEN_DEG
                    if self._keyboard_jaw_open
                    else KEYBOARD_JAW_CLOSED_DEG
                )
                self.command_joints[5] = math.radians(target_deg)
                self._refresh_leader_raw_from_command()
                print(
                    f"[teleop] Jaw -> {'open' if self._keyboard_jaw_open else 'closed'}"
                )

    def _update_episode_metrics(self) -> None:
        assert self.robot is not None
        joints = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=np.concatenate(
                    (self.arm_indices, [self.jaw_index])
                )
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(joints)):
            if not self._physics_broken_reported:
                print(
                    "[physics] Non-finite joint values detected (the arm was "
                    "likely driven into the geometry). Press R to reset or "
                    "restart the app; statistics are paused."
                )
                self._physics_broken_reported = True
            return
        self._physics_broken_reported = False
        jaw_actual = float(joints[-1])
        in_rack: set[str] = set()
        all_stable = True
        for name, target in self.targets.items():
            position, _orientation = target.get_world_pose()
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
            height_ok = (
                self.rack_success_min_z
                <= float(position[2])
                <= self.rack_success_max_z
            )
            if distance <= self.rack_success_radius and height_ok:
                in_rack.add(name)
        if in_rack != self.objects_in_rack:
            self.objects_in_rack = in_rack
            print(
                f"[SORT] In the vial rack: "
                + (", ".join(sorted(in_rack)) if in_rack else "none")
                + f" ({len(in_rack)}/{len(self.targets)})"
            )
        released = jaw_openness(jaw_actual) * 100.0 >= RELEASE_OPENNESS_RAW
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
                "[SUCCESS] All vials are stable in the rack; settle time="
                f"{self.rack_settle_elapsed:.2f}s"
            )

    def _start_bridge(self) -> None:
        if ARGS.no_start_bridge:
            print("[teleop] Waiting for an external so101_leader_bridge.py process")
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
            command.extend(["--leader-port", ARGS.leader_port])
        if self._follower_enabled():
            if ARGS.follower_port:
                command.extend(["--follower-port", ARGS.follower_port])
        else:
            command.append("--no-follower")
        if ARGS.test or ARGS.auto_record:
            command.extend(["--mock", "--mock-profile", "grasp"])
        bridge_environment = os.environ.copy()
        # Isaac's python.sh injects its Python 3.10 standard library into
        # PYTHONPATH/PYTHONHOME. The leader runs in LeRobot's Python 3.12
        # venv, so inheriting those variables causes an immediate SRE ABI
        # mismatch.
        bridge_environment.pop("PYTHONHOME", None)
        bridge_environment.pop("PYTHONPATH", None)
        bridge_environment["VIRTUAL_ENV"] = str(leader_python.parent.parent)
        self.bridge_process = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
            env=bridge_environment,
        )

    def _recv_latest(self) -> tuple[np.ndarray, np.ndarray | None, np.ndarray] | None:
        assert self.input_socket is not None
        latest = None
        while True:
            try:
                packet, _address = self.input_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                decoded = decode_payload(packet)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                print(f"[udp-warn] Dropping an invalid packet: {exc}")
                continue

            if ARGS.test:
                # The mock bridge emits a wall-clock scripted timeline; queue
                # every packet and replay it at the nominal 60 Hz rate below,
                # so a render-limited loop cannot compress the timeline.
                self._test_packet_queue.append(
                    (
                        np.asarray(decoded.joint_positions, dtype=np.float64),
                        (
                            np.asarray(
                                decoded.follower_positions, dtype=np.float64
                            )
                            if decoded.follower_positions is not None
                            else None
                        ),
                        np.asarray(decoded.leader_positions, dtype=np.float64),
                        int(decoded.sequence),
                    )
                )
                continue

            now = time.monotonic()
            sequence_restarted = (
                self.last_packet_monotonic > 0.0
                and now - self.last_packet_monotonic > self.watchdog_timeout
            )
            if decoded.sequence <= self.last_sequence and not sequence_restarted:
                continue
            self.last_sequence = decoded.sequence
            self.last_packet_monotonic = now
            self.packet_count += 1
            latest = decoded

        if ARGS.test:
            self._test_consume_countdown -= 1
            if self._test_consume_countdown <= 0 and self._test_packet_queue:
                joint_positions, follower_positions, leader_raw, sequence = (
                    self._test_packet_queue.popleft()
                )
                self._test_consume_countdown = 2
                self.last_sequence = sequence
                self.last_packet_monotonic = time.monotonic()
                self.packet_count += 1
                joint_positions = joint_positions.copy()
                if follower_positions is None:
                    follower_positions = None
                leader_raw = leader_raw.copy()
                return joint_positions, follower_positions, leader_raw
        if latest is not None:
            joint_positions = np.asarray(
                latest.joint_positions, dtype=np.float64
            )
            follower_positions = (
                np.asarray(latest.follower_positions, dtype=np.float64)
                if latest.follower_positions is not None
                else None
            )
            leader_raw = np.asarray(
                latest.leader_positions, dtype=np.float64
            )
            return joint_positions, follower_positions, leader_raw
        return None

    def _update_command(
        self,
        joint_target: np.ndarray,
        follower_positions: np.ndarray | None,
        leader_raw: np.ndarray,
    ) -> None:
        """Accept one packet and update the mirrored joint targets."""
        if (
            self.sim_source == "follower"
            and follower_positions is not None
            and np.all(np.isfinite(follower_positions))
        ):
            joint_target = follower_positions
            self.latest_source = "FOLLOWER"
        else:
            self.latest_source = "LEADER"
        joint_target = np.asarray(joint_target, dtype=np.float64)
        arm_target = np.clip(joint_target[:5], self.arm_lower, self.arm_upper)
        jaw_target = float(
            np.clip(joint_target[5], self.jaw_lower, self.jaw_upper)
        )

        arm_error = arm_target - self.command_joints[:5]
        arm_delta = np.clip(arm_error, -self.max_arm_step, self.max_arm_step)
        # Suppress encoder noise near a held pose, but respond quickly when
        # the leader actually moves.
        arm_delta = np.where(
            np.abs(arm_error) <= self.arm_deadband, 0.0, arm_delta
        )
        arm_alpha = np.where(
            np.abs(arm_error) >= self.fast_filter_threshold,
            self.fast_filter_alpha,
            self.filter_alpha,
        )
        jaw_delta = float(
            np.clip(
                jaw_target - self.command_joints[5],
                -self.max_jaw_step,
                self.max_jaw_step,
            )
        )
        if self.arm_control_mode == "direct":
            # Match the reBot receiver: the five arm joints mirror the newest
            # sample immediately and their drives hold the pose after a
            # watchdog timeout. The Jaw is still driven (never teleported) so
            # contact with a vial is resolved physically.
            self.command_joints[:5] = arm_target.copy()
        else:
            self.command_joints[:5] += arm_alpha * arm_delta
        self.command_joints[5] += self.jaw_alpha * jaw_delta
        self.latest_received_joints = joint_target.copy()
        self.latest_leader_raw = np.asarray(leader_raw, dtype=np.float64)

    def _apply_targets(self) -> None:
        assert self.robot is not None
        assert self.controller is not None
        assert self.arm_indices is not None
        if self.arm_control_mode == "direct":
            self.robot.set_joint_positions(
                self.command_joints[:5],
                joint_indices=self.arm_indices,
            )
            self.robot.set_joint_velocities(
                np.zeros(len(self.arm_indices), dtype=np.float64),
                joint_indices=self.arm_indices,
            )
        indices = np.concatenate((self.arm_indices, [self.jaw_index]))
        self.controller.apply_action(
            ArticulationAction(
                joint_positions=self.command_joints,
                joint_indices=indices,
            )
        )

    def _gripper_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
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

    def _assist_corridor_samples(self) -> list[np.ndarray]:
        position, orientation = self._gripper_world_pose()
        return [
            position
            + _quaternion_rotate(
                orientation,
                np.array([0.0, 0.0, -float(offset)]),
            )
            for offset in ASSIST_CORRIDOR_OFFSETS_M
        ]

    def _nearest_vial(self) -> tuple | None:
        """Nearest vial by corridor distance: name, target, position, corridor_d, direct_d."""
        samples = self._assist_corridor_samples()
        gripper_position, _ = self._gripper_world_pose()
        best = None
        for name, target in self.targets.items():
            position = np.asarray(
                target.get_world_pose()[0], dtype=np.float64
            )
            corridor_distance = min(
                float(np.linalg.norm(position - sample))
                for sample in samples
            )
            direct_distance = float(
                np.linalg.norm(position - gripper_position)
            )
            if best is None or corridor_distance < best[3]:
                best = (name, target, position, corridor_distance, direct_distance)
        return best

    def _build_kinematics(self) -> None:
        """Extract the FK chain from the USD joint frames (analytic FK).

        Each revolute joint composes parent ∘ T(localPos0, localRot0) ∘
        Rz(q) ∘ T(localPos1, localRot1)^-1; the frames come straight from
        the referenced asset, so the FK tracks the USD exactly.
        """
        from pxr import Gf, UsdGeom

        stage = get_current_stage()
        self._ik_frames = []
        for name in ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"):
            joint = stage.GetPrimAtPath(f"/World/Robot/joints/{name}")
            if not joint.IsValid():
                raise RuntimeError(f"IK joint is missing: {name}")
            p0 = np.asarray(joint.GetAttribute("physics:localPos0").Get(), dtype=np.float64)
            q0 = joint.GetAttribute("physics:localRot0").Get()
            p1 = np.asarray(joint.GetAttribute("physics:localPos1").Get(), dtype=np.float64)
            q1 = joint.GetAttribute("physics:localRot1").Get()
            self._ik_frames.append(
                (
                    p0,
                    np.array([q0.GetReal(), *q0.GetImaginary()], dtype=np.float64),
                    p1,
                    np.array([q1.GetReal(), *q1.GetImaginary()], dtype=np.float64),
                )
            )
        # The gripper link's authored local translate inside the asset root
        # (constant; the joint frames already end at the gripper link, so
        # this is zero and kept for clarity).
        self._ik_gripper_local = np.zeros(3, dtype=np.float64)
        # The root prim's world transform is the FK base.
        root = stage.GetPrimAtPath("/World/Robot")
        ops = UsdGeom.Xformable(root).GetOrderedXformOps()
        self._ik_base_pos = np.asarray(ops[0].Get(), dtype=np.float64)
        orient = ops[1].Get()
        self._ik_base_quat = np.array(
            [orient.GetReal(), *orient.GetImaginary()], dtype=np.float64
        )
        self._ik_validated = False

    def _fk(self, arm_joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Gripper-link world pose for the five arm joint values (rad)."""
        pos = self._ik_base_pos.copy()
        quat = self._ik_base_quat.copy()
        for value, (p0, r0, p1, r1) in zip(arm_joints, self._ik_frames):
            # parent ∘ T0
            pos = pos + _quaternion_rotate(quat, p0)
            quat = _quaternion_multiply(quat, r0)
            # ∘ Rz(q)
            half = 0.5 * float(value)
            rz = np.array(
                [math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64
            )
            quat = _quaternion_multiply(quat, rz)
            # ∘ T1^-1
            r1_inv = np.array(
                [r1[0], -r1[1], -r1[2], -r1[3]], dtype=np.float64
            )
            pos = pos + _quaternion_rotate(quat, -_quaternion_rotate(r1_inv, p1))
            quat = _quaternion_multiply(quat, r1_inv)
        pos = pos + _quaternion_rotate(quat, self._ik_gripper_local)
        return pos, quat

    def _fk_tcp(self, arm_joints: np.ndarray) -> tuple[np.ndarray, float]:
        """Tool-center point position and tool-direction z (tilt)."""
        pos, quat = self._fk(arm_joints)
        tcp = pos + _quaternion_rotate(
            quat, np.array([0.0, 0.0, -IK_TOOL_TCP_OFFSET_M])
        )
        direction = _quaternion_rotate(quat, np.array([0.0, 0.0, -1.0]))
        return tcp, float(direction[2])

    def _validate_fk(self) -> None:
        actual_position, _ = self._gripper_world_pose()
        fk_position, _ = self._fk(self.command_joints[:5])
        error = float(np.linalg.norm(actual_position - fk_position))
        self._ik_validated = error < 0.01
        verdict = "OK" if self._ik_validated else "MISMATCH — Cartesian IK disabled"
        print(
            f"[ik] Forward kinematics vs physics: gripper error "
            f"{error * 1000:.1f} mm ({verdict})"
        )

    def _apply_cartesian_step(self, direction: np.ndarray, step_m: float) -> None:
        """One damped-least-squares IK step for a Cartesian TCP delta."""
        arm = self.command_joints[:5].copy()
        tcp0, tilt0 = self._fk_tcp(arm)
        jacobian = np.zeros((4, 4))
        epsilon = 0.01
        for column, index in enumerate(IK_JOINT_INDICES):
            probe = arm.copy()
            probe[index] += epsilon
            tcp, tilt = self._fk_tcp(probe)
            jacobian[:3, column] = (tcp - tcp0) / epsilon
            jacobian[3, column] = (tilt - tilt0) / epsilon
        task = np.array(
            [direction[0] * step_m, direction[1] * step_m, direction[2] * step_m, 0.0]
        )
        damping = 0.03
        jt = jacobian.T
        try:
            delta = np.linalg.solve(
                jt @ jacobian + (damping**2) * np.eye(4), jt @ task
            )
        except np.linalg.LinAlgError:
            return
        for column, index in enumerate(IK_JOINT_INDICES):
            value = float(np.clip(
                arm[index] + delta[column],
                self.arm_lower[index],
                self.arm_upper[index],
            ))
            self.command_joints[index] = value
        self._refresh_leader_raw_from_command()

    def _refresh_leader_raw_from_command(self) -> None:
        self.latest_leader_raw = np.asarray(
            [
                rad_to_raw(name, value)
                for name, value in zip(SO101_LEADER_JOINTS, self.command_joints)
            ],
            dtype=np.float64,
        )

    def _create_grasp_guide(self) -> None:
        """Terminal-numbers plus a 3D guide line for depth/occlusion cues.

        A thin line from the gripper to the nearest vial and a marker sphere
        at the vial, drawn in the debug viewport (invisible to recordings:
        hidden while an episode is being captured).  Green when the vial is
        inside the grasp corridor, orange otherwise.
        """
        from pxr import Sdf, UsdGeom, UsdShade

        stage = get_current_stage()
        UsdGeom.Scope.Define(stage, "/World/Guides")
        self._guide_green = _preview_guide_material(
            stage, "/World/Guides/GuideGreen", (0.1, 0.85, 0.2)
        )
        self._guide_orange = _preview_guide_material(
            stage, "/World/Guides/GuideOrange", (1.0, 0.55, 0.1)
        )
        line = UsdGeom.Cylinder.Define(
            stage, "/World/Guides/GraspGuideLine"
        )
        line.CreateRadiusAttr(0.0025)
        line.CreateHeightAttr(1.0)
        line.CreateAxisAttr(UsdGeom.Tokens.z)
        marker = UsdGeom.Sphere.Define(
            stage, "/World/Guides/GraspGuideMarker"
        )
        marker.CreateRadiusAttr(0.012)
        for prim in (line, marker):
            UsdGeom.Imageable(prim).CreateVisibilityAttr("inherited")
        self._guide_line_path = "/World/Guides/GraspGuideLine"
        self._guide_marker_path = "/World/Guides/GraspGuideMarker"

    def _update_grasp_guide(self) -> None:
        if self._guide_line_path is None:
            return
        from pxr import Usd, UsdGeom, UsdShade

        stage = get_current_stage()
        recording = bool(
            self.recorder is not None and self.recorder.recording
        )
        nearest = self._nearest_vial()
        line = UsdGeom.Cylinder(stage.GetPrimAtPath(self._guide_line_path))
        marker = UsdGeom.Sphere(
            stage.GetPrimAtPath(self._guide_marker_path)
        )
        if recording or nearest is None:
            UsdGeom.Imageable(line.GetPrim()).GetVisibilityAttr().Set(
                "invisible"
            )
            UsdGeom.Imageable(marker.GetPrim()).GetVisibilityAttr().Set(
                "invisible"
            )
            return
        name, _target, vial_position, corridor_distance, _direct = nearest
        gripper_position, _ = self._gripper_world_pose()
        UsdGeom.Imageable(line.GetPrim()).GetVisibilityAttr().Set(
            "inherited"
        )
        UsdGeom.Imageable(marker.GetPrim()).GetVisibilityAttr().Set(
            "inherited"
        )
        direction = vial_position - gripper_position
        length = float(np.linalg.norm(direction))
        if length < 1e-4:
            return
        direction = direction / length
        # orient the cylinder's +Z along the direction
        z_axis = np.array([0.0, 0.0, 1.0])
        cross = np.cross(z_axis, direction)
        sin = float(np.linalg.norm(cross))
        cos = float(np.dot(z_axis, direction))
        if sin < 1e-9:
            orientation = (
                np.array([1.0, 0.0, 0.0, 0.0])
                if cos > 0
                else np.array([0.0, 1.0, 0.0, 0.0])
            )
        else:
            axis = cross / sin
            angle = math.atan2(sin, cos)
            half = angle / 2.0
            orientation = np.array(
                [math.cos(half), *(math.sin(half) * axis)], dtype=np.float64
            )
        midpoint = gripper_position + direction * (length / 2.0)
        _set_guide_transform(
            stage.GetPrimAtPath(self._guide_line_path),
            midpoint,
            orientation,
            length,
        )
        _set_guide_transform(
            stage.GetPrimAtPath(self._guide_marker_path),
            vial_position,
            np.array([1.0, 0.0, 0.0, 0.0]),
            1.0,
        )
        material = (
            self._guide_green
            if corridor_distance <= ASSIST_CORRIDOR_RADIUS_M
            else self._guide_orange
        )
        for prim in (
            stage.GetPrimAtPath(self._guide_line_path),
            stage.GetPrimAtPath(self._guide_marker_path),
        ):
            UsdShade.MaterialBindingAPI(prim).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )
        self._nearest_vial_name = name
        self._nearest_vial_distance = corridor_distance

    def _filter_gripper_contacts(self, vial_path: str, filter_on: bool) -> None:
        """Filter the vial's contacts with the gripper/jaw colliders while attached."""
        from pxr import Sdf, UsdPhysics

        collider = get_current_stage().GetPrimAtPath(f"{vial_path}/collider")
        if not collider.IsValid():
            return
        pairs = UsdPhysics.FilteredPairsAPI.Apply(collider)
        relationship = pairs.CreateFilteredPairsRel()
        if filter_on:
            relationship.SetTargets(
                [Sdf.Path(path) for path in GRIPPER_COLLIDER_PATHS]
            )
        else:
            relationship.SetTargets([])

    def _update_grasp_assist(self) -> None:
        if self.attached_target is not None:
            if (
                float(self.latest_leader_raw[5])
                >= ASSIST_RELEASE_ABOVE_RAW
            ):
                target = self.attached_target
                target.set_linear_velocity(np.zeros(3, dtype=np.float64))
                target.set_angular_velocity(np.zeros(3, dtype=np.float64))
                if self.attached_target_path is not None:
                    self._filter_gripper_contacts(
                        self.attached_target_path, filter_on=False
                    )
                print(
                    f"[RELEASE] {self.attached_target_name} released; "
                    "gravity takes over"
                )
                self.attached_target = None
                self.attached_target_name = None
                self.attached_target_path = None
            return

        if float(self.latest_leader_raw[5]) > ASSIST_ATTACH_BELOW_RAW:
            return
        nearest = self._nearest_vial()
        if nearest is None:
            return
        name, target, _position, distance, _direct = nearest
        if distance > ASSIST_CORRIDOR_RADIUS_M:
            return
        body_position, body_orientation = self._gripper_world_pose()
        target_position, target_orientation = target.get_world_pose()
        target_position = np.asarray(target_position, dtype=np.float64)
        target_orientation = _quaternion_normalize(
            np.asarray(target_orientation, dtype=np.float64)
        )
        body_inverse = _quaternion_normalize(
            np.asarray(
                [
                    body_orientation[0],
                    -body_orientation[1],
                    -body_orientation[2],
                    -body_orientation[3],
                ],
                dtype=np.float64,
            )
        )
        self.attached_relative_position = _quaternion_rotate(
            body_inverse, target_position - body_position
        ).copy()
        self.attached_relative_orientation = _quaternion_multiply(
            body_inverse, target_orientation
        ).copy()
        self.attached_target = target
        self.attached_target_name = name
        self.attached_target_path = self.target_paths[name]
        self._filter_gripper_contacts(self.attached_target_path, True)
        target.set_linear_velocity(np.zeros(3, dtype=np.float64))
        target.set_angular_velocity(np.zeros(3, dtype=np.float64))
        self.episode_grasped = True
        print(
            f"[GRASP] {name} attached at gripper distance "
            f"{distance:.3f}m; pose preserved. Open the leader gripper "
            "to release"
        )

    def _update_attached_target_pose(self) -> None:
        if self.attached_target is None:
            return
        body_position, body_orientation = self._gripper_world_pose()
        target_position = body_position + _quaternion_rotate(
            body_orientation, self.attached_relative_position
        )
        target_orientation = _quaternion_multiply(
            body_orientation, self.attached_relative_orientation
        )
        self.attached_target.set_world_pose(
            position=target_position,
            orientation=target_orientation,
        )
        self.attached_target.set_linear_velocity(
            np.zeros(3, dtype=np.float64)
        )
        self.attached_target.set_angular_velocity(
            np.zeros(3, dtype=np.float64)
        )

    def _detach_assist(self, silent: bool = True) -> None:
        if self.attached_target is None:
            return
        if self.attached_target_path is not None:
            self._filter_gripper_contacts(
                self.attached_target_path, filter_on=False
            )
        if not silent:
            print(f"[RELEASE] {self.attached_target_name} detached")
        self.attached_target = None
        self.attached_target_name = None
        self.attached_target_path = None

    def _validate_test(self) -> bool:
        assert self.robot is not None
        joints = np.asarray(
            self.robot.get_joint_positions(
                joint_indices=np.concatenate(
                    (self.arm_indices, [self.jaw_index])
                )
            ),
            dtype=np.float64,
        )
        error_deg = np.degrees(
            np.abs(joints - self.command_joints)
        )
        self.test_tracking_error_deg = float(np.max(error_deg))
        # The vial origin sits 17 mm above its base, so a vial resting on the
        # work surface reads z ~= 0.017 m; they spawn at z = 0.057 m.
        base_position = np.asarray(
            self.robot.get_world_pose()[0], dtype=np.float64
        )
        expected_base = np.asarray(
            [SO101_POSITION[0], SO101_POSITION[1], SO101_POSITION[2]],
            dtype=np.float64,
        )
        results = {
            "packets": self.packet_count >= ARGS.test_packets,
            "joints_finite": bool(np.all(np.isfinite(joints))),
            "arm_tracking": bool(
                self.test_tracking_error_deg <= TEST_TRACKING_TOLERANCE_DEG
            ),
            "base_anchored": bool(
                np.linalg.norm(base_position - expected_base) < 0.01
            ),
        }
        vials_settled = True
        for target in self.targets.values():
            position = np.asarray(
                target.get_world_pose()[0], dtype=np.float64
            )
            velocity = np.asarray(
                target.get_linear_velocity(), dtype=np.float64
            )
            if not (0.005 <= float(position[2]) <= 0.12):
                vials_settled = False
            if float(np.linalg.norm(velocity)) > 0.5:
                vials_settled = False
        results["vials_settled"] = vials_settled
        print(
            f"[test] packets={self.packet_count} "
            f"tracking_error_deg={self.test_tracking_error_deg:.3f} "
            f"base={[round(float(v), 3) for v in base_position]} "
            f"joints_deg={np.degrees(joints).round(2).tolist()}"
        )
        for name, ok in results.items():
            print(f"[test-{ 'PASS' if ok else 'FAIL' }] {name}")
        return all(results.values())

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
                if self.keyboard_mode:
                    # The bridge stays live (watchdog/recording gate), but
                    # the leader's targets do not fight the keyboard.
                    self.last_packet_monotonic = time.monotonic()
                else:
                    self._update_command(*latest)
                self._watchdog_reported = False
            elif (
                self.last_packet_monotonic > 0.0
                and time.monotonic() - self.last_packet_monotonic
                > self.watchdog_timeout
                and not self._watchdog_reported
                and not self.keyboard_mode
            ):
                print(
                    "[watchdog] Leader data timed out; the simulated arm is "
                    "holding its last target"
                )
                self._watchdog_reported = True

            self._handle_keyboard()

            if self._auto_record_remaining > 0:
                if self._auto_record_phase == "reset":
                    print(
                        f"[auto-record] Episode "
                        f"{ARGS.auto_record - self._auto_record_remaining + 1}"
                        f"/{ARGS.auto_record} starting"
                    )
                    self._reset_episode()
                    self._start_recording()
                    self._auto_record_phase = "recording"
                    self._auto_record_deadline = self.sim_elapsed + 5.5
                elif (
                    self._auto_record_phase == "recording"
                    and self.sim_elapsed >= self._auto_record_deadline
                ):
                    self._stop_recording()
                    self._auto_record_remaining -= 1
                    self._auto_record_phase = (
                        "reset" if self._auto_record_remaining > 0 else "off"
                    )

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

            self._apply_targets()
            self._update_grasp_assist()
            if self._guide_line_path is not None:
                self._update_grasp_guide()
            recording = bool(
                self.recorder is not None and self.recorder.recording
            )
            if self.render_frames or (recording and not (ARGS.headless or ARGS.test)):
                render = True
            else:
                # Headless recording: render only when the 30 FPS capture
                # is due.  Path-Traced products cost ~1 s per frame on this
                # GPU, so rendering every physics step starves the sim
                # clock; the other steps stay physics-only.
                render = recording and (
                    self.sim_elapsed + 1e-9 >= self.next_record_time
                )
            self.world.step(render=render)
            self._update_attached_target_pose()
            self.sim_elapsed += self.physics_dt
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
                if self._guide_line_path is None:
                    nearest = self._nearest_vial()
                    if nearest is not None:
                        (
                            self._nearest_vial_name,
                            _target,
                            _position,
                            self._nearest_vial_distance,
                            _direct,
                        ) = nearest
                status = (
                    ("KB" if self.keyboard_mode else "LIVE")
                    if self.last_packet_monotonic > 0.0
                    and now - self.last_packet_monotonic
                    <= self.watchdog_timeout
                    else ("KB" if self.keyboard_mode else "HOLD")
                )
                q_deg = np.degrees(self.command_joints).round(1).tolist()
                jaw_actual = float(
                    self.robot.get_joint_positions(
                        joint_indices=[self.jaw_index]
                    )[0]
                )
                if not math.isfinite(jaw_actual):
                    jaw_actual = 0.0
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
                    f"jaw_cmd={np.degrees(self.command_joints[5]):.2f}deg "
                    f"jaw_actual={np.degrees(jaw_actual):.2f}deg "
                    f"src={self.latest_source} "
                    f"attached={self.attached_target_name or 'none'} "
                    f"nearest={self._nearest_vial_name or '-'} "
                    f"d={self._nearest_vial_distance:.3f}m "
                    f"in_rack={len(self.objects_in_rack)}/{len(self.targets)}"
                )
                last_report = now

            if self._auto_record_phase == "off" and ARGS.auto_record:
                print("[auto-record] All episodes collected; finishing")
                break

            if self._auto_record_phase == "off" and ARGS.auto_record:
                print("[auto-record] All episodes collected; finishing")
                break

            if ARGS.test and self.packet_count >= ARGS.test_packets:
                for _ in range(30):
                    self._apply_targets()
                    self.world.step(render=False)
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
            # Thresholds calibrated on clean Path-Traced stills of this
            # scene: the front view (workshop gripper-camera framing, mostly
            # smooth white surfaces) measures ~5.7 and the external ~8.4,
            # while a blank or frozen frame reads ~0-1.  The reBot's >= 8.0
            # gate does not fit this view's lower texture.
            images_ok = (
                camera_edges["front"] >= 4.0
                and camera_edges["external"] >= 6.0
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
        for viewport in (
            self.side_viewport,
            self.front_viewport,
            self.external_viewport,
        ):
            if viewport is not None:
                try:
                    viewport.destroy()
                except Exception:
                    pass
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
    teleop = SO101Teleop()
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
