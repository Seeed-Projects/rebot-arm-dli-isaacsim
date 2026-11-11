#!/usr/bin/env python3
"""Build the SO-ARM101 vials scene: the reBot box and task with the robot swapped.

The workspace enclosure, table, vial rack, vials, lighting, and camera
layout come from the reBot scene builder (``build_scene.py``); only the
robot changes.  The NVIDIA Sim-to-Real SO-101 Workshop's SO-ARM101 USD is
referenced at the same ``/World/Robot`` path and painted white, and the
wrist camera is re-attached to the SO-101 gripper link.  The reBot builder
is imported, never modified, so ``scenes/rebot_*`` keeps building exactly
as before.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

_simulation_app = None
try:
    from pxr import (
        Gf,
        PhysxSchema,
        Sdf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdShade,
    )
except ModuleNotFoundError:
    # Isaac Sim exposes pxr only after SimulationApp has started.
    from isaacsim import SimulationApp

    _simulation_app = SimulationApp({"headless": True})
    from pxr import (
        Gf,
        PhysxSchema,
        Sdf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdShade,
    )


from build_scene import (
    VIAL_RACK_PATH,
    _add_external_camera,
    _add_lighting,
    _add_table,
    _add_vial_rack,
    _add_vial_targets,
    _add_workspace_box,
    _cube,
    _look_at_camera,
    _preview_material,
    _set_transform,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "scenes" / "so101_vials_sorting.usda"

ROBOT_ASSET = "../assets/so101/SO-ARM101-USD-NO-CAMERA.usd"
ROBOT_PATH = "/World/Robot"
# The SO-101 USD names its links and joints directly (see
# assets/so101/README.md for the full table).
SO101_JOINT_NAMES = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
)
SO101_BASE_PATH = f"{ROBOT_PATH}/base"
SO101_ROOT_JOINT_PATH = f"{ROBOT_PATH}/root_joint"
SO101_JOINT_PATHS = tuple(
    f"{ROBOT_PATH}/joints/{name}" for name in SO101_JOINT_NAMES
)
GRIPPER_LINK_PATH = f"{ROBOT_PATH}/gripper"

# X = -0.0208 recentres the robot on the enclosure's left-right midline:
# the USD's base plate spans X -0.035..+0.076 around its own origin
# (measured centre +0.0208), so the origin sits that far left of centre.
# Y = 0.45 (owner-specified).  The folded init pose's raised upper arm
# reaches Y + 0.0767 = 0.5267, clear of the open side's interior line at
# 0.535 m.
SO101_POSITION = Gf.Vec3d(-0.0208, 0.45, -0.0301)
# Yaw 0, not the reBot's -90: the SO-101 USD's root_joint carries a -90
# degree Z twist (localRot1) that PhysX applies to the whole link chain, so
# the visible arm of the zero-pose articulation extends along the base
# frame's -Y.  With the base unrotated that points straight into the
# enclosure toward the vials; at -90 degrees it pointed at the side wall.
# Measured on the built scene: at yaw 0 the jaw meshes chain from the base
# toward world -Y at height ~0.26 m.
SO101_YAW_DEG = 0.0

# Vial-grasp hardening: the shipped USD lets the Jaw close at up to
# 573 deg/s with default (low) finger friction and the vials carry no
# damping, so a closing squeeze shoves the vial out of the finger gap
# ("clips through, then pops out").  Three scene-level fixes, modelled on
# what the reBot scene and the workshop's vial config do: cap the closing
# speed, give the gripper/jaw colliders a high-friction physics material,
# and damp the vials (the workshop's own vials use angular damping 100).
SO101_JAW_MAX_VELOCITY_DEG = 120.0
SO101_VIAL_ANGULAR_DAMPING = 100.0
SO101_FINGER_STATIC_FRICTION = 2.4
SO101_FINGER_DYNAMIC_FRICTION = 2.0
SO101_FINGER_COLLIDER_PATHS = (
    f"{GRIPPER_LINK_PATH}/collisions",
    f"{ROBOT_PATH}/jaw/collisions",
)
SO101_GRIPPER_PHYSICS_MATERIAL_PATH = "/World/Looks/SO101GripperPhysics"

# Arm joint velocity caps: the USD ships 573 deg/s, which lets a live
# teleop target drive a link deep into the table within one step and the
# depenetration solver blows the whole articulation up (transforms fly to
# 1e12, joints go NaN).  240 deg/s keeps the per-step penetration bounded;
# the jaw already runs at 120 deg/s from the grasp hardening.
SO101_ARM_MAX_VELOCITY_DEG = 240.0

# Articulation tuning from the workshop's ArticulationRootPropertiesCfg:
# self-collisions off and a stronger position solver than the USD default.
SO101_SELF_COLLISIONS = False
SO101_SOLVER_POSITION_ITERATIONS = 32

# White paint: the printed-parts material is re-authored to a clean white
# while the STS3215 servo bodies keep their dark grey, matching how a real
# white SO-101 kit looks.  Values are linear-space; slightly below 1.0 so
# 4000 K strip lights do not clip the albedo.
SO101_PRINTED_SHADER_PATH = f"{ROBOT_PATH}/Looks/material_a_3d_printed/Shader"
SO101_WHITE = Gf.Vec3f(0.93, 0.93, 0.91)

# Wrist camera rig: the SO-101 Workshop's own gripper-camera pose, replicated
# verbatim.  tasks/task_env_cfg.py spawns camera_ego at prim path
# Robot/gripper/gripper_cam with offset pos (-0.005, 0.06, -0.062) and rot
# euler(-45, 0, 0) in the gripper link's frame, convention "opengl" (a USD
# camera looking along its local -Z with +Y up).  The mount prim carries
# that transform directly, so the rig tracks the gripper exactly like the
# workshop's camera in every arm pose; the FrontCamera child stays at
# identity inside the mount.
SO101_WRIST_CAM_OFFSET = Gf.Vec3d(-0.005, 0.06, -0.062)
SO101_WRIST_CAM_ROTATION = Gf.Quatd(
    math.cos(math.radians(-45.0) / 2.0),
    Gf.Vec3d(math.sin(math.radians(-45.0) / 2.0), 0.0, 0.0),
)
WRIST_CAMERA_MOUNT_PATH = f"{GRIPPER_LINK_PATH}/WristCameraMount"
FRONT_CAMERA_PATH = f"{WRIST_CAMERA_MOUNT_PATH}/FrontCamera"
WRIST_CAMERA_FOCAL_MM = 11.04
SIDE_CAMERA_PATH = "/World/Cameras/Side"

# Kept as the reBot task objects (three vials, one fixed rack), but laid
# out for the SO-101: the rack and the vials swap left/right (X mirrored)
# and move 10 cm toward the robot, which keeps every slot and spawn point
# comfortably inside the SO-101's ~0.38 m reach from its back position.
# The reBot builder still creates the props with all their physics and
# materials; the scene then re-poses the finished prims, so build_scene.py
# stays untouched.
# The rack asset's origin sits at one corner of its 120 mm square
# footprint, so a true left-right swap places that corner at X = +0.10:
# the footprint then spans X 0.10..0.22 with slot centres at
# (0.13/0.19, 0.19/0.25), i.e. centred on (0.16, 0.22), which the grasp
# config's success region uses.
SO101_RACK_POSITION = Gf.Vec3d(0.10, 0.16, 0.006)
SO101_VIAL_POSITIONS = {
    "vial_1": Gf.Vec3d(-0.07, 0.12, 0.057),
    "vial_2": Gf.Vec3d(-0.15, 0.28, 0.057),
    "vial_3": Gf.Vec3d(-0.115, 0.195, 0.057),
}
TARGET_PATHS = {
    "vial_1": "/World/Objects/Vial1",
    "vial_2": "/World/Objects/Vial2",
    "vial_3": "/World/Objects/Vial3",
}


def _joint_drive_target(joint: Usd.Prim) -> Usd.Attribute:
    target = joint.GetAttribute("drive:angular:physics:targetPosition")
    if target is None or not target.IsValid():
        raise RuntimeError(
            f"SO-101 joint has no angular drive target: {joint.GetPath()}"
        )
    return target


def _robot_orient() -> Gf.Quatd:
    yaw = math.radians(SO101_YAW_DEG)
    return Gf.Quatd(
        math.cos(yaw / 2.0), Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0))
    )


def _harden_vial_grasp_physics(stage: Usd.Stage) -> None:
    """Stabilize grasping: slow gentle closure, grippy fingers, damped vials."""
    for path in TARGET_PATHS.values():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Vial is missing for damping: {path}")
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body.CreateAngularDampingAttr(SO101_VIAL_ANGULAR_DAMPING)

    jaw_joint = stage.GetPrimAtPath(SO101_JOINT_PATHS[-1])
    if not jaw_joint.IsValid():
        raise RuntimeError(f"Jaw joint is missing: {SO101_JOINT_PATHS[-1]}")
    jaw_joint.GetAttribute("physxJoint:maxJointVelocity").Set(
        SO101_JAW_MAX_VELOCITY_DEG
    )

    material = UsdShade.Material.Define(
        stage, SO101_GRIPPER_PHYSICS_MATERIAL_PATH
    )
    physics_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_api.CreateStaticFrictionAttr(SO101_FINGER_STATIC_FRICTION)
    physics_api.CreateDynamicFrictionAttr(SO101_FINGER_DYNAMIC_FRICTION)
    physics_api.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr(PhysxSchema.Tokens.max)
    physx_material.CreateRestitutionCombineModeAttr(PhysxSchema.Tokens.min)
    improve_patch = getattr(
        physx_material, "CreateImprovePatchFrictionAttr", None
    )
    if improve_patch is not None:
        improve_patch(True)

    for path in SO101_FINGER_COLLIDER_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Finger collider is missing: {path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )


def _repose_task_props(stage: Usd.Stage) -> None:
    """Swap the rack/vials sides and pull them 10 cm toward the robot.

    The reBot builders have already created the rack and vials (colliders,
    drives-adjacent physics, materials) at the reBot layout; this only
    re-authors their translate ops, keeping the spawn yaw angles.
    """
    rack = stage.GetPrimAtPath(VIAL_RACK_PATH)
    rack_translate = rack.GetAttribute("xformOp:translate")
    if not rack.IsValid() or rack_translate is None or not rack_translate.IsValid():
        raise RuntimeError(f"Vial rack is not re-posable: {VIAL_RACK_PATH}")
    rack_translate.Set(Gf.Vec3d(SO101_RACK_POSITION))

    for name, path in TARGET_PATHS.items():
        prim = stage.GetPrimAtPath(path)
        translate = prim.GetAttribute("xformOp:translate")
        if not prim.IsValid() or translate is None or not translate.IsValid():
            raise RuntimeError(f"Vial is not re-posable: {path}")
        translate.Set(Gf.Vec3d(SO101_VIAL_POSITIONS[name]))


def _add_so101_robot(stage: Usd.Stage) -> None:
    """Reference the SO-ARM101 articulation and anchor it in the enclosure.

    The USD's root_joint is a fixed joint whose body0 is the world, so
    PhysX pins the base to that joint's world frame and would otherwise
    drag the robot back to the world origin regardless of the reference
    transform.  The scene therefore re-authors the joint's world-side
    frame (localPos0/localRot0) so the anchor sits exactly where the
    referenced base is placed: anchor = base_pose * (localPos1, localRot1).
    """
    robot = UsdGeom.Xform.Define(stage, ROBOT_PATH)
    robot.GetPrim().GetReferences().AddReference(ROBOT_ASSET)
    orient = _robot_orient()
    _set_transform(robot.GetPrim(), translate=SO101_POSITION, orient=orient)

    root_joint = stage.GetPrimAtPath(SO101_ROOT_JOINT_PATH)
    if not root_joint.IsValid():
        raise RuntimeError(
            f"SO-101 root joint is missing: {SO101_ROOT_JOINT_PATH}"
        )
    local_pos1 = root_joint.GetAttribute("physics:localPos1").Get()
    local_rot1 = root_joint.GetAttribute("physics:localRot1").Get()
    if local_pos1 is None or local_rot1 is None:
        raise RuntimeError("SO-101 root joint frames are not authored")
    anchor_position = Gf.Vec3f(
        orient.Transform(Gf.Vec3d(local_pos1)) + SO101_POSITION
    )
    anchor_rotation = Gf.Quatf(orient * Gf.Quatd(local_rot1))
    root_joint.GetAttribute("physics:localPos0").Set(anchor_position)
    root_joint.GetAttribute("physics:localRot0").Set(anchor_rotation)

    base = stage.GetPrimAtPath(SO101_BASE_PATH)
    if not base.IsValid() or not base.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(
            f"SO-101 articulation root is missing: {SO101_BASE_PATH}"
        )
    physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(base)
    physx_articulation.CreateSolverPositionIterationCountAttr(
        SO101_SOLVER_POSITION_ITERATIONS
    )
    physx_articulation.CreateSolverVelocityIterationCountAttr(1)
    physx_articulation.CreateEnabledSelfCollisionsAttr(
        SO101_SELF_COLLISIONS
    )

    # All six joints must carry the angular drives the USD ships.  Their
    # authored targets are 0 degrees, so the arm waits flat in the USD zero
    # pose until teleop data arrives.  Cap the five arm joints' velocity
    # (the Jaw is capped in the grasp hardening below).
    for path in SO101_JOINT_PATHS[:-1]:
        joint = stage.GetPrimAtPath(path)
        if not joint.IsValid():
            raise RuntimeError(f"SO-101 joint is missing: {path}")
        _joint_drive_target(joint)
        joint.GetAttribute("physxJoint:maxJointVelocity").Set(
            SO101_ARM_MAX_VELOCITY_DEG
        )
    jaw_joint = stage.GetPrimAtPath(SO101_JOINT_PATHS[-1])
    if not jaw_joint.IsValid():
        raise RuntimeError(f"SO-101 joint is missing: {SO101_JOINT_PATHS[-1]}")
    _joint_drive_target(jaw_joint)


def _paint_robot_white(stage: Usd.Stage) -> None:
    """Re-author the printed-parts colour; the asset file itself is untouched."""
    shader = stage.GetPrimAtPath(SO101_PRINTED_SHADER_PATH)
    if not shader.IsValid():
        raise RuntimeError(
            f"SO-101 printed material is missing: {SO101_PRINTED_SHADER_PATH}"
        )
    colour = shader.GetAttribute("inputs:diffuse_color_constant")
    if colour is None or not colour.IsValid():
        raise RuntimeError(
            "SO-101 printed material has no diffuse_color_constant input"
        )
    colour.Set(SO101_WHITE)


def _add_wrist_camera(stage: Usd.Stage) -> None:
    """Attach the workshop's gripper camera to the SO-101 gripper link.

    The mount prim carries the workshop's offset (translate + Rx(-45))
    directly in the gripper link's frame — no derived math — and the
    FrontCamera child is authored at identity, so it looks along the
    mount's -Z with +Y up, exactly the workshop's opengl-convention view.
    """
    mount = UsdGeom.Xform.Define(stage, WRIST_CAMERA_MOUNT_PATH)
    mount_xform = UsdGeom.Xformable(mount.GetPrim())
    mount_xform.ClearXformOpOrder()
    mount_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        SO101_WRIST_CAM_OFFSET
    )
    mount_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        SO101_WRIST_CAM_ROTATION
    )

    housing_material = _preview_material(
        stage, "/World/Looks/SO101WristCamera", (0.045, 0.048, 0.055), 0.42
    )
    lens_material = _preview_material(
        stage,
        "/World/Looks/SO101WristCameraLens",
        (0.012, 0.018, 0.03),
        0.12,
        0.35,
    )
    # Camera visuals sit behind the sensor plane along the mount's +Z (the
    # camera's backward axis).
    housing = UsdGeom.Cube.Define(stage, f"{WRIST_CAMERA_MOUNT_PATH}/Body")
    housing.CreateSizeAttr(1.0)
    _set_transform(
        housing.GetPrim(),
        translate=Gf.Vec3d(0.0, 0.0, 0.009),
        scale=Gf.Vec3d(0.024, 0.024, 0.007),
    )
    UsdShade.MaterialBindingAPI(housing.GetPrim()).Bind(housing_material)
    lens = UsdGeom.Cylinder.Define(stage, f"{WRIST_CAMERA_MOUNT_PATH}/Lens")
    lens.CreateAxisAttr(UsdGeom.Tokens.z)
    lens.CreateRadiusAttr(0.006)
    lens.CreateHeightAttr(0.005)
    _set_transform(lens.GetPrim(), translate=Gf.Vec3d(0.0, 0.0, 0.004))
    UsdShade.MaterialBindingAPI(lens.GetPrim()).Bind(lens_material)

    # The camera itself: identity local transform inside the mount.  The
    # domain randomizer re-aims it in this frame with its own look-at.
    camera = UsdGeom.Camera.Define(stage, FRONT_CAMERA_PATH)
    camera.CreateFocalLengthAttr(WRIST_CAMERA_FOCAL_MM)
    # Match the 4:3 image aspect used by teleoperation and LeRobot recording.
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(15.71625)
    camera.CreateFocusDistanceAttr(0.16)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))


def _add_side_camera(stage: Usd.Stage) -> None:
    """Fixed side camera at the robot's upper-left, aimed across the enclosure."""
    side_eye = Gf.Vec3d(0.37, -0.05, 0.60)
    side_target = Gf.Vec3d(0.0, 0.25, 0.16)
    side_forward = (side_target - side_eye).GetNormalized()
    side_orientation = Gf.Rotation(
        Gf.Vec3d(0.0, 0.0, 1.0), side_forward
    ).GetQuat()

    camera_body_material = _preview_material(
        stage, "/World/Looks/FixedCamera", (0.035, 0.038, 0.045), 0.45
    )
    camera_lens_material = _preview_material(
        stage, "/World/Looks/FixedCameraLens", (0.015, 0.025, 0.05), 0.12, 0.25
    )
    UsdGeom.Xform.Define(stage, "/World/Cameras/SideModel")
    side_body = UsdGeom.Cube.Define(stage, "/World/Cameras/SideModel/Body")
    side_body.CreateSizeAttr(1.0)
    _set_transform(
        side_body.GetPrim(),
        translate=side_eye - side_forward * 0.025,
        orient=side_orientation,
        scale=Gf.Vec3d(0.05, 0.04, 0.04),
    )
    UsdShade.MaterialBindingAPI(side_body.GetPrim()).Bind(camera_body_material)
    side_lens = UsdGeom.Cylinder.Define(
        stage, "/World/Cameras/SideModel/Lens"
    )
    side_lens.CreateAxisAttr(UsdGeom.Tokens.z)
    side_lens.CreateRadiusAttr(0.011)
    side_lens.CreateHeightAttr(0.014)
    _set_transform(
        side_lens.GetPrim(),
        translate=side_eye - side_forward * 0.002,
        orient=side_orientation,
    )
    UsdShade.MaterialBindingAPI(side_lens.GetPrim()).Bind(camera_lens_material)
    _look_at_camera(
        stage,
        SIDE_CAMERA_PATH,
        eye=side_eye,
        target=side_target,
        up=Gf.Vec3d(0.0, 0.0, 1.0),
        focal_length=9.5,
        clipping_range=Gf.Vec2f(0.01, 100.0),
    )


def so101_scene_is_configured(stage: Usd.Stage) -> bool:
    """Return whether the SO-101 robot, white paint, and cameras are in place."""
    base = stage.GetPrimAtPath(SO101_BASE_PATH)
    if not base.IsValid() or not base.HasAPI(UsdPhysics.ArticulationRootAPI):
        return False
    for path in SO101_JOINT_PATHS:
        joint = stage.GetPrimAtPath(path)
        if not joint.IsValid():
            return False
        target = joint.GetAttribute("drive:angular:physics:targetPosition")
        if target is None or target.Get() is None:
            return False
    shader = stage.GetPrimAtPath(SO101_PRINTED_SHADER_PATH)
    if not shader.IsValid():
        return False
    colour = shader.GetAttribute("inputs:diffuse_color_constant").Get()
    if colour is None or not all(
        math.isclose(float(component), float(expected), rel_tol=1e-3, abs_tol=1e-3)
        for component, expected in zip(colour, SO101_WHITE)
    ):
        return False
    mount = stage.GetPrimAtPath(WRIST_CAMERA_MOUNT_PATH)
    if not mount.IsValid():
        return False
    mount_translate = mount.GetAttribute("xformOp:translate").Get()
    mount_orient = mount.GetAttribute("xformOp:orient").Get()
    if mount_translate is None or mount_orient is None:
        return False
    if not all(
        math.isclose(
            float(component), float(expected), rel_tol=1e-6, abs_tol=1e-6
        )
        for component, expected in zip(mount_translate, SO101_WRIST_CAM_OFFSET)
    ):
        return False
    if not math.isclose(
        float(mount_orient.GetReal()),
        float(SO101_WRIST_CAM_ROTATION.GetReal()),
        rel_tol=1e-4,
        abs_tol=1e-4,
    ):
        return False
    for path in (
        FRONT_CAMERA_PATH,
        SIDE_CAMERA_PATH,
        "/World/Cameras/External",
    ):
        camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
        if not camera:
            return False
        horizontal = camera.GetHorizontalApertureAttr().Get()
        vertical = camera.GetVerticalApertureAttr().Get()
        if horizontal is None or vertical is None or not math.isclose(
            float(horizontal) / float(vertical), 4.0 / 3.0, rel_tol=1e-5
        ):
            return False
    for path in TARGET_PATHS.values():
        if not stage.GetPrimAtPath(path).IsValid():
            return False
    for path in TARGET_PATHS.values():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return False
        damping = prim.GetAttribute("physxRigidBody:angularDamping").Get()
        if damping is None or not math.isclose(
            float(damping), SO101_VIAL_ANGULAR_DAMPING, rel_tol=1e-6
        ):
            return False
    for path in SO101_JOINT_PATHS[:-1]:
        velocity = stage.GetPrimAtPath(path).GetAttribute(
            "physxJoint:maxJointVelocity"
        ).Get()
        if velocity is None or not math.isclose(
            float(velocity), SO101_ARM_MAX_VELOCITY_DEG, rel_tol=1e-6
        ):
            return False
    jaw_joint = stage.GetPrimAtPath(SO101_JOINT_PATHS[-1])
    if not jaw_joint.IsValid():
        return False
    velocity = jaw_joint.GetAttribute("physxJoint:maxJointVelocity").Get()
    if velocity is None or not math.isclose(
        float(velocity), SO101_JAW_MAX_VELOCITY_DEG, rel_tol=1e-6
    ):
        return False
    for path in SO101_FINGER_COLLIDER_PATHS:
        relationship = stage.GetPrimAtPath(path).GetRelationship(
            "material:binding:physics"
        )
        if (
            not relationship.IsValid()
            or relationship.GetTargets()
            != [Sdf.Path(SO101_GRIPPER_PHYSICS_MATERIAL_PATH)]
        ):
            return False
    rack = stage.GetPrimAtPath(VIAL_RACK_PATH)
    if not rack.IsValid():
        return False
    rack_translate = rack.GetAttribute("xformOp:translate").Get()
    if rack_translate is None or not all(
        math.isclose(
            float(component), float(expected), rel_tol=1e-6, abs_tol=1e-6
        )
        for component, expected in zip(rack_translate, SO101_RACK_POSITION)
    ):
        return False
    return True


def build_scene_so101(
    output: Path = DEFAULT_OUTPUT,
    render_mode: str | None = None,
) -> Path:
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    # Rebuilding from inside a running app has the scene already loaded as a
    # layer, so reuse it instead of failing on CreateNew.
    existing = Sdf.Layer.Find(str(output))
    if existing is not None:
        existing.Clear()
        stage = Usd.Stage.Open(existing)
    else:
        if output.exists():
            output.unlink()
        stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(120.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(1200.0)
    if render_mode is not None:
        layer_data = dict(stage.GetRootLayer().customLayerData)
        layer_data["renderSettings"] = {"rtx:rendermode": render_mode}
        stage.GetRootLayer().customLayerData = layer_data

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Looks")
    UsdGeom.Scope.Define(stage, "/World/Lights")
    UsdGeom.Scope.Define(stage, "/World/Cameras")

    floor_material = _preview_material(
        stage, "/World/Looks/Floor", (0.14, 0.16, 0.19), 0.8
    )
    table_top_material = _preview_material(
        stage, "/World/Looks/TableTop", (0.30, 0.16, 0.075), 0.48
    )
    table_leg_material = _preview_material(
        stage, "/World/Looks/TableLegs", (0.055, 0.06, 0.07), 0.36, 0.65
    )
    _add_workspace_box(stage)
    _add_table(stage, table_top_material, table_leg_material)
    _add_vial_rack(stage)
    _cube(
        stage,
        "/World/Environment/Floor",
        position=(0.0, 0.0, -0.47),
        size=(4.0, 4.0, 0.06),
        material=floor_material,
    )

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    _add_so101_robot(stage)
    _paint_robot_white(stage)
    _add_vial_targets(stage)
    _repose_task_props(stage)
    _harden_vial_grasp_physics(stage)
    _add_side_camera(stage)
    _add_external_camera(stage)
    _add_wrist_camera(stage)
    _add_lighting(stage)

    stage.GetRootLayer().Save()
    print(f"[scene] Generated: {output}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--render-mode",
        default=None,
        choices=("PathTracing", "RayTracedLighting"),
        help="Optional rtx:rendermode stored in the layer's custom data",
    )
    args = parser.parse_args()
    try:
        build_scene_so101(args.output, render_mode=args.render_mode)
    except Exception:
        # python.sh swallows tracebacks into the kit log; surface them.
        import traceback

        traceback.print_exc()
        raise SystemExit(1)
    finally:
        if _simulation_app is not None:
            _simulation_app.close()


if __name__ == "__main__":
    main()
