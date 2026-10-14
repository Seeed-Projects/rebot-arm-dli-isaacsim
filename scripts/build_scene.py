#!/usr/bin/env python3
"""Build the portable reBot stationery-sorting USD scene."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

_simulation_app = None
try:
    from pxr import (
        Gf,
        Sdf,
        Usd,
        UsdGeom,
        UsdLux,
        UsdPhysics,
        UsdShade,
        PhysxSchema,
    )
except ModuleNotFoundError:
    # Isaac Sim exposes pxr only after SimulationApp has started.
    from isaacsim import SimulationApp

    _simulation_app = SimulationApp({"headless": True})
    from pxr import (
        Gf,
        Sdf,
        Usd,
        UsdGeom,
        UsdLux,
        UsdPhysics,
        UsdShade,
        PhysxSchema,
    )


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "scenes" / "rebot_stationery_sorting.usda"
DEFAULT_OUTPUT_STATIONERY = (
    PROJECT_ROOT / "scenes" / "rebot_stationery_pencil_cup.usda"
)
ROBOT_ASSET = "../assets/rebot_physx/00-arm-rs_asm-v3.usd"
CAMERA_MOUNT_ASSET = "../assets/camera/UVC32_mount_portable.usd"
WORKSPACE_ASSET = "../assets/workspace/box.usdz"
STATIONERY_ASSET = "../assets/stationery/stationery_models.usd"

ROBOT_PATH = "/World/Robot"
WORKSPACE_PATH = "/World/WorkspaceBox"
WORKSPACE_COLLIDER_PATH = (
    f"{WORKSPACE_PATH}/box/Base_08q/tn__131_m6/tn__13_/Mesh"
)
WORKSPACE_LIGHT_PATH = f"{WORKSPACE_PATH}/box/DiskLight"
# The authored enclosure measures 794 x 699.19 x 650 mm internally, with
# 3 mm walls.  It is referenced once and rescaled so the interior reads
# 760 x 600 x 700 mm.  The rescaling is pinned at the work surface (Z = 0)
# and at WORKSPACE_Y_PIVOT, which places the resized interior at
# Y = -65 .. +535 mm: that clears the robot's base plate at the rear
# (+524.9 mm), the stationery's worst-case jittered pose (-53 mm) and the
# side camera (-50 mm) at the front.  The walls follow the scale
# (2.87 / 2.75 / 3.23 mm) and CAD details stretch by the same factors, so a
# CAD export at the target size remains the better answer if the enclosure
# ever has to match real hardware exactly.
WORKSPACE_INTERIOR_M = Gf.Vec3d(0.794, 0.69919, 0.650)
WORKSPACE_INTERIOR_TARGET_M = Gf.Vec3d(0.760, 0.600, 0.700)
WORKSPACE_Y_PIVOT = 0.31186
WORKSPACE_FLOOR_TOP_Z = 0.0
WORKSPACE_MODEL_OFFSET_Z = -0.49846
# The task objects: three opaque vials (SO-101 workshop assets) that go into
# a 120 mm square vial rack.  Both assets carry their own materials, so the
# scene must not override them, and both ship their own physics, so the scene
# only tunes contact behaviour and mass.
VIAL_ASSET = "../assets/stationery/Vial_opaque.usda"
VIAL_RACK_ASSET = "../assets/stationery/Vial_rack_simple.usda"
VIAL_RACK_PATH = "/World/Environment/VialRack"
VIAL_RACK_COLLIDER_PATH = f"{VIAL_RACK_PATH}/Body1/Mesh/Mesh"
# The rack ships an SDF collider with the contact offsets left unset, so the
# importer gives it a 20 mm envelope while the slots clear the vial by only
# ~1 mm; the vial was contact-held at the slot mouth and never entered.
# Author a tight envelope and SDF band so the pair can actually meet inside
# the slots.  (The vial side uses STATIONERY_CONTACT_OFFSET.)
VIAL_RACK_CONTACT_OFFSET = 0.0003
VIAL_RACK_SDF_MARGIN = 0.001
VIAL_RACK_SDF_NARROW_BAND = 0.002
TARGET_SPECS = {
    "vial_1": ("/World/Objects/Vial1", "/Vial", 0.018),
    "vial_2": ("/World/Objects/Vial2", "/Vial", 0.018),
    "vial_3": ("/World/Objects/Vial3", "/Vial", 0.018),
}
# The rack asset's origin sits at one corner of its 120 mm footprint and 6 mm
# below its base, so this seats that base on the work surface, centred where
# the pencil cup used to be.
VIAL_RACK_POSITION = Gf.Vec3d(-0.22, 0.06, 0.006)
# The asset's own OmniPBR colour is (0.938, 0.586, 0.043), which the renderer
# treats as linear and therefore shows as a pale yellow.  The scene keeps the
# same material but re-authors the colour as the reference photo's coral
# orange, i.e. the photo's lit-face sRGB #F77A58 converted to linear.  The
# warm strip lights then render it a touch deeper in-scene, which matches the
# photo's shaded faces.  Do not over-compensate for that lighting in the
# albedo: it would read pink under any other exposure.
VIAL_RACK_COLOR = Gf.Vec3f(0.943, 0.241, 0.130)
# The vial is 116.5 mm tall with its origin 17 mm above its base, so its
# centre of mass sits at the bbox centre.
VIAL_CENTER_OF_MASS_Z = 0.0413
# Pencil-cup variant ("stationery"): the source Poly Haven stationery set
# with a fixed pencil cup as the receptacle, and two pens plus an eraser as
# the dynamic targets.  Kept alongside the vial scene as a second task.
PENCIL_CUP_PATH = "/World/Environment/PencilCup"
PENCIL_CUP_CENTER = Gf.Vec3d(-0.16, 0.12, 0.0365904756)
STATIONERY_TARGET_SPECS = {
    "pen_red": ("/World/Objects/PenRed", "/Stationery/PenRed", 0.012),
    "pen_blue": ("/World/Objects/PenBlue", "/Stationery/PenBlue", 0.012),
    "eraser": ("/World/Objects/Eraser", "/Stationery/Eraser", 0.025),
}
TARGET_SPECS_BY_VARIANT = {
    "vials": TARGET_SPECS,
    "stationery": STATIONERY_TARGET_SPECS,
}
STATIONERY_TARGET_CONTACT_OFFSET = 0.002
GRIPPER_BODY_PATH = f"{ROBOT_PATH}/link6"
ASSIST_JOINT_PATH = "/World/GraspAssistJoint"
SIDE_CAMERA_PATH = "/World/Cameras/Side"
WRIST_CAMERA_MOUNT_PATH = f"{GRIPPER_BODY_PATH}/UVC32Mount"
WRIST_CAMERA_BOARD_PATH = f"{WRIST_CAMERA_MOUNT_PATH}/UVC32Camera"
FRONT_CAMERA_PATH = f"{WRIST_CAMERA_BOARD_PATH}/FrontCamera"
STATIONERY_PHYSICS_MATERIAL_PATH = "/World/Looks/StationeryPhysics"
GRIPPER_PHYSICS_MATERIAL_PATH = "/World/Looks/GripperPhysics"
GRIPPER_COLLIDER_PATHS = (
    f"{ROBOT_PATH}/gripper_left/collisions",
    f"{ROBOT_PATH}/gripper_right/collisions",
)
# The wrist-camera rig is placed from Seeed's own assembly numbers instead of
# being measured off the bracket, because the bracket revision in
# assets/camera and the frames below come from the same CAD:
#   Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf   link6 -> gripper_end
#   Rebot_Arm_description/Camera/urdf/uvc32.urdf      gripper_end -> mount -> board
# The published mount transform expects a mesh whose origin sits at the
# bracket's clamp centre; Rebot_Arm_description/Camera/scripts/
# build-uvc32-assets.py builds exactly that mesh.  The STEP in assets/camera
# keeps the CAD datum instead, which sits at (0, 7.110, 24.310) mm from the
# clamp centre, so that offset is folded into the numbers below.  The clamp's
# wrap axis then lands on link6 +Z (the wrist axis) within 0.25 mm.
WRIST_CAMERA_MOUNT_POSITION = Gf.Vec3d(
    0.0231517170, -0.0002007298, 0.0477621244
)
WRIST_CAMERA_MOUNT_ORIENT = Gf.Quatd(
    0.7062529194, Gf.Vec3d(-0.0026105535, -0.0110436469, -0.7078686578)
)
# Back-centre of the 32 x 32 x 1.6 mm board, which sits on the bracket's
# camera plate, plus the board frame the lens and the camera share.  These are
# link6-relative; the rig itself nests bracket -> board -> lens/camera, so the
# children's own transforms are parent-relative (see _relative_pose).
WRIST_CAMERA_BOARD_POSITION = Gf.Vec3d(
    0.1053627295, -0.0004402568, 0.0456517470
)
WRIST_CAMERA_BOARD_ORIENT = Gf.Quatd(
    0.6728290044, Gf.Vec3d(-0.2202297202, -0.2067096982, 0.6753303649)
)
# Optical origin: 9.9 mm out along the board normal, at the lens front.
WRIST_CAMERA_LENS_FRONT_M = 0.0099
# Distance from the optical origin to the look-at point, matching the previous
# rig's focus distance.
WRIST_CAMERA_FOCUS_M = 0.15358633
WRIST_CAMERA_FORWARD = WRIST_CAMERA_BOARD_ORIENT.Transform(
    Gf.Vec3d(0.0, 0.0, 1.0)
).GetNormalized()
# Horizontal field of view matching the real wrist camera's lens, which is a
# normal-angle module: the previous 4.2 mm (a 136 deg fisheye) was calibrated
# against a different stream.  11.04 mm with the 20.955 mm aperture below is
# 87 deg horizontally, the same class as the D405/D435 wrist cameras the
# official assembly compares this UVC module to.
# config/domain_randomization.json carries the same value, which is what the
# reset actually applies.
WRIST_CAMERA_FOCAL_MM = 11.04

# External data-collection camera on Seeed's "data-collection-camera-mount-v2"
# stand.  The pose below is the hand-placed calibration from the working scene
# (2026-10-13) and must survive rebuilds, so it is authored verbatim here:
# the mount sits just in front of the work surface's rear edge, rotated 180
# degrees about the local (0, 1, 1) diagonal (mast up, camera arm toward the
# robot), its top arm piece is relocated to carry the camera, and the two
# middle arm sections are switched off.  The camera hangs at (0, 60, 650) mm,
# tilted ~13 degrees toward +Y, focal 16 mm (~66 deg horizontal).  The camera
# matrix is not orthonormal (it is exactly as placed in the GUI), so it is
# authored as a raw matrix instead of an eye/target/up look-at; the
# randomizer never rewrites this camera.
EXTERNAL_CAMERA_MOUNT_ASSET = (
    "../assets/camera/data_collection_camera_mount.usd"
)
EXTERNAL_CAMERA_MOUNT_PATH = "/World/Cameras/ExternalMount"
EXTERNAL_CAMERA_PATH = "/World/Cameras/External"
EXTERNAL_CAMERA_MOUNT_POSITION = Gf.Vec3d(0.0, -0.022780342815431287, 0.015)
EXTERNAL_CAMERA_MOUNT_ORIENT = Gf.Quatd(
    6.123233995736766e-17,
    Gf.Vec3d(4.329780281177466e-17, 0.7071067811865476, 0.7071067811865475),
)
EXTERNAL_CAMERA_ARM_PIECE = "tn__0001_ASM_/tn__1_bmf0zaj0/Mesh"
EXTERNAL_CAMERA_ARM_PIECE_TRANSLATE = Gf.Vec3d(
    0.06020525070086105, -0.012551881792145479, 0.6347620242140731
)
EXTERNAL_CAMERA_REMOVED_PIECES = ("tn__0001_ASM_/tn__2_bmf0zaj0",
                                  "tn__0001_ASM_/tn__taKka7")
EXTERNAL_CAMERA_FOCAL_MM = 16.0
EXTERNAL_CAMERA_TRANSFORM = Gf.Matrix4d(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, -0.23, 1.0, 0.0,
    0.0, 0.06, 0.65, 1.0,
)


def _relative_pose(
    parent_position: Gf.Vec3d,
    parent_orient: Gf.Quatd,
    child_position: Gf.Vec3d,
    child_orient: Gf.Quatd,
) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """Express a link6-relative pose in its parent's frame."""
    inverse = parent_orient.GetInverse()
    return inverse.Transform(child_position - parent_position), inverse * child_orient

# Seeed reBot-Isaacsim PR #21 establishes explicit drives for both fingers.
# The imported asset is far too soft (~0.259 N/m), while the raw hardware
# effort ceiling is excessive for small simulated objects.  These values
# retain gravity-resistant position control but cap squeeze force and speed.
GRIPPER_JOINT_PATHS = (
    f"{ROBOT_PATH}/joints/joint_left",
    f"{ROBOT_PATH}/joints/joint_right",
)
GRIPPER_DRIVE_STIFFNESS = 1200.0
GRIPPER_DRIVE_DAMPING = 30.0
GRIPPER_DRIVE_MAX_FORCE = 12.0
GRIPPER_MAX_VELOCITY = 0.08
GRIPPER_STROKE = 0.05

# Small-object contact limits cap residual penetration correction and contact
# impulses caused by direct Real-to-Sim mirroring.
OBJECT_MAX_DEPENETRATION_VELOCITY = 0.25
OBJECT_MAX_CONTACT_IMPULSE = 0.05
GRIPPER_STATIC_FRICTION = 2.4
GRIPPER_DYNAMIC_FRICTION = 2.0
STATIONERY_STATIC_FRICTION = 1.8
STATIONERY_DYNAMIC_FRICTION = 1.5
# Small objects need a small, explicit contact envelope.  The default is
# chosen from shape extent and occasionally allowed a one-frame missed
# contact against the gripper CAD.  It must also stay well under the vial
# rack's ~1 mm slot clearance: anything larger (the importer default is
# 20 mm) makes the rack-vial pair contact at the slot mouth and the vial is
# held above the slot instead of entering it.
STATIONERY_CONTACT_OFFSET = 0.0005
STATIONERY_REST_OFFSET = 0.0
STATIONERY_TORSIONAL_PATCH_RADIUS = 0.002

# The CAD export uses very high angular velocity limits and relatively light
# damping on the wrist joints.  With live 60 Hz leader commands that makes the
# position drives visibly ring after each target update.  Keep the exported
# stiffness/torque limits, but add conservative velocity caps and damping so
# the simulated arm settles instead of behaving like a spring.
ARM_JOINT_PATHS = tuple(f"{ROBOT_PATH}/joints/joint{i}" for i in range(1, 7))
ARM_DRIVE_DAMPING = (85.0, 105.0, 85.0, 30.0, 23.0, 18.0)
# USD's angular maxJointVelocity is authored in degrees/s.
ARM_MAX_VELOCITY = tuple(
    math.degrees(value) for value in (12.0, 12.0, 12.0, 10.0, 10.0, 10.0)
)

# Seeed's current layered USD assigns these four materials per CAD piece.
# The older flat PhysX asset bundled with this project retained the piece
# names but collapsed every visual onto material_C0C0C0, so the same palette
# and bindings are re-authored as scene-level appearance opinions.
ROBOT_MATERIAL_SPECS = {
    "green": ("ReBotGreen", (0.278, 0.616, 0.298), 0.45, 0.0),
    "black": ("ReBotBlack", (0.055, 0.055, 0.062), 0.55, 0.0),
    "aluminium": ("ReBotAluminium", (0.78, 0.80, 0.83), 0.32, 0.85),
    "motor": ("ReBotMotor", (0.13, 0.13, 0.14), 0.38, 0.6),
}
ROBOT_VISUAL_MATERIALS = {
    "base_link/visuals/base_link": "aluminium",
    "link1/visuals/link1": "aluminium",
    "link2/visuals/motor_2_3": "motor",
    "link2/visuals/cnc2": "aluminium",
    "link2/visuals/pla2_black": "black",
    "link2/visuals/pla2_green": "green",
    "link3/visuals/cnc3": "aluminium",
    "link3/visuals/motor_4": "motor",
    "link3/visuals/pla3_black": "black",
    "link3/visuals/pla3_green": "green",
    "link4/visuals/motor_5": "motor",
    "link4/visuals/cnc4": "aluminium",
    "link5/visuals/motor_6": "motor",
    "link5/visuals/cnc5": "aluminium",
    "link5/visuals/pla5_green": "green",
    "link6/visuals/link6": "aluminium",
    "link6/visuals/pla7_green": "green",
    "link6/visuals/cnc7": "aluminium",
    "link6/visuals/motor_7": "motor",
    "gripper_left/visuals/pla_left": "black",
    "gripper_left/visuals/cnc_left": "aluminium",
    "gripper_right/visuals/pla_right": "black",
    "gripper_right/visuals/cnc_right": "aluminium",
}

# The flat PhysX asset has no separate gripper_end rigid body. In link6
# coordinates the grasp centre is 0.09121 m along +Z. The frame
# orientation is the original URDF's fixed link6 -> gripper_end rotation.
# Where the assist joint parks the active vial: its origin 118 mm out along
# the tool axis puts the vial's upper half between the fingers and keeps its
# base clear of the wrist, with the vial upright on the tool axis.
# Grasp-assist joint placement per scene variant: the offset/rotation carry
# the wrist frame to the calibrated grasp point of the variant's test target.
GRASP_JOINT_POSES = {
    "vials": (
        Gf.Vec3f(0.0, 0.0, 0.118),
        Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)),
    ),
    "stationery": (
        Gf.Vec3f(0.0, 0.0, 0.09121),
        Gf.Quatf(0.0, Gf.Vec3f(-0.7071068, 0.0, -0.7071068)),
    ),
}
ASSIST_JOINT_BODY_TARGET = {"vials": "vial_1", "stationery": "pen_red"}


def _set_transform(
    prim: Usd.Prim,
    *,
    translate: Gf.Vec3d | None = None,
    orient: Gf.Quatd | None = None,
    scale: Gf.Vec3d | None = None,
) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    if translate is not None:
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(translate)
    if orient is not None:
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orient)
    if scale is not None:
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(scale)


def _preview_material(
    stage: Usd.Stage,
    path: str,
    color: tuple[float, float, float],
    roughness: float,
    metallic: float = 0.0,
) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _enable_patch_friction(physx_api) -> None:
    """Enable patch friction on a PhysX material where the schema allows it.

    Patch friction is what keeps thin shafts from slipping out of the
    fingers, but not every PhysX schema build exposes the option.
    """
    improve_patch = getattr(
        physx_api, "CreateImprovePatchFrictionAttr", None
    )
    if improve_patch is not None:
        improve_patch(True)




def _cube(
    stage: Usd.Stage,
    path: str,
    *,
    position: tuple[float, float, float],
    size: tuple[float, float, float],
    material: UsdShade.Material,
    collision: bool = True,
    rotate_z_deg: float | None = None,
) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*position)
    )
    if rotate_z_deg is not None:
        xform.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(rotate_z_deg)
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*size))
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _add_vial_rack(stage: Usd.Stage) -> None:
    """Add the vial rack as a fixed receptacle, using its own collision."""
    rack = UsdGeom.Xform.Define(stage, VIAL_RACK_PATH)
    rack.GetPrim().GetReferences().AddReference(VIAL_RACK_ASSET)
    _set_transform(rack.GetPrim(), translate=VIAL_RACK_POSITION)
    # The asset ships a rigid body; pin it so the rack cannot be pushed.
    body = stage.GetPrimAtPath(f"{VIAL_RACK_PATH}/Body1")
    if not body.IsValid() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"Vial rack body is missing: {VIAL_RACK_PATH}/Body1")
    UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)

    # Recolour the asset's own material: same OmniPBR shader, orange input.
    colour = stage.GetPrimAtPath(f"{VIAL_RACK_PATH}/Looks/OmniPBR/Shader")
    if not colour.IsValid():
        raise RuntimeError(f"Vial rack material is missing: {colour}")
    colour.GetAttribute("inputs:diffuse_color_constant").Set(VIAL_RACK_COLOR)

    # Tighten the collider envelope so the vials can enter the slots; see the
    # VIAL_RACK_CONTACT_OFFSET comment for why the shipped defaults fail.
    collider = stage.GetPrimAtPath(VIAL_RACK_COLLIDER_PATH)
    if not collider.IsValid():
        raise RuntimeError(f"Vial rack collider is missing: {VIAL_RACK_COLLIDER_PATH}")
    collision = PhysxSchema.PhysxCollisionAPI.Apply(collider)
    collision.CreateContactOffsetAttr(VIAL_RACK_CONTACT_OFFSET)
    collision.CreateRestOffsetAttr(0.0)
    sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(collider)
    sdf.CreateSdfMarginAttr(VIAL_RACK_SDF_MARGIN)
    sdf.CreateSdfNarrowBandThicknessAttr(VIAL_RACK_SDF_NARROW_BAND)


def _look_at_camera(
    stage: Usd.Stage,
    path: str,
    *,
    eye: Gf.Vec3d,
    target: Gf.Vec3d,
    up: Gf.Vec3d,
    focal_length: float,
    clipping_range: Gf.Vec2f | None = None,
    aperture_mm: tuple[float, float] = (20.955, 15.71625),
) -> None:
    camera = UsdGeom.Camera.Define(stage, path)
    camera_matrix = Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse()
    camera_xform = UsdGeom.Xformable(camera.GetPrim())
    camera_xform.ClearXformOpOrder()
    camera_xform.AddTransformOp().Set(camera_matrix)
    camera.CreateFocalLengthAttr(focal_length)
    # Match the 4:3 image aspect used by teleoperation and LeRobot recording.
    camera.CreateHorizontalApertureAttr(aperture_mm[0])
    camera.CreateVerticalApertureAttr(aperture_mm[1])
    camera.CreateFocusDistanceAttr(float((eye - target).GetLength()))
    if clipping_range is not None:
        camera.CreateClippingRangeAttr(clipping_range)


def _add_uvc32_camera_model(stage: Usd.Stage) -> None:
    """Attach the official Seeed mount and a 32 mm UVC camera visual."""
    mount_material = _preview_material(
        stage, "/World/Looks/UVC32Mount", (0.035, 0.038, 0.045), 0.62
    )
    pcb_material = _preview_material(
        stage, "/World/Looks/UVC32PCB", (0.025, 0.19, 0.09), 0.52
    )
    housing_material = _preview_material(
        stage, "/World/Looks/UVC32Housing", (0.045, 0.048, 0.055), 0.42
    )
    lens_material = _preview_material(
        stage, "/World/Looks/UVC32Lens", (0.012, 0.018, 0.03), 0.12, 0.35
    )

    mount = UsdGeom.Xform.Define(stage, WRIST_CAMERA_MOUNT_PATH)
    mount.GetPrim().GetReferences().AddReference(CAMERA_MOUNT_ASSET)
    _set_transform(
        mount.GetPrim(),
        translate=WRIST_CAMERA_MOUNT_POSITION,
        orient=WRIST_CAMERA_MOUNT_ORIENT,
    )
    UsdShade.MaterialBindingAPI.Apply(mount.GetPrim()).Bind(
        mount_material, UsdShade.Tokens.strongerThanDescendants
    )

    # The rig nests the way the official assembly does:
    #   link6 -> bracket -> board -> lens barrel, lens glass, camera
    # so each child's transform is expressed in its parent's frame.  The
    # board frame's +Z is the plate normal, which is also the optical axis.
    board_local = _relative_pose(
        WRIST_CAMERA_MOUNT_POSITION,
        WRIST_CAMERA_MOUNT_ORIENT,
        WRIST_CAMERA_BOARD_POSITION,
        WRIST_CAMERA_BOARD_ORIENT,
    )
    board_orientation = board_local[1]
    board = UsdGeom.Cube.Define(stage, WRIST_CAMERA_BOARD_PATH)
    board.CreateSizeAttr(1.0)
    _set_transform(
        board.GetPrim(),
        translate=board_local[0] + Gf.Vec3d(0.0, 0.0, 0.0008),
        orient=board_orientation,
        scale=Gf.Vec3d(0.032, 0.032, 0.0016),
    )
    UsdShade.MaterialBindingAPI(board.GetPrim()).Bind(pcb_material)

    housing = UsdGeom.Cylinder.Define(
        stage, f"{WRIST_CAMERA_BOARD_PATH}/LensHousing"
    )
    housing.CreateAxisAttr(UsdGeom.Tokens.z)
    housing.CreateRadiusAttr(0.0065)
    housing.CreateHeightAttr(0.006)
    _set_transform(
        housing.GetPrim(),
        translate=Gf.Vec3d(0.0, 0.0, 0.0069),
    )
    UsdShade.MaterialBindingAPI(housing.GetPrim()).Bind(housing_material)

    lens = UsdGeom.Cylinder.Define(
        stage, f"{WRIST_CAMERA_BOARD_PATH}/LensGlass"
    )
    lens.CreateAxisAttr(UsdGeom.Tokens.z)
    lens.CreateRadiusAttr(0.0047)
    lens.CreateHeightAttr(0.001)
    _set_transform(
        lens.GetPrim(),
        translate=Gf.Vec3d(0.0, 0.0, 0.0094),
    )
    UsdShade.MaterialBindingAPI(lens.GetPrim()).Bind(lens_material)


def _add_external_camera(stage: Usd.Stage) -> None:
    """Attach the hand-placed overhead camera and its stand."""
    mount_material = _preview_material(
        stage, "/World/Looks/ExternalCameraMount", (0.42, 0.44, 0.47), 0.55, 0.55
    )

    mount = UsdGeom.Xform.Define(stage, EXTERNAL_CAMERA_MOUNT_PATH)
    mount.GetPrim().GetReferences().AddReference(EXTERNAL_CAMERA_MOUNT_ASSET)
    _set_transform(
        mount.GetPrim(),
        translate=EXTERNAL_CAMERA_MOUNT_POSITION,
        orient=EXTERNAL_CAMERA_MOUNT_ORIENT,
        scale=Gf.Vec3d(1.0, 1.0, 1.0),
    )
    UsdShade.MaterialBindingAPI.Apply(mount.GetPrim()).Bind(
        mount_material, UsdShade.Tokens.strongerThanDescendants
    )

    # The top arm piece is relocated to hold the camera; the two middle
    # sections are removed.  These are over-rides on the referenced prims.
    arm_mesh = stage.OverridePrim(
        f"{EXTERNAL_CAMERA_MOUNT_PATH}/{EXTERNAL_CAMERA_ARM_PIECE}"
    )
    arm_mesh.CreateAttribute(
        "xformOp:translate", Sdf.ValueTypeNames.Double3
    ).Set(EXTERNAL_CAMERA_ARM_PIECE_TRANSLATE)
    arm_mesh.CreateAttribute(
        "xformOp:orient", Sdf.ValueTypeNames.Quatf
    ).Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    arm_mesh.CreateAttribute(
        "xformOp:scale", Sdf.ValueTypeNames.Float3
    ).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    arm_mesh.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray
    ).Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])
    # Deactivate the removed pieces; author any nested over before its
    # parent goes inactive, because an inactive subtree is unreachable.
    clamp_mesh = stage.OverridePrim(
        f"{EXTERNAL_CAMERA_MOUNT_PATH}/tn__0001_ASM_/tn__taKka7/Mesh"
    )
    clamp_mesh.SetActive(False)
    for piece in EXTERNAL_CAMERA_REMOVED_PIECES:
        stage.OverridePrim(
            f"{EXTERNAL_CAMERA_MOUNT_PATH}/{piece}"
        ).SetActive(False)

    # Attribute values first via look-at, then replace the transform with the
    # exact hand-placed matrix (which is not orthonormal).
    _look_at_camera(
        stage,
        EXTERNAL_CAMERA_PATH,
        eye=Gf.Vec3d(0.0, 0.06, 0.65),
        target=Gf.Vec3d(0.0, 0.1720724, 0.1627222),
        up=Gf.Vec3d(0.0, 0.9745556, 0.2241449),
        focal_length=EXTERNAL_CAMERA_FOCAL_MM,
        clipping_range=Gf.Vec2f(0.01, 100.0),
    )
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(EXTERNAL_CAMERA_PATH))
    xform.ClearXformOpOrder()
    xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(
        EXTERNAL_CAMERA_TRANSFORM
    )


def _add_cameras(stage: Usd.Stage) -> None:
    UsdGeom.Scope.Define(stage, "/World/Cameras")
    # Fixed side camera at the robot's upper-left, aimed across the enclosure.
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

    _add_external_camera(stage)

    _add_uvc32_camera_model(stage)

    # This camera is authored below the board, so its pose follows the whole
    # rig and can share the board's frame: the lens front is 9.9 mm out along
    # the board normal, and the assembly's camera frame has +Y pointing down
    # the image, as ROS optical frames do.  USD cameras look down local -Z
    # with +Y as image-up, and the look-at below expresses both.
    _look_at_camera(
        stage,
        FRONT_CAMERA_PATH,
        eye=Gf.Vec3d(0.0, 0.0, WRIST_CAMERA_LENS_FRONT_M),
        target=Gf.Vec3d(
            0.0, 0.0, WRIST_CAMERA_LENS_FRONT_M + WRIST_CAMERA_FOCUS_M
        ),
        up=Gf.Vec3d(0.0, -1.0, 0.0),
        # The grab_tube_0 front stream uses a strong ultra-wide/fisheye view;
        # this rectilinear approximation matches its fingertip scale (~136°).
        focal_length=WRIST_CAMERA_FOCAL_MM,
        # A physical lens now sits clear of link6, so a normal close near-plane
        # can replace the old 4 cm clipping workaround.
        clipping_range=Gf.Vec2f(0.01, 100.0),
    )
    stage.SetMetadata("renderSettingsPrimPath", "/Render")


def camera_rig_is_configured(stage: Usd.Stage) -> bool:
    """Return whether the physical UVC32 rig and 4:3 cameras are present."""
    mount = stage.GetPrimAtPath(WRIST_CAMERA_MOUNT_PATH)
    board = stage.GetPrimAtPath(WRIST_CAMERA_BOARD_PATH)
    if (
        not mount.IsValid()
        or not mount.HasAuthoredReferences()
        or not board.IsValid()
    ):
        return False
    for path in (
        SIDE_CAMERA_PATH,
        FRONT_CAMERA_PATH,
        EXTERNAL_CAMERA_PATH,
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
    clipping = UsdGeom.Camera(stage.GetPrimAtPath(FRONT_CAMERA_PATH))
    near = clipping.GetClippingRangeAttr().Get()[0]
    return math.isclose(float(near), 0.01, rel_tol=1e-5, abs_tol=1e-6)


def _add_lighting(stage: Usd.Stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(800.0)
    dome.CreateColorAttr(Gf.Vec3f(0.94, 0.96, 1.0))

    key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(650.0)
    # A larger angular diameter produces a broad, soft shadow instead of a
    # moving near-black patch beneath the arm.
    key.CreateAngleAttr(4.0)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.82))
    _set_transform(
        key.GetPrim(),
        orient=Gf.Quatd(
            math.cos(math.radians(32.5)),
            Gf.Vec3d(math.sin(math.radians(32.5)), 0.0, 0.0),
        ),
    )

    strip_material = _preview_material(
        stage, "/World/Looks/WarmLightStrip", (1.0, 0.64, 0.32), 0.22
    )
    for index, x_position in enumerate((-0.20, 0.20), start=1):
        _cube(
            stage,
            f"/World/Lights/WarmStripVisual_{index}",
            position=(x_position, 0.20, 0.625),
            size=(0.035, 0.44, 0.008),
            material=strip_material,
            collision=False,
        )
        strip = UsdLux.RectLight.Define(
            stage, f"/World/Lights/WarmStrip_{index}"
        )
        strip.CreateWidthAttr(0.035)
        strip.CreateHeightAttr(0.44)
        strip.CreateIntensityAttr(950.0)
        strip.CreateNormalizeAttr(True)
        strip.CreateColorAttr(Gf.Vec3f(1.0, 0.84, 0.68))
        strip.CreateEnableColorTemperatureAttr(True)
        strip.CreateColorTemperatureAttr(4000.0)
        _set_transform(
            strip.GetPrim(),
            translate=Gf.Vec3d(x_position, 0.20, 0.618),
        )


def _workspace_placement() -> tuple[Gf.Vec3d, Gf.Vec3d]:
    """Return the translate and scale that give the enclosure its interior."""
    scale = Gf.Vec3d(
        WORKSPACE_INTERIOR_TARGET_M[0] / WORKSPACE_INTERIOR_M[0],
        WORKSPACE_INTERIOR_TARGET_M[1] / WORKSPACE_INTERIOR_M[1],
        WORKSPACE_INTERIOR_TARGET_M[2] / WORKSPACE_INTERIOR_M[2],
    )
    pivot = Gf.Vec3d(0.0, WORKSPACE_Y_PIVOT, WORKSPACE_FLOOR_TOP_Z)
    offset = Gf.Vec3d(0.0, 0.0, WORKSPACE_MODEL_OFFSET_Z)
    # world = scale * local + translate, arranged so the pivot stays put.
    translate = pivot + Gf.CompMult(offset - pivot, scale)
    return translate, scale


def _add_workspace_box(stage: Usd.Stage) -> None:
    """Reference the complete USDZ enclosure and add static collision."""
    workspace = UsdGeom.Xform.Define(stage, WORKSPACE_PATH)
    workspace.GetPrim().GetReferences().AddReference(WORKSPACE_ASSET)
    translate, scale = _workspace_placement()
    _set_transform(workspace.GetPrim(), translate=translate, scale=scale)
    collider = stage.GetPrimAtPath(WORKSPACE_COLLIDER_PATH)
    if not collider.IsValid():
        raise RuntimeError(
            f"Workspace mesh is missing: {WORKSPACE_COLLIDER_PATH}"
        )
    UsdPhysics.CollisionAPI.Apply(collider)
    UsdPhysics.MeshCollisionAPI.Apply(
        collider
    ).CreateApproximationAttr("none")
    packaged_light = stage.GetPrimAtPath(WORKSPACE_LIGHT_PATH)
    if packaged_light.IsValid():
        packaged_light.SetActive(False)


def _add_table(
    stage: Usd.Stage,
    top_material: UsdShade.Material,
    leg_material: UsdShade.Material,
) -> None:
    """Add a static table whose top meets the enclosure's lower surface."""
    UsdGeom.Xform.Define(stage, "/World/Environment/Table")
    # The authored enclosure's lower bound sits at z=-0.00300286 m once the
    # model offset is applied; carry it through the same Z scaling as the
    # enclosure so the table cannot drift away and leave a visible gap.
    _, scale = _workspace_placement()
    table_top_z = -0.00300286194 * scale[2]
    top_thickness = 0.05
    _cube(
        stage,
        "/World/Environment/Table/Top",
        position=(0.0, 0.22, table_top_z - top_thickness / 2.0),
        size=(1.15, 0.95, top_thickness),
        material=top_material,
    )

    floor_top_z = -0.44
    leg_top_z = table_top_z - top_thickness
    leg_height = leg_top_z - floor_top_z
    leg_center_z = (leg_top_z + floor_top_z) / 2.0
    for index, (x_position, y_position) in enumerate(
        ((-0.49, -0.17), (-0.49, 0.61), (0.49, -0.17), (0.49, 0.61)),
        start=1,
    ):
        _cube(
            stage,
            f"/World/Environment/Table/Leg_{index}",
            position=(x_position, y_position, leg_center_z),
            size=(0.065, 0.065, leg_height),
            material=leg_material,
        )


def _add_robot(stage: Usd.Stage) -> None:
    robot = UsdGeom.Xform.Define(stage, ROBOT_PATH)
    robot.GetPrim().GetReferences().AddReference(ROBOT_ASSET)
    yaw = math.radians(-90.0)
    _set_transform(
        robot.GetPrim(),
        translate=Gf.Vec3d(0.0, 0.45488233155, 0.0),
        orient=Gf.Quatd(
            math.cos(yaw / 2.0),
            Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0)),
        ),
    )


def _add_robot_appearance(stage: Usd.Stage) -> None:
    """Restore Seeed's per-piece green/black/aluminium/motor palette."""
    materials: dict[str, UsdShade.Material] = {}
    for key, (name, color, roughness, metallic) in ROBOT_MATERIAL_SPECS.items():
        materials[key] = _preview_material(
            stage,
            f"/World/Looks/{name}",
            color,
            roughness,
            metallic,
        )

    # Each visual group in the flat asset is instanceable. Descendants of an
    # instance cannot receive local material overrides, so de-instance only
    # the eight visual groups; collision prototypes remain instanceable and
    # physics topology is unchanged.
    visual_roots = {
        f"{ROBOT_PATH}/{relative.split('/visuals/', 1)[0]}/visuals"
        for relative in ROBOT_VISUAL_MATERIALS
    }
    for path in sorted(visual_roots):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Robot visual prim is missing: {path}")
        stage.OverridePrim(path).SetInstanceable(False)

    for relative, material_key in ROBOT_VISUAL_MATERIALS.items():
        path = f"{ROBOT_PATH}/{relative}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or prim.IsInstanceProxy():
            raise RuntimeError(
                f"Robot visual component cannot be edited: {path}"
            )
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            materials[material_key],
            UsdShade.Tokens.strongerThanDescendants,
        )


def robot_appearance_is_configured(stage: Usd.Stage) -> bool:
    """Return whether every named robot part resolves to the Seeed material."""
    for relative, material_key in ROBOT_VISUAL_MATERIALS.items():
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{relative}")
        if not prim.IsValid() or prim.IsInstanceProxy():
            return False
        mesh = prim.GetChild("mesh")
        if not mesh.IsValid():
            return False
        material, _relationship = UsdShade.MaterialBindingAPI(
            mesh
        ).ComputeBoundMaterial()
        expected_name = ROBOT_MATERIAL_SPECS[material_key][0]
        if not material or material.GetPath() != Sdf.Path(
            f"/World/Looks/{expected_name}"
        ):
            return False
    return True


def _configure_gripper_physics(stage: Usd.Stage) -> None:
    """Apply gravity-resistant, fruit-safe two-finger position drives."""
    joints = [stage.GetPrimAtPath(path) for path in GRIPPER_JOINT_PATHS]
    if not all(prim.IsValid() for prim in joints):
        missing = [
            path
            for path, prim in zip(GRIPPER_JOINT_PATHS, joints)
            if not prim.IsValid()
        ]
        raise RuntimeError(f"Gripper joints are missing: {missing}")

    for prim in joints:
        drive = UsdPhysics.DriveAPI.Apply(prim, UsdPhysics.Tokens.linear)
        drive.CreateStiffnessAttr(GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr(GRIPPER_DRIVE_DAMPING)
        drive.CreateMaxForceAttr(GRIPPER_DRIVE_MAX_FORCE)
        drive.CreateTypeAttr(UsdPhysics.Tokens.force)
        UsdPhysics.PrismaticJoint(prim).CreateUpperLimitAttr(GRIPPER_STROKE)
        prim.CreateAttribute(
            "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float
        ).Set(GRIPPER_MAX_VELOCITY)
        prim.CreateAttribute(
            "urdf:limit:effort", Sdf.ValueTypeNames.Float, custom=True
        ).Set(GRIPPER_DRIVE_MAX_FORCE)

    # PR #21 models the real one-motor/two-rack linkage with a PhysX mimic
    # joint. That schema is version-sensitive for prismatic joints: on some
    # Isaac Sim runtime versions it can lock both fingers at an end stop.
    # The teleop controller therefore implements the same 1:1 linkage by
    # applying one shared target to these two equally configured drives.


def _add_gripper_physics_material(stage: Usd.Stage) -> None:
    """Give both finger colliders enough friction to hold the vials."""
    material = UsdShade.Material.Define(
        stage, GRIPPER_PHYSICS_MATERIAL_PATH
    )
    physics_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_api.CreateStaticFrictionAttr(GRIPPER_STATIC_FRICTION)
    physics_api.CreateDynamicFrictionAttr(GRIPPER_DYNAMIC_FRICTION)
    physics_api.CreateRestitutionAttr(0.0)
    physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_api.CreateFrictionCombineModeAttr(PhysxSchema.Tokens.max)
    physx_api.CreateRestitutionCombineModeAttr(PhysxSchema.Tokens.min)
    _enable_patch_friction(physx_api)

    for path in GRIPPER_COLLIDER_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Finger collider prim is missing: {path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )


def _configure_arm_physics(stage: Usd.Stage) -> None:
    """Damp the six arm position drives for stable live leader teleop."""
    joints = [stage.GetPrimAtPath(path) for path in ARM_JOINT_PATHS]
    if not all(prim.IsValid() for prim in joints):
        missing = [
            path
            for path, prim in zip(ARM_JOINT_PATHS, joints)
            if not prim.IsValid()
        ]
        raise RuntimeError(f"Arm joints are missing: {missing}")

    for prim, damping, max_velocity in zip(
        joints, ARM_DRIVE_DAMPING, ARM_MAX_VELOCITY
    ):
        drive = UsdPhysics.DriveAPI.Apply(prim, UsdPhysics.Tokens.angular)
        drive.CreateDampingAttr(damping)
        prim.CreateAttribute(
            "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float
        ).Set(max_velocity)


def gripper_physics_is_fixed(stage: Usd.Stage) -> bool:
    """Return whether an opened scene contains the persistent PR #21 fix."""
    for path in GRIPPER_JOINT_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return False
        expected = {
            "drive:linear:physics:stiffness": GRIPPER_DRIVE_STIFFNESS,
            "drive:linear:physics:damping": GRIPPER_DRIVE_DAMPING,
            "drive:linear:physics:maxForce": GRIPPER_DRIVE_MAX_FORCE,
            "physics:upperLimit": GRIPPER_STROKE,
            "physxJoint:maxJointVelocity": GRIPPER_MAX_VELOCITY,
        }
        for name, value in expected.items():
            actual = prim.GetAttribute(name).Get()
            if actual is None or not math.isclose(
                float(actual), value, rel_tol=1e-5, abs_tol=1e-6
            ):
                return False
    material = stage.GetPrimAtPath(GRIPPER_PHYSICS_MATERIAL_PATH)
    if not material.IsValid():
        return False
    expected_material = {
        "physics:staticFriction": GRIPPER_STATIC_FRICTION,
        "physics:dynamicFriction": GRIPPER_DYNAMIC_FRICTION,
        "physics:restitution": 0.0,
    }
    for name, value in expected_material.items():
        actual = material.GetAttribute(name).Get()
        if actual is None or not math.isclose(
            float(actual), value, rel_tol=1e-5, abs_tol=1e-6
        ):
            return False
    if (
        material.GetAttribute("physxMaterial:frictionCombineMode").Get()
        != PhysxSchema.Tokens.max
    ):
        return False
    # Patch friction keeps thin shafts from slipping out of the fingers, but
    # PhysX 110 (Isaac Sim 6.1) dropped the schema option entirely, so only
    # require it on builds whose schema can author it at all.
    if hasattr(PhysxSchema.PhysxMaterialAPI, "CreateImprovePatchFrictionAttr"):
        if (
            material.GetAttribute("physxMaterial:improvePatchFriction").Get()
            is not True
        ):
            return False
    for path in GRIPPER_COLLIDER_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return False
        relationship = prim.GetRelationship("material:binding:physics")
        if (
            not relationship.IsValid()
            or relationship.GetTargets()
            != [Sdf.Path(GRIPPER_PHYSICS_MATERIAL_PATH)]
        ):
            return False

    # A scene generated by an earlier revision may already have the correct
    # gains but still contain the version-sensitive mimic schema.
    return not any(
        schema.startswith("PhysxMimicJointAPI:")
        for path in GRIPPER_JOINT_PATHS
        for schema in stage.GetPrimAtPath(path).GetAppliedSchemas()
    )


def _add_vial_targets(stage: Usd.Stage) -> None:
    """Add three vials as independent dynamic rigid bodies."""
    UsdGeom.Scope.Define(stage, "/World/Objects")
    physics_material = UsdShade.Material.Define(
        stage, STATIONERY_PHYSICS_MATERIAL_PATH
    )
    physics_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
    physics_api.CreateStaticFrictionAttr(STATIONERY_STATIC_FRICTION)
    physics_api.CreateDynamicFrictionAttr(STATIONERY_DYNAMIC_FRICTION)
    physics_api.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(
        physics_material.GetPrim()
    )
    physx_material.CreateFrictionCombineModeAttr(PhysxSchema.Tokens.max)
    physx_material.CreateRestitutionCombineModeAttr(
        PhysxSchema.Tokens.min
    )
    physx_material.CreateDampingCombineModeAttr(PhysxSchema.Tokens.max)
    _enable_patch_friction(physx_material)

    # Spawned with their bases ~40 mm above the work surface; the origin of
    # the vial asset sits 17 mm above its base.
    initial_poses = {
        "vial_1": ((0.07, 0.02, 0.057), 18.0),
        "vial_2": ((0.15, 0.18, 0.057), -32.0),
        "vial_3": ((0.115, 0.095, 0.057), 28.0),
    }
    for name, (path, source_prim, mass_kg) in TARGET_SPECS.items():
        target = UsdGeom.Xform.Define(stage, path)
        target.GetPrim().GetReferences().AddReference(
            VIAL_ASSET, source_prim
        )
        position, yaw_deg = initial_poses[name]
        yaw = math.radians(yaw_deg)
        _set_transform(
            target.GetPrim(),
            translate=Gf.Vec3d(*position),
            orient=Gf.Quatd(
                math.cos(yaw / 2.0),
                Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0)),
            ),
        )
        rigid = UsdPhysics.RigidBodyAPI.Apply(target.GetPrim())
        # Dynamic from the start: the vials are pushed around and lifted,
        # both in teleoperation and when the scene is opened on its own.
        # Only the rack is meant to stay put.
        rigid.CreateKinematicEnabledAttr(False)
        physx_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(target.GetPrim())
        physx_rigid.CreateMaxDepenetrationVelocityAttr(
            OBJECT_MAX_DEPENETRATION_VELOCITY
        )
        physx_rigid.CreateMaxContactImpulseAttr(OBJECT_MAX_CONTACT_IMPULSE)
        mass = UsdPhysics.MassAPI.Apply(target.GetPrim())
        mass.CreateMassAttr(mass_kg)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, VIAL_CENTER_OF_MASS_Z))

        # The vial asset ships its own collider mesh; the visual meshes stay
        # collision-free and keep their own materials.
        collider = stage.GetPrimAtPath(f"{path}/collider")
        if not collider.IsValid():
            raise RuntimeError(f"Vial collider is missing: {path}/collider")
        # The vial asset declares a bare physics MassAPI on this collider with
        # no values authored, so the schema fallbacks apply: mass 0 and an
        # infinite centre of mass.  PhysX cannot build that body — the strict
        # case crashes the simulator — so author finite shape values here.
        collider_mass = UsdPhysics.MassAPI.Apply(collider)
        collider_mass.CreateMassAttr(0.0)
        collider_mass.CreateCenterOfMassAttr(
            Gf.Vec3f(0.0, 0.0, VIAL_CENTER_OF_MASS_Z)
        )
        UsdPhysics.MeshCollisionAPI.Apply(collider).CreateApproximationAttr(
            "convexHull"
        )
        physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(collider)
        physx_collision.CreateContactOffsetAttr(STATIONERY_CONTACT_OFFSET)
        physx_collision.CreateRestOffsetAttr(STATIONERY_REST_OFFSET)
        physx_collision.CreateTorsionalPatchRadiusAttr(
            STATIONERY_TORSIONAL_PATCH_RADIUS
        )
        physx_collision.CreateMinTorsionalPatchRadiusAttr(
            STATIONERY_TORSIONAL_PATCH_RADIUS
        )
        UsdShade.MaterialBindingAPI(collider).Bind(
            physics_material,
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )


def _stationery_material(stage: Usd.Stage) -> UsdShade.Material:
    """Create the Poly Haven stationery PBR material in scene scope."""
    path = "/World/Looks/StationeryPBR"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")

    reader = UsdShade.Shader.Define(stage, f"{path}/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

    def texture(name: str, asset: str, color_space: str) -> UsdShade.Shader:
        node = UsdShade.Shader.Define(stage, f"{path}/{name}")
        node.CreateIdAttr("UsdUVTexture")
        node.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(asset)
        node.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(
            color_space
        )
        node.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result"
        )
        return node

    diffuse = texture(
        "DiffuseTexture",
        "../assets/stationery/textures/stationery_supplies_diff_4k.jpg",
        "sRGB",
    )
    arm = texture(
        "ArmTexture",
        "../assets/stationery/textures/stationery_supplies_arm_4k.jpg",
        "raw",
    )
    normal = texture(
        "NormalTexture",
        "../assets/stationery/textures/stationery_supplies_nor_gl_4k.exr",
        "raw",
    )
    # UsdPreviewSurface remaps the normal input to [-1, 1] itself; adding a
    # manual scale/bias double-transforms it, which scatters per-pixel
    # micro-normals and turns 1-sample RT renders into full-frame fireflies.
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diffuse.ConnectableAPI(), "rgb"
    )
    # Poly Haven's ARM map stores AO, roughness and metalness in RGB.
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
        arm.ConnectableAPI(), "g"
    )
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).ConnectToSource(
        arm.ConnectableAPI(), "b"
    )
    shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
        normal.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _add_pencil_cup(
    stage: Usd.Stage, visual_material: UsdShade.Material
) -> None:
    """Add the source pencil cup as a fixed concave collision receptacle."""
    cup = UsdGeom.Xform.Define(stage, PENCIL_CUP_PATH)
    cup.GetPrim().GetReferences().AddReference(
        STATIONERY_ASSET, "/Stationery/Pencilcup"
    )
    _set_transform(cup.GetPrim(), translate=PENCIL_CUP_CENTER)
    cup_mesh = stage.OverridePrim(f"{PENCIL_CUP_PATH}/Mesh")
    UsdShade.MaterialBindingAPI.Apply(cup_mesh).Bind(
        visual_material, UsdShade.Tokens.strongerThanDescendants
    )
    UsdPhysics.CollisionAPI.Apply(cup_mesh)
    UsdPhysics.MeshCollisionAPI.Apply(cup_mesh).CreateApproximationAttr(
        "none"
    )


def _add_stationery_targets(
    stage: Usd.Stage, visual_material: UsdShade.Material
) -> None:
    """Add two pens and one eraser as independent dynamic rigid bodies."""
    physics_material = UsdShade.Material.Define(
        stage, STATIONERY_PHYSICS_MATERIAL_PATH
    )
    physics_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
    physics_api.CreateStaticFrictionAttr(STATIONERY_STATIC_FRICTION)
    physics_api.CreateDynamicFrictionAttr(STATIONERY_DYNAMIC_FRICTION)
    physics_api.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(
        physics_material.GetPrim()
    )
    physx_material.CreateFrictionCombineModeAttr(PhysxSchema.Tokens.max)
    physx_material.CreateRestitutionCombineModeAttr(
        PhysxSchema.Tokens.min
    )
    physx_material.CreateDampingCombineModeAttr(PhysxSchema.Tokens.max)
    _enable_patch_friction(physx_material)

    # Spawned above the work surface; they drop and settle like the vials.
    initial_poses = {
        "pen_red": ((0.07, 0.02, 0.075), 18.0),
        "pen_blue": ((0.15, 0.18, 0.095), -32.0),
        "eraser": ((0.115, 0.095, 0.055), 28.0),
    }
    for name, (path, source_prim, mass_kg) in STATIONERY_TARGET_SPECS.items():
        target = UsdGeom.Xform.Define(stage, path)
        target.GetPrim().GetReferences().AddReference(
            STATIONERY_ASSET, source_prim
        )
        position, yaw_deg = initial_poses[name]
        yaw = math.radians(yaw_deg)
        _set_transform(
            target.GetPrim(),
            translate=Gf.Vec3d(*position),
            orient=Gf.Quatd(
                math.cos(yaw / 2.0),
                Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0)),
            ),
        )
        rigid = UsdPhysics.RigidBodyAPI.Apply(target.GetPrim())
        # Dynamic from the start, matching the vial targets: only the cup
        # and the enclosure are static.
        rigid.CreateKinematicEnabledAttr(False)
        physx_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(target.GetPrim())
        physx_rigid.CreateMaxDepenetrationVelocityAttr(
            OBJECT_MAX_DEPENETRATION_VELOCITY
        )
        physx_rigid.CreateMaxContactImpulseAttr(OBJECT_MAX_CONTACT_IMPULSE)
        mass = UsdPhysics.MassAPI.Apply(target.GetPrim())
        mass.CreateMassAttr(mass_kg)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0))

        mesh = stage.OverridePrim(f"{path}/Mesh")
        UsdShade.MaterialBindingAPI.Apply(mesh).Bind(
            visual_material, UsdShade.Tokens.strongerThanDescendants
        )
        UsdPhysics.CollisionAPI.Apply(mesh)
        UsdPhysics.FilteredPairsAPI.Apply(mesh).CreateFilteredPairsRel()
        UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(
            "convexHull"
        )
        physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh)
        physx_collision.CreateContactOffsetAttr(
            STATIONERY_TARGET_CONTACT_OFFSET
        )
        physx_collision.CreateRestOffsetAttr(STATIONERY_REST_OFFSET)
        physx_collision.CreateTorsionalPatchRadiusAttr(
            STATIONERY_TORSIONAL_PATCH_RADIUS
        )
        physx_collision.CreateMinTorsionalPatchRadiusAttr(
            STATIONERY_TORSIONAL_PATCH_RADIUS
        )
        UsdShade.MaterialBindingAPI(mesh).Bind(
            physics_material,
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )


def _add_assist_joint(stage: Usd.Stage, variant: str = "vials") -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, ASSIST_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(GRIPPER_BODY_PATH)])
    target_specs = TARGET_SPECS_BY_VARIANT[variant]
    joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(target_specs[ASSIST_JOINT_BODY_TARGET[variant]][0])]
    )
    offset, local_rot = GRASP_JOINT_POSES[variant]
    joint.CreateLocalPos0Attr(offset)
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr(local_rot)
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
    joint.CreateJointEnabledAttr(False)
    joint.GetPrim().SetCustomDataByKey("purpose", "reliable_demo_grasp_assist")
    joint.GetPrim().SetActive(False)


def _add_gripper_collision_filter(stage: Usd.Stage) -> None:
    group_path = Sdf.Path("/World/GripperFingerNoCollideGroup")
    group = UsdPhysics.CollisionGroup.Define(stage, group_path)
    colliders = group.GetCollidersCollectionAPI()
    includes = colliders.CreateIncludesRel()
    includes.SetTargets(
        [
            Sdf.Path(f"{ROBOT_PATH}/gripper_left"),
            Sdf.Path(f"{ROBOT_PATH}/gripper_right"),
        ]
    )
    group.CreateFilteredGroupsRel().AddTarget(group_path)


def build_scene(
    output: Path = DEFAULT_OUTPUT,
    variant: str = "vials",
    render_mode: str | None = None,
) -> Path:
    if variant not in TARGET_SPECS_BY_VARIANT:
        raise ValueError(
            f"Unknown scene variant: {variant!r}; "
            f"expected one of {sorted(TARGET_SPECS_BY_VARIANT)}"
        )
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    # Rebuilding from inside a running app (the teleop staleness path) has
    # the scene already loaded as a layer, so CreateNew would fail with
    # "A layer already exists with identifier".  Empty and reuse that layer,
    # leaving the file on disk alone until the rebuild saves over it.
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
        # Isaac reads the requested renderer from the layer's custom data
        # (the GUI writes the same key when a mode is picked in the
        # viewport). Used by the still-capture flow, which needs the
        # accumulated Path Tracing result rather than 1-sample Real-Time.
        layer_data = dict(stage.GetRootLayer().customLayerData)
        layer_data["renderSettings"] = {"rtx:rendermode": render_mode}
        stage.GetRootLayer().customLayerData = layer_data

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Looks")
    UsdGeom.Scope.Define(stage, "/World/Lights")

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
    stationery_material = None
    if variant == "stationery":
        stationery_material = _stationery_material(stage)
        _add_pencil_cup(stage, stationery_material)
    else:
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

    _add_robot(stage)
    _add_robot_appearance(stage)
    _configure_arm_physics(stage)
    _configure_gripper_physics(stage)
    _add_gripper_physics_material(stage)
    if variant == "stationery":
        _add_stationery_targets(stage, stationery_material)
    else:
        _add_vial_targets(stage)
    _add_assist_joint(stage, variant)
    _add_gripper_collision_filter(stage)
    _add_lighting(stage)
    _add_cameras(stage)

    stage.GetRootLayer().Save()
    print(f"[scene] Generated: {output}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        choices=sorted(TARGET_SPECS_BY_VARIANT),
        default="vials",
        help="Scene content to generate: vials+vial rack (default) or "
        "pens+eraser with a fixed pencil cup",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or (
        DEFAULT_OUTPUT_STATIONERY
        if args.variant == "stationery"
        else DEFAULT_OUTPUT
    )
    try:
        build_scene(output, variant=args.variant)
    finally:
        if _simulation_app is not None:
            _simulation_app.close()


if __name__ == "__main__":
    main()
