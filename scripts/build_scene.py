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
ROBOT_ASSET = "../assets/rebot_physx/00-arm-rs_asm-v3.usd"
STATIONERY_ASSET = "../assets/stationery/stationery_models.usd"
CAMERA_MOUNT_ASSET = "../assets/camera/UVC32_mount_portable.usd"
WORKSPACE_ASSET = "../assets/workspace/box.usdz"

ROBOT_PATH = "/World/Robot"
WORKSPACE_PATH = "/World/WorkspaceBox"
WORKSPACE_COLLIDER_PATH = (
    f"{WORKSPACE_PATH}/box/Base_08q/tn__131_m6/tn__13_/Mesh"
)
WORKSPACE_LIGHT_PATH = f"{WORKSPACE_PATH}/box/DiskLight"
TARGET_SPECS = {
    "pen_red": ("/World/Objects/PenRed", "/Stationery/PenRed", 0.012),
    "pen_blue": ("/World/Objects/PenBlue", "/Stationery/PenBlue", 0.012),
    "eraser": ("/World/Objects/Eraser", "/Stationery/Eraser", 0.025),
}
PENCIL_CUP_PATH = "/World/Environment/PencilCup"
GRIPPER_BODY_PATH = f"{ROBOT_PATH}/link6"
ASSIST_JOINT_PATH = "/World/GraspAssistJoint"
SIDE_CAMERA_PATH = "/World/Cameras/Side"
FRONT_CAMERA_PATH = f"{GRIPPER_BODY_PATH}/FrontCamera"
WRIST_CAMERA_MOUNT_PATH = f"{GRIPPER_BODY_PATH}/UVC32Mount"
WRIST_CAMERA_BOARD_PATH = f"{GRIPPER_BODY_PATH}/UVC32Camera"
STATIONERY_PHYSICS_MATERIAL_PATH = "/World/Looks/StationeryPhysics"
GRIPPER_PHYSICS_MATERIAL_PATH = "/World/Looks/GripperPhysics"
GRIPPER_COLLIDER_PATHS = (
    f"{ROBOT_PATH}/gripper_left/collisions",
    f"{ROBOT_PATH}/gripper_right/collisions",
)
PENCIL_CUP_CENTER = Gf.Vec3d(-0.16, 0.12, 0.0365904756)

# The official Seeed UVC32 mount places the 32 mm camera board on a plane
# pitched 15 degrees toward the gripper centreline. The STEP's +Y extension
# is rotated onto link6 +X (above the gripper in the real assembly), and its
# origin is seated on motor_7's front face rather than at link6's rear face.
WRIST_CAMERA_PITCH_DEG = 15.0
WRIST_CAMERA_MOUNT_YAW_DEG = -90.0
WRIST_CAMERA_MOUNT_Z = 0.09
WRIST_CAMERA_BOARD_CENTER = Gf.Vec3d(0.0685, 0.0, 0.08444)
WRIST_CAMERA_EYE = Gf.Vec3d(0.065394, 0.0, 0.096031)
# The recorded grab_tube_0 front view centres the work area between the two
# fingertips, so the optical axis is angled slightly further inward than the
# bare 15-degree mounting plane.
WRIST_CAMERA_TARGET = Gf.Vec3d(0.0, 0.0, 0.235)
WRIST_CAMERA_UP = Gf.Vec3d(0.9048, 0.0, 0.4258)

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
# Thin pen shafts need a small, explicit contact envelope.  The default is
# chosen from shape extent and was inconsistent between the imported pen hull
# and the gripper CAD, occasionally allowing a one-frame missed contact.
STATIONERY_CONTACT_OFFSET = 0.002
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
GRASP_OFFSET = Gf.Vec3f(0.0, 0.0, 0.09121)
GRASP_LOCAL_ROT = Gf.Quatf(
    0.0, Gf.Vec3f(-0.7071068, 0.0, -0.7071068)
)


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
    normal.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(2.0, 2.0, 2.0, 1.0)
    )
    normal.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(-1.0, -1.0, -1.0, 0.0)
    )
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
    mount_yaw = math.radians(WRIST_CAMERA_MOUNT_YAW_DEG)
    mount_orientation = Gf.Quatd(
        math.cos(mount_yaw / 2.0),
        Gf.Vec3d(0.0, 0.0, math.sin(mount_yaw / 2.0)),
    )
    _set_transform(
        mount.GetPrim(),
        translate=Gf.Vec3d(0.0, 0.0, WRIST_CAMERA_MOUNT_Z),
        orient=mount_orientation,
    )
    UsdShade.MaterialBindingAPI.Apply(mount.GetPrim()).Bind(
        mount_material, UsdShade.Tokens.strongerThanDescendants
    )

    pitch = math.radians(WRIST_CAMERA_PITCH_DEG)
    board_pitch_orientation = Gf.Quatd(
        math.cos(pitch / 2.0),
        Gf.Vec3d(math.sin(pitch / 2.0), 0.0, 0.0),
    )
    board_orientation = mount_orientation * board_pitch_orientation
    board = UsdGeom.Cube.Define(stage, WRIST_CAMERA_BOARD_PATH)
    board.CreateSizeAttr(1.0)
    _set_transform(
        board.GetPrim(),
        translate=WRIST_CAMERA_BOARD_CENTER,
        orient=board_orientation,
        scale=Gf.Vec3d(0.032, 0.032, 0.003),
    )
    UsdShade.MaterialBindingAPI(board.GetPrim()).Bind(pcb_material)

    normal = Gf.Vec3d(-math.sin(pitch), 0.0, math.cos(pitch))
    housing_center = WRIST_CAMERA_BOARD_CENTER + normal * 0.0045
    housing = UsdGeom.Cylinder.Define(
        stage, f"{WRIST_CAMERA_BOARD_PATH}/LensHousing"
    )
    housing.CreateAxisAttr(UsdGeom.Tokens.z)
    housing.CreateRadiusAttr(0.0065)
    housing.CreateHeightAttr(0.006)
    _set_transform(
        housing.GetPrim(),
        translate=housing_center,
        orient=board_orientation,
    )
    UsdShade.MaterialBindingAPI(housing.GetPrim()).Bind(housing_material)

    lens_center = WRIST_CAMERA_BOARD_CENTER + normal * 0.0080
    lens = UsdGeom.Cylinder.Define(
        stage, f"{WRIST_CAMERA_BOARD_PATH}/LensGlass"
    )
    lens.CreateAxisAttr(UsdGeom.Tokens.z)
    lens.CreateRadiusAttr(0.0047)
    lens.CreateHeightAttr(0.001)
    _set_transform(
        lens.GetPrim(),
        translate=lens_center,
        orient=board_orientation,
    )
    UsdShade.MaterialBindingAPI(lens.GetPrim()).Bind(lens_material)


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

    _add_uvc32_camera_model(stage)

    # This camera is authored below link6, so its pose follows the wrist. Its
    # optical centre follows the real +X-above-gripper placement.
    # USD cameras look down local -Z with +Y as image-up.
    _look_at_camera(
        stage,
        FRONT_CAMERA_PATH,
        eye=WRIST_CAMERA_EYE,
        target=WRIST_CAMERA_TARGET,
        up=WRIST_CAMERA_UP,
        # The grab_tube_0 front stream uses a strong ultra-wide/fisheye view;
        # this rectilinear approximation matches its fingertip scale (~136°).
        focal_length=4.2,
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


def _add_workspace_box(stage: Usd.Stage) -> None:
    """Reference the complete USDZ enclosure and add static collision."""
    workspace = UsdGeom.Xform.Define(stage, WORKSPACE_PATH)
    workspace.GetPrim().GetReferences().AddReference(WORKSPACE_ASSET)
    # CAD work-surface top is z=0.49846 m; align it with the robot base.
    _set_transform(
        workspace.GetPrim(), translate=Gf.Vec3d(0.0, 0.0, -0.49846)
    )
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
    # The transformed USDZ enclosure has a measured lower bound of
    # z=-0.00300286 m.  Matching that value avoids a visible floating gap.
    table_top_z = -0.00300286194
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
    """Give both finger colliders enough friction to hold thin stationery."""
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
    physx_api.CreateImprovePatchFrictionAttr(True)

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
    if material.GetAttribute("physxMaterial:improvePatchFriction").Get() is not True:
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


def _add_stationery_targets(
    stage: Usd.Stage, visual_material: UsdShade.Material
) -> None:
    """Add two pens and one eraser as independent dynamic rigid bodies."""
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
    physx_material.CreateImprovePatchFrictionAttr(True)

    initial_poses = {
        "pen_red": ((0.07, 0.02, 0.075), 18.0),
        "pen_blue": ((0.15, 0.18, 0.095), -32.0),
        "eraser": ((0.115, 0.095, 0.055), 28.0),
    }
    for name, (path, source_prim, mass_kg) in TARGET_SPECS.items():
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
        rigid.CreateKinematicEnabledAttr(True)
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
        physx_collision.CreateContactOffsetAttr(STATIONERY_CONTACT_OFFSET)
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


def _add_assist_joint(stage: Usd.Stage) -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, ASSIST_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(GRIPPER_BODY_PATH)])
    joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(TARGET_SPECS["pen_red"][0])]
    )
    joint.CreateLocalPos0Attr(GRASP_OFFSET)
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr(GRASP_LOCAL_ROT)
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


def build_scene(output: Path = DEFAULT_OUTPUT) -> Path:
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()

    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(120.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(1200.0)

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
    stationery_material = _stationery_material(stage)
    _add_workspace_box(stage)
    _add_table(stage, table_top_material, table_leg_material)
    _add_pencil_cup(stage, stationery_material)
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
    _add_stationery_targets(stage, stationery_material)
    _add_assist_joint(stage)
    _add_gripper_collision_filter(stage)
    _add_lighting(stage)
    _add_cameras(stage)

    stage.GetRootLayer().Save()
    print(f"[scene] Generated: {output}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        build_scene(args.output)
    finally:
        if _simulation_app is not None:
            _simulation_app.close()


if __name__ == "__main__":
    main()
