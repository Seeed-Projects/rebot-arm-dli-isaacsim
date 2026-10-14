#!/usr/bin/env python3
"""Verify the ported Newton reBot DevArm asset.

Checks that assets/newton_rebot_devarm/usd_structured/seeed_rebot_devarm.usda
composes correctly (payload, meshes, joints, gripper coupling, filtered
pairs, materials, units, bounds) both when opened directly and when
referenced into a stage, which is how our scenes consume assets.

Run with the Isaac Sim python:

    "${ISAACSIM_ROOT:-$HOME/下载/isaac-sim-standalone-6.1.0-linux-x86_64}/python.sh" \
        scripts/verify_newton_asset.py [--render]

Exit code 0 = all checks passed.  --render additionally writes a preview
image (path printed at the end); it needs the GPU and may contend with a
running GUI.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp

_simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ENTRY = (
    PROJECT_ROOT
    / "assets"
    / "newton_rebot_devarm"
    / "usd_structured"
    / "seeed_rebot_devarm.usda"
)
PREVIEW_PATH = Path("/tmp/newton_rebot_preview/newton_rebot.png")

EXPECTED_JOINTS = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "joint_left", "joint_right",
}
EXPECTED_BBOX_MM = (561.3, 200.0, 320.8)  # tolerance below
BBOX_TOL_MM = 2.0

_failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    tag = "PASS" if ok else "FAIL"
    line = f"[verify] {tag}  {name}"
    if detail:
        line += f" — {detail}"
    print(line)
    if not ok:
        _failures.append(name)


def _bbox_mm(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
    return (hi - lo) * 1000.0, lo * 1000.0


def composition_checks() -> None:
    check("entry file exists", ASSET_ENTRY.is_file(), str(ASSET_ENTRY))
    stage = Usd.Stage.Open(str(ASSET_ENTRY))
    root = stage.GetDefaultPrim()
    check("defaultPrim present", bool(root and root.IsValid()))
    if not root or not root.IsValid():
        return

    meshes = [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Mesh)]
    check("payload composed (meshes)", len(meshes) > 0, f"{len(meshes)} meshes")

    joints = [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
    names = {p.GetName() for p in joints}
    missing = EXPECTED_JOINTS - names
    check("expected joints present", not missing, f"found {sorted(names)}")

    left = stage.GetPrimAtPath(f"{root.GetPath()}/Geometry/base_link/link1/"
                               "link2/link3/link4/link5/link6/"
                               "gripper_end/gripper_left/joint_left")
    if not left or not left.IsValid():
        left = next(
            (p for p in Usd.PrimRange(root) if p.GetName() == "joint_left"), None
        )
    schemas = left.GetAppliedSchemas() if left else []
    check(
        "gripper coupling (NewtonMimicAPI)",
        "NewtonMimicAPI" in schemas and "MjcEqualityJointAPI" in schemas,
        f"joint_left schemas: {schemas}",
    )

    filtered = [
        p for p in Usd.PrimRange(root) if p.HasRelationship("physics:filteredPairs")
    ]
    check("filtered pairs authored", len(filtered) >= 9, f"{len(filtered)} bodies")

    materials = [p for p in Usd.PrimRange(root) if p.IsA(UsdShade.Material)]
    check("materials present", len(materials) > 0, f"{len(materials)} materials")

    size, lo = _bbox_mm(root)
    want = np.array(EXPECTED_BBOX_MM)
    ok = bool(np.all(np.abs(size - want) < BBOX_TOL_MM)) and abs(float(lo[2])) < 1.0
    check(
        "bounds in metres, base at z=0",
        ok,
        f"bbox {np.round(size, 1).tolist()} mm, min {np.round(lo, 1).tolist()} "
        f"(expected {list(want)}, base 0)",
    )
    check(
        "up axis Z / metersPerUnit 1",
        UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
        and UsdGeom.GetStageMetersPerUnit(stage) == 1.0,
    )


def reference_checks() -> None:
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    robot = UsdGeom.Xform.Define(stage, "/World/Robot")
    robot.GetPrim().GetReferences().AddReference(str(ASSET_ENTRY))
    # The entry carries a defaultPrim, so its content merges into the
    # referencing prim itself; there is no extra child prim.
    prim = stage.GetPrimAtPath("/World/Robot")
    ok = prim.IsValid() and bool([p for p in Usd.PrimRange(prim)
                                  if p.IsA(UsdGeom.Mesh)])
    check("references compose into the referencing prim", ok)


def render_preview() -> None:
    import cv2  # noqa: PLC0415
    import omni.replicator.core as rep  # noqa: PLC0415
    from omni.usd import get_context  # noqa: PLC0415

    scene = PREVIEW_PATH.parent / "_verify.usda"
    PREVIEW_PATH.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(scene))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    robot = UsdGeom.Xform.Define(stage, "/World/Robot")
    robot.GetPrim().GetReferences().AddReference(str(ASSET_ENTRY))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(robot.GetPrim()).ComputeAlignedRange()
    lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
    centre = (lo + hi) / 2.0
    radius = float(np.linalg.norm(hi - lo) / 2.0)
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(700.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Key")
    key.CreateIntensityAttr(900.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(40.0, 0.0, -30.0))
    cam = UsdGeom.Camera.Define(stage, "/World/Cam")
    cam.CreateFocalLengthAttr(32.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
    eye = [
        centre[0] + 1.9 * radius,
        centre[1] - 1.9 * radius,
        centre[2] + 1.1 * radius,
    ]
    view = (
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*centre), Gf.Vec3d(0, 0, 1))
        .GetInverse()
    )
    xform = UsdGeom.Xformable(cam)
    xform.ClearXformOpOrder()
    xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(view)
    stage.GetRootLayer().Save()

    get_context().open_stage(str(scene))
    for _ in range(40):
        _simulation_app.update()
    rep.settings.set_render_pathtraced(samples_per_pixel=96)
    product = rep.create.render_product("/World/Cam", (500, 380))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(product)
    for _ in range(20):
        _simulation_app.update()
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=16)
        for _ in range(6):
            _simulation_app.update()
        data = annotator.get_data()
        if hasattr(data, "numpy"):
            data = data.numpy()
        data = np.asarray(data).astype(np.uint8)
        if data.size and float(data[:, :, :3].mean()) > 2.0:
            cv2.imwrite(
                str(PREVIEW_PATH),
                cv2.cvtColor(data[:, :, :3], cv2.COLOR_RGB2BGR),
            )
            print(f"[verify] preview written to {PREVIEW_PATH}")
            return
    print("[verify] preview skipped (no rendered frame; GPU busy?)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true",
                        help="also write a preview render (needs the GPU)")
    args = parser.parse_args()
    try:
        composition_checks()
        reference_checks()
        if args.render:
            render_preview()
    finally:
        print(
            "[verify] RESULT:",
            "ALL CHECKS PASSED" if not _failures else f"FAILED: {_failures}",
        )
        _simulation_app.close()
    return 1 if _failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
