#!/usr/bin/env python3
"""Convert a camera-mount CAD file into the project's USD asset format.

The HOOPS CAD converter only exists once Kit is running, so this script must
run under Isaac Sim's Python:

    ~/isaacsim/python.sh scripts/convert_camera_asset.py mount.step out.usd

The conversion is tuned for a render-only part: no physics data (collision or
mass), a single self-contained file, Z-up, and the CAD colours kept as
UsdPreviewSurface materials so the scene needs no MDL library search path.
"""

from __future__ import annotations

import re

import argparse
from pathlib import Path

# Tessellation fine enough that the printed fillets read as curves; the
# converter's default (2) leaves visible facets on this part.
TESSELLATION_LOD = 5

def _default_root_prim(source: Path) -> str:
    """A valid prim name derived from the source file's own name."""
    stem = re.sub(r"[^0-9A-Za-z_]", "_", source.stem)
    if not stem or stem[0].isdigit():
        stem = f"part_{stem}"
    return stem


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="STEP/IGES/JT source file")
    parser.add_argument("output", type=Path, help="destination USD file")
    parser.add_argument(
        "--compare",
        type=Path,
        action="append",
        default=[],
        help="existing USD to report bounds for, so a swap can be checked",
    )
    parser.add_argument(
        "--root-prim",
        default=None,
        help="name for the converted part's root prim (defaults to the "
        "source file name)",
    )
    parser.add_argument(
        "--no-materials",
        action="store_true",
        help="convert geometry only, without any material",
    )
    parser.add_argument(
        "--mdl",
        action="store_true",
        help="keep the converter's MDL materials instead of UsdPreviewSurface",
    )
    return parser.parse_args()


def _report(path: Path) -> None:
    """Print the stage units, default prim and size of a camera-mount USD."""
    from pxr import Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(str(path.resolve()))
    if stage is None:
        raise RuntimeError(f"Cannot open {path}")
    default_prim = stage.GetDefaultPrim()
    print(f"[asset] {path}")
    print(f"[asset]   metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
    print(
        f"[asset]   defaultPrim: "
        f"{default_prim.GetPath() if default_prim else '<none>'}"
    )
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    )
    bounds = cache.ComputeWorldBound(default_prim).ComputeAlignedRange()
    size = bounds.GetSize()
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    scale = 1000.0 * UsdGeom.GetStageMetersPerUnit(stage)
    print(
        "[asset]   size (mm): "
        f"({size[0] * scale:.2f}, {size[1] * scale:.2f}, {size[2] * scale:.2f})"
    )
    print(
        "[asset]   min (mm): "
        f"({minimum[0] * scale:.2f}, {minimum[1] * scale:.2f}, "
        f"{minimum[2] * scale:.2f})"
    )
    print(
        "[asset]   max (mm): "
        f"({maximum[0] * scale:.2f}, {maximum[1] * scale:.2f}, "
        f"{maximum[2] * scale:.2f})"
    )
    meshes = 0
    faces = 0
    materials = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            meshes += 1
            faces += len(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get() or [])
        elif prim.IsA(UsdShade.Material):
            materials += 1
    print(f"[asset]   meshes={meshes} faces={faces} materials={materials}")


def _options(no_materials: bool, mdl: bool) -> dict[str, str]:
    """Converter arguments: a single render-only file with CAD colours."""
    return {
        "iTessLOD": str(TESSELLATION_LOD),
        "bUseMaterials": "false" if no_materials else "true",
        "bUseNormals": "true",
        "bDedup": "true",
        "bAccurateSurfaceCurvatures": "true",
        "bConvertCurves": "false",
        "convertPhysicsData": "false",
        # eNone: one monolithic file instead of references to sub-components.
        "compositionStyle": "0",
        "instancing": "false",
        # The scene's stage runs in metres, and USD does not rescale a
        # reference whose units differ, so the part has to be converted to
        # metres rather than left in the CAD file's millimetres.
        "dMetersPerUnit": "1.0",
        # ePreviewSurface (1) keeps materials usable without an MDL search
        # path; ePreviewSurface_OmniPBR (2) keeps the converter's MDL
        # bindings for the asset that is not referenced by the scene.
        "materialType": "2" if mdl else "1",
        # eZup (2) matches the scene's stage up axis.
        "upAxis": "2",
    }


def _bake_units(root, units: float) -> None:
    """Fold a unit conversion into the part's geometry and transforms.

    USD does not rescale a reference whose ``metersPerUnit`` differs, so the
    CAD file's millimetres have to become metres here.  A scale op on the
    part root would do that for a bare reference but not for this scene:
    ``build_scene`` authors its own ``xformOpOrder`` on the mount prim, and
    because ``xformOpOrder`` is a single strongest-wins attribute, an op the
    scene does not list (the scale) is silently dropped and the part renders
    1000x too large.  Scaling the data itself has no such failure mode.
    """
    from pxr import Usd, UsdGeom

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if points:
                mesh.GetPointsAttr().Set(
                    type(points)([point * units for point in points])
                )
            extent = mesh.GetExtentAttr().Get()
            if extent:
                mesh.GetExtentAttr().Set(
                    type(extent)([corner * units for corner in extent])
                )
        if prim.IsA(UsdGeom.Cube):
            cube = UsdGeom.Cube(prim)
            size = cube.GetSizeAttr().Get()
            if size is not None:
                cube.GetSizeAttr().Set(size * units)
        if prim.IsA(UsdGeom.Sphere):
            sphere = UsdGeom.Sphere(prim)
            radius = sphere.GetRadiusAttr().Get()
            if radius is not None:
                sphere.GetRadiusAttr().Set(radius * units)
        if prim.IsA(UsdGeom.Cylinder):
            cylinder = UsdGeom.Cylinder(prim)
            for attribute in (cylinder.GetRadiusAttr(), cylinder.GetHeightAttr()):
                value = attribute.Get()
                if value is not None:
                    attribute.Set(value * units)

        if not prim.IsA(UsdGeom.Xformable):
            continue
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            name = op.GetOpName()
            if name == "xformOp:translate":
                value = op.Get()
                if value is not None:
                    op.Set(value * units)
            elif name == "xformOp:transform":
                matrix = op.Get()
                if matrix is not None:
                    # Row-vector convention: the translation lives in row 3,
                    # and S*T*S^-1 only rescales that row.
                    row = matrix.GetRow(3)
                    matrix.SetRow(
                        3,
                        type(row)(
                            row[0] * units, row[1] * units, row[2] * units, row[3]
                        ),
                    )
                    op.Set(matrix)


def _normalize(path: Path, root_prim_name: str) -> None:
    """Give the converted part a clean name and metres as its unit."""
    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))
    default_prim = stage.GetDefaultPrim()
    if default_prim is None:
        raise RuntimeError(f"Converted asset has no default prim: {path}")
    current_name = default_prim.GetName()
    if current_name != root_prim_name:
        renamed = Sdf.Path(f"/{root_prim_name}")
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(default_prim.GetPath(), renamed)
        if not stage.GetRootLayer().Apply(edit):
            raise RuntimeError(f"Failed to rename part root: {current_name}")
        default_prim = stage.GetPrimAtPath(renamed)
        stage.SetDefaultPrim(default_prim)

    units = UsdGeom.GetStageMetersPerUnit(stage)
    if units != 1.0:
        _bake_units(default_prim, units)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.GetRootLayer().Save()


def _convert(args: argparse.Namespace) -> None:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        _run(args)
    except BaseException:
        # Kit's fast shutdown discards buffered output and pending tracebacks,
        # so report the failure while the interpreter is still alive.
        import traceback

        traceback.print_exc()
    finally:
        import sys

        sys.stdout.flush()
        sys.stderr.flush()
        app.close()


def _run(args: argparse.Namespace) -> None:
    from isaacsim.core.experimental.utils.app import enable_extension

    # The bundle registers the HOOPS backend that reads CAD formats.
    enable_extension("omni.kit.converter.cad")

    # Imported here: this module needs pxr, which Kit only exposes once
    # SimulationApp has started.
    from make_camera_mount_portable import strip_mdl
    from omni.converter.hoops import Converter, Parameters

    parameters = Parameters()
    parameters.parseArgs(_options(args.no_materials, args.mdl))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    intermediate = args.output.with_suffix(".raw.usd")
    converter = Converter(parameters)
    result = converter.convert(
        str(args.source.resolve()), str(intermediate.resolve())
    )
    status, message = result if isinstance(result, tuple) else (result, "")
    print(f"[asset] convert status={status!r} message={message!r}", flush=True)
    # The status enum's numeric values are not documented, so judge the
    # conversion by its artifact.
    if not intermediate.is_file() or intermediate.stat().st_size == 0:
        raise RuntimeError(f"Conversion failed: {args.source} ({message})")

    if args.mdl:
        intermediate.replace(args.output)
        removed = 0
    else:
        # A CAD export can embed MDL shader prims even in preview-surface
        # mode; the scene must not depend on the MDL search path.
        removed = strip_mdl(intermediate, args.output)
        intermediate.unlink()
    _normalize(args.output, args.root_prim or _default_root_prim(args.source))
    print(
        f"[asset] converted {args.source} -> {args.output} "
        f"(removed {removed} MDL shaders)",
        flush=True,
    )

    _report(args.output)
    for other in args.compare:
        _report(other)


if __name__ == "__main__":
    _convert(_arguments())
