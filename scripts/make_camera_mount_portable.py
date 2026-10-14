#!/usr/bin/env python3
"""Create a portable camera-mount USD without Isaac Sim MDL references."""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Sdf


def strip_mdl(source: Path, output: Path) -> int:
    """Copy ``source`` to ``output`` without its embedded MDL shader prims."""
    source_layer = Sdf.Layer.FindOrOpen(str(source.resolve()))
    if source_layer is None:
        raise RuntimeError(f"Cannot open source layer: {source}")

    output.parent.mkdir(parents=True, exist_ok=True)
    output_layer = Sdf.Layer.CreateNew(str(output.resolve()))
    output_layer.TransferContent(source_layer)

    # CAD conversion embeds the same MDL shader inside normal prims, prototype
    # prims and flattened prototypes.  Traverse the Sdf layer (not a composed
    # UsdStage) so instance/prototype specs are covered as well.
    mdl_shader_paths: list[Sdf.Path] = []

    def collect(path: Sdf.Path) -> None:
        if path.IsPrimPath() and path.name == "MDLShader":
            mdl_shader_paths.append(path)

    output_layer.Traverse(Sdf.Path.absoluteRootPath, collect)
    edits = Sdf.BatchNamespaceEdit()
    for path in sorted(mdl_shader_paths, key=lambda item: len(str(item)), reverse=True):
        edits.Add(path, Sdf.Path.emptyPath)
    if not output_layer.Apply(edits):
        raise RuntimeError("Failed to remove embedded MDL shader prims")

    # Connections to the removed MDL shaders are harmless but unnecessary.
    stale_properties: list[Sdf.PropertySpec] = []

    def collect_stale(path: Sdf.Path) -> None:
        spec = output_layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PropertySpec) and ":mdl:" in spec.name:
            stale_properties.append(spec)

    output_layer.Traverse(Sdf.Path.absoluteRootPath, collect_stale)
    for prop in stale_properties:
        prop.owner.RemoveProperty(prop)

    output_layer.Save()
    return len(mdl_shader_paths)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    removed = strip_mdl(args.source, args.output)
    print(
        f"[asset] wrote {args.output.resolve()} (removed {removed} MDL shaders)"
    )


if __name__ == "__main__":
    main()
