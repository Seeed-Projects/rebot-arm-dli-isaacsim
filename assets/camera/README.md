# UVC32 wrist-camera mount asset

`UVC32_mount.usd` is a USD conversion of Seeed Studio's `UVC32_mount.step`
mechanical design for the reBot B601 wrist camera, revision
`332000147,uvc32_mount_dyx0922` (SolidWorks 2023, exported 2026-09-22).
`UVC32_mount_portable.usd` contains the same geometry with the CAD material
converted to UsdPreviewSurface instead of MDL; the simulation references this
portable copy so it has no dependency on an Isaac Sim material-library search
path.

- Source:
  <https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/camera-mounts/b601-camera-mounts/UVC32_mount.step>
- Source repository hardware license: CERN Open Hardware Licence Version 2 —
  Weakly Reciprocal (CERN-OHL-W-2.0)
- Conversion: `scripts/convert_camera_asset.py` (or `--mdl` for this file),
  which drives the NVIDIA HOOPS CAD converter at tessellation LOD 5, Z-up,
  metres, without physics data
- Converted bounds: approximately 38.4 × 101.7 × 61.0 mm

Regenerate with:

```bash
~/isaacsim/python.sh scripts/convert_camera_asset.py UVC32_mount.step \
    assets/camera/UVC32_mount_portable.usd
```

## Frame convention

The USD keeps the CAD datum: the part's origin sits 23.39 mm in front of the
axis of the Ø57 mm clamp and 24.31 mm above the centre of the clamp's bolt
holes. Seeed's published assembly, by contrast, places a mesh whose origin is
at that bolt-hole centre — `Camera/scripts/build-uvc32-assets.py` builds it by
translating the model. `scripts/build_scene.py` therefore carries both the
official placement (`Rebot_Arm_description/Camera/urdf/uvc32.urdf` relative to
`Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`) and the 7.110 / 24.310 mm
offset between the two conventions, so the clamp ends up coaxial with the
wrist axis.

The scene adds a separate, render-only 32 mm UVC board, lens housing and lens
glass; none of these camera visuals have collision or mass, so they do not
alter arm dynamics.

## External data-collection camera stand

`data_collection_camera_mount.usd` is the converted
`data-collection-camera-mount-v2.stp` frame used for the overhead camera: a
590 mm mast built from four 150 mm 2020 extrusions on a 130 x 105 mm foot,
with a camera arm reaching 104 mm to one side. The CAD models the mast along
+Y and the arm along -Z, so `build_scene.py` rotates it 90 degrees about X to
stand it on the work surface (`EXTERNAL_CAMERA_MOUNT_*`). It carries no
collision or mass.

Regenerate with:

```bash
~/isaacsim/python.sh scripts/convert_camera_asset.py \
    data-collection-camera-mount-v2.stp \
    assets/camera/data_collection_camera_mount.usd
```
