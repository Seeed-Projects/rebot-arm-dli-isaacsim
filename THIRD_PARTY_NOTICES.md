# Third-party notices

The root MIT License applies only to source code and documentation authored for
this project. It does not replace licenses attached to third-party assets.

## reBot B601-RS robot assets

Files under `assets/rebot/` and `assets/rebot_physx/` are converted reBot arm
USD assets derived from Seeed Studio material. The implementation was developed
with reference to:

- <https://github.com/Seeed-Projects/reBot-Isaacsim>
- <https://wiki.seeedstudio.com/rebot_arm_b601_rs_isaacsim/>

Confirm the upstream repository's current asset license and redistribution
terms before publishing these files. If redistribution is not permitted,
exclude these directories from the public repository and document how users
can obtain and convert the robot asset locally.

## UVC32 camera mount

Files under `assets/camera/` are derived from Seeed Studio's
`UVC32_mount.step` design:

<https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/3D_Printed_Parts/UVC32_mount.step>

The source repository identifies its hardware designs as CERN Open Hardware
Licence Version 2 — Weakly Reciprocal (CERN-OHL-W-2.0). See
`assets/camera/README.md` for conversion details. Preserve all notices and
comply with the source license when redistributing modified design files.

## Newton reBot DevArm assets

Files under `assets/newton_rebot_devarm/` are a verbatim copy of the
`seeed_rebot_devarm` package from
[newton-physics/newton-assets](https://github.com/newton-physics/newton-assets)
(commit `a0547548eaa966c2f5478bee496c3cfba1fa98fc`), ported on 2026-10-13. The
upstream package is released under the MIT License (carried verbatim in
`assets/newton_rebot_devarm/LICENSE`); it derives from the Seeed-Projects
reBot-Isaacsim URDF and the MuJoCo Menagerie MJCF of the same robot. It is a
Newton/MuJoCo-stack asset and is not used by the PhysX teleoperation pipeline;
see `assets/newton_rebot_devarm/PORTING_NOTES.md`.

## Stationery Supplies

Files under `assets/stationery/` are derived from Poly Haven's Stationery
Supplies asset by Mateusz Sadek:

<https://polyhaven.com/a/stationery_supplies>

Poly Haven publishes the asset under CC0:

<https://polyhaven.com/license>

See `assets/stationery/README.md` for provenance and conversion details.

## Workspace enclosure

`assets/workspace/box.usdz` was supplied separately by the project owner. No
license information accompanied the file. Redistribution rights must be
confirmed before including it in a public GitHub repository. See
`assets/workspace/README.md` for its recorded checksum and provenance.

## Legacy banana asset

`assets/banana/011_banana.usd` is retained only for historical compatibility
and is not referenced by the active stationery-sorting scene. Its source and
license are not documented in this repository. Confirm redistribution rights
or omit this unused directory before publishing.

## NVIDIA Isaac Sim and LeRobot

NVIDIA Isaac Sim and LeRobot are external runtime dependencies and are not
distributed by the root MIT License. Their own licenses and terms apply.
