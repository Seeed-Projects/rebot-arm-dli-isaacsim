# SO-ARM101 robot asset (white variant source)

`SO-ARM101-USD-NO-CAMERA.usd` is a verbatim copy of the SO-101 follower
articulation from NVIDIA's [Sim-to-Real SO-101 Workshop]
(https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop)
(`source/sim_to_real_so101/assets/usd/SO-ARM101-USD-NO-CAMERA.usd`),
Apache-2.0, (c) NVIDIA CORPORATION & AFFILIATES. It is the camera-less
robot: this project attaches its own wrist camera to the gripper link.

Facts verified against the copy:

- default prim `/so101_new_calib`, up axis Z, 1 unit = 1 m
- articulation root API on `/so101_new_calib/base`; `root_joint` is a fixed
  joint with **body0 = world**, so the base is anchored wherever the scene
  places that joint's world frame (see `scripts/build_scene_so101.py`,
  which re-authors `localPos0`/`localRot0` accordingly)
- links: `base, shoulder, upper_arm, lower_arm, wrist, gripper, jaw`;
  visual meshes are instanceable internal references to the stage-root
  `/visuals/<link>` scopes
- joints `/so101_new_calib/joints/<Name>` are revolute with angular
  position drives already applied (stiffness 17.8, damping 0.6,
  max force 10) and limits in degrees:

  | joint | sim name | leader joint | range (deg) |
  | --- | --- | --- | --- |
  | base→shoulder | Rotation | shoulder_pan | ±110 |
  | shoulder→upper_arm | Pitch | shoulder_lift | ±100 |
  | upper_arm→lower_arm | Elbow | elbow_flex | −100..90 |
  | lower_arm→wrist | Wrist_Pitch | wrist_flex | ±95 |
  | wrist→gripper | Wrist_Roll | wrist_roll | ±160 |
  | gripper→jaw | Jaw | gripper | −10..100 |

- materials under `/so101_new_calib/Looks`: `material_a_3d_printed`
  (printed parts, orange by default), `material_sts3215` (servo bodies,
  dark grey), `DefaultMaterial` (white). The scene paints the arm white by
  overriding `inputs:diffuse_color_constant` on the printed material, so
  this file stays byte-identical to the upstream workshop asset.
