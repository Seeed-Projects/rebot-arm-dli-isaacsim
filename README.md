# reBot Arm DLI Isaac Sim

An Isaac Sim teleoperation and data-collection project for the Seeed Studio
reBot Arm B601-RS. It adapts NVIDIA's SO-101 Sim-to-Real workflow to a
stationery-sorting task: use a physical reBot Arm 102 Leader to place two pens
and one eraser into a fixed pencil cup.
![overview](./media/overview.gif)
The default application contains no autonomous arm trajectory. The simulated
B601 follows the leader's six arm joints and gripper command, and holds its
last target whenever leader data is unavailable.

> 📖 **Isaac Sim 6.1 数据采集教程（中文）: [TUTORIAL_zh.md](TUTORIAL_zh.md)** —
> 完整的安装、环境配置、采集、可视化与无硬件验证流程。

## Features

- Real-to-Sim control from a reBot Arm 102 Leader at 60 Hz
- 120 Hz Isaac Sim physics and control loop
- Direct arm-joint mirroring for low-latency response
- Gravity-resistant, low-force dual-finger gripper control based on
  [Seeed PR #21](https://github.com/Seeed-Projects/reBot-Isaacsim/pull/21)
- High-friction thin-object contact and configurable near-field grasp assist
- Two recorded cameras: wrist-mounted `front` and fixed `side`
- One unrecorded, freely adjustable Isaac Sim debug viewport
- Domain randomization for object poses, masses, lighting, and the wrist camera
- LeRobot Dataset v3 recording with `front` and `side` images
- Portable, project-local USD assets

## Task

The workspace contains:

- one pencil;
- one ballpoint pen;
- one eraser;
- one fixed pencil cup.

Press `R` to reset the objects. They are spawned above separate areas of the
workspace and fall onto the work surface under gravity. Use the leader arm to
place all three objects into the pencil cup.

## Requirements

- Ubuntu with a working NVIDIA GPU and Isaac Sim 6.1
- Launch isaacsim from `~/isaacsim` by default, `ISAACSIM_ROOT` set to a
  different installation directory
- `git-lfs` installed before cloning (the robot, workspace, and stationery USD
  assets are stored with Git LFS; without it only pointer files are downloaded)
- A reBot Arm 102 Leader connected through USB-UART

## Installation

### IsaacSim

Please refer to the documentation provided by nvidia to install Isaac Sim 6.1:
`https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html`

For a standalone Linux installation, extract the
`isaac-sim-standalone-6.1.0-linux-x86_64.zip` package into `~/isaacsim` and run
the post-install script once:

```bash
unzip isaac-sim-standalone-6.1.0-linux-x86_64.zip -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
```

### Lerobot


```bash
mkdir ~/rebot_lerobot
cd ~/rebot_lerobot
sudo apt update
sudo apt install -y ffmpeg

# Download github repo
git clone https://github.com/Seeed-Projects/lerobot.git
git clone https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102.git
git clone https://github.com/Seeed-Projects/lerobot-robot-seeed-b601.git

# Create virtual environment (Python 3.12)
uv venv --python 3.12 .venv

# Activate environment
source .venv/bin/activate

# Upgrade pip (optional)
uv pip install --upgrade pip

# Install lerobot main project (editable mode, with the dataset extra
# required by the Isaac Sim episode writer and the dataset visualizer)
uv pip install -e "./lerobot[dataset]"

# Add local dependency packages (editable install)
uv pip install -e ./lerobot-teleoperator-rebot-arm-102
uv pip install -e ./lerobot-robot-seeed-b601
uv pip install motorbridge
```

### This project
```bash
sudo apt update
sudo apt install -y git-lfs

cd ~
git clone https://github.com/yuyoujiang/rebot-arm-dli-isaacsim.git
cd rebot-arm-dli-isaacsim
git lfs install
git lfs pull
```

## Leader calibration

The default calibration ID is `rebot_arm_102_leader`. To recalibrate, place the
leader in the zero pose required by Seeed's guide and fully close the gripper:

```bash
cd ~/rebot_lerobot
source .venv/bin/activate

lerobot-calibrate \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader
```

For temporary serial-port access:

```bash
sudo chmod 666 /dev/ttyUSB0
```

For persistent access, add the current user to `dialout`, then log out and back
in:

```bash
sudo usermod -aG dialout "$USER"
```

References:

- [Seeed leader calibration guide](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/#calibrate-the-leader-arm)
- [Seeed reBot Isaac Sim guide](https://wiki.seeedstudio.com/rebot_arm_b601_rs_isaacsim/)

## Quick start

Connect the leader, then run:

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --leader-port /dev/ttyUSB0
```

If only one `/dev/ttyUSB*` device is present, automatic detection is normally sufficient:

```bash
./run.sh
```

To use another Isaac Sim installation:

```bash
ISAACSIM_ROOT=/path/to/isaacsim ./run.sh
```

Do not run another process that uses the same serial port or UDP port. The launcher starts a separate LeRobot Python process and sends leader state to
`udp://127.0.0.1:5005`.

### Keyboard controls

| Key | Action |
| --- | --- |
| `R` | Reset all three objects and apply domain randomization |
| `S` | Start or stop one recorded episode |
| `C` | Cancel the current episode without saving it |

The Isaac Sim viewport must have keyboard focus. Closing Isaac Sim or pressing
`Ctrl+C` also shuts down the leader serial process.

## Workspace layout

The scene uses `assets/workspace/box.usdz` as a self-contained enclosure. Its
opening faces world `+Y`; the robot tail faces the opening and its working
direction points into the enclosure. The robot base is offset 5 cm inward from
the opening edge. Two 4000 K light strips illuminate the workspace from above,
and the enclosure sits on a wooden table with static collision geometry.

The pencil cup is fixed at `(x=-0.16, y=0.12) m`. The stationery items are
independent dynamic rigid bodies.

## Gripper and grasp behavior

Move the gripper close to an object and close the leader gripper. The default
near-field grasp assist selects the nearest pen or eraser using a finite
cylinder or oriented-box surface approximation. Attachment is permitted only
when the gripper closes and the object surface is within `0.040 m` of a sampled
`0.090 m` corridor running from the finger roots toward the fingertips. This
volume tolerates small CAD and fingertip calibration errors without extending
sideways across the workspace. Opening the leader gripper releases the object.

The assist only stabilizes human demonstrations; it never moves the arm by
itself. It preserves the object's measured pose relative to the wrist, filters
only current object-to-finger contacts while attached, clears residual release
velocity, and restores contact after the fingers separate.

Physical contact is configured as follows:

- finger static/dynamic friction: `2.4 / 2.0`;
- stationery static/dynamic friction: `1.8 / 1.5`;
- maximum friction combination and improved patch friction;
- stationery contact offset: `0.002 m`;
- stationery rest offset: `0 m`;
- torsional friction patch radius: `0.002 m`;
- zero restitution;
- object maximum depenetration velocity: `0.25 m/s`;
- object maximum contact impulse: `0.05 N·s`.

The gripper uses two explicit linear drives with a shared 1:1 target:

- stroke: `0.0500 m`;
- stiffness: `1200 N/m`;
- damping: `30 N·s/m`;
- maximum force: `12 N`;
- maximum speed: `0.08 m/s`.

The contact lock activates in the `0.022 m` closing band and prevents the
fingers from driving through a nearby object. Look for `[CONTACT-LOCK]` and
`[GRASP]` in the terminal when a grasp succeeds. If a thin object does not
produce enough physical drive lag for the primary detector, a fallback can
activate only after the measured finger opening is below `0.028 m` and the
object is already inside the grasp corridor. It prints
`[CONTACT-LOCK-FALLBACK]` before `[GRASP]`.

The one-second teleoperation status line reports `gripper_cmd`,
`gripper_actual`, `nearest`, `distance`, `lock`, and `attached`. These fields
make it possible to distinguish leader mapping, physical finger motion,
near-field selection, and attachment failures directly from the terminal.

To disable grasp assist and use only PhysX contact and friction:

```bash
./run.sh --leader-port /dev/ttyUSB0 --strict-physics
```

## Cameras

The GUI exposes three views:

1. the default adjustable Isaac Sim viewport, used only for debugging;
2. `front`, mounted on the wrist and recorded;
3. `side`, fixed inside the enclosure and recorded.

The recorded cameras run at `640x480`, 30 FPS. Both `front` and `side`
therefore use the dataset image shape `[480, 640, 3]` (height, width, RGB).

- `front`: `/World/Robot/link6/FrontCamera`
- `side`: `/World/Cameras/Side`

The side camera is located at `[0.37, -0.05, 0.60] m` and points toward
`[0.00, 0.25, 0.16] m`. It uses a `9.5 mm` wide-angle focal length.

The wrist camera uses a converted version of Seeed's UVC32 mount. The mount is
rotated onto the upper side of the gripper and pitched toward the fingertip
workspace. Its visuals have no mass or collision and therefore do not affect
robot dynamics.

Use `--single-viewport` to hide the two floating camera windows while keeping
both recording streams active.

## Domain randomization

At startup and after pressing `R`, the application randomizes:

- stationery XY position, yaw, and mass;
- dome and key-light intensity and color;
- a small wrist-camera pose and focal-length range.

The pencil cup, enclosure, and side camera remain fixed. Configuration lives in
`config/domain_randomization.json`.

Run a fixed scene with:

```bash
./run.sh --leader-port /dev/ttyUSB0 --no-dr
```

## Recording LeRobot demonstrations

1. Press `R` to reset and randomize the objects.
2. Press `S` to start recording.
3. Place both pens and the eraser into the pencil cup.
4. Press `S` again to finish and save the episode.
5. Press `C` instead if the demonstration should be discarded.

The default dataset is written to:

```text
~/rebot-arm-dli-isaacsim/datasets/rebot_stationery_front_side
```

Each episode includes:

- `action`: six leader arm joints plus the gripper;
- `observation.state`: six B601 follower joints plus the gripper;
- `observation.images.front`;
- `observation.images.side`;
- task text: `Put the two pens and the eraser into the pencil cup`;
- episode randomization and success metadata.

The success detector requires all objects to be inside the cup, both pens to
have an inserted orientation, the gripper to be released, and object speed to
remain below `0.25 m/s` for `0.35 s`.

Visualize episode 0 with:

```bash
cd ~/rebot_lerobot
source .venv/bin/activate

lerobot-dataset-viz \
  --repo-id local/rebot_stationery_front_side \
  --root ~/rebot-arm-dli-isaacsim/datasets/rebot_stationery_front_side \
  --episode-index 0
```

## Replay

Start the Isaac receiver without opening the leader or recording:

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --no-start-bridge --no-recording --no-dr
```

In another terminal, replay episode 0:

```bash
cd ~/rebot-arm-dli-isaacsim
~/rebot_lerobot/.venv/bin/python \
  scripts/replay_episode.py --episode 0
```

Add `--loop` for continuous playback.

## Offline validation

Run the mock-leader regression without hardware:

```bash
./run.sh --test
```

Also validate both cameras, H.264 encoding, and LeRobot Dataset v3 output:

```bash
./run.sh --test --test-recording
```

Temporary test data is written under `/tmp/rebot_stationery_dataset_*`.

`run_demo.sh` is retained as a compatibility shortcut to the same regression:

```bash
./run_demo.sh
```

## Configuration

- `config/teleop_config.json`: UDP, leader mapping, filters, and grasp assist
- `config/grasp_config.json`: scene paths, object geometry, and success criteria
- `config/domain_randomization.json`: object, lighting, camera, and recording
  randomization

The default leader-to-Isaac arm signs are:

```text
shoulder_pan  shoulder_lift  elbow_flex  wrist_flex  wrist_yaw  wrist_roll
     -1             -1            +1          +1          +1         -1
```

The calibrated leader gripper range of `0...45 degrees` maps to a simulated
range of `0...0.05 m`. Update `gripper_mapping.leader_open_deg` if the measured
leader range differs.

## Repository structure

```text
rebot-arm-dli-isaacsim/
├── assets/
│   ├── camera/UVC32_mount_portable.usd
│   ├── rebot_physx/00-arm-rs_asm-v3.usd
│   ├── stationery/stationery_models.usd
│   └── workspace/box.usdz
├── config/
│   ├── domain_randomization.json
│   ├── grasp_config.json
│   └── teleop_config.json
├── scenes/rebot_stationery_sorting.usda
├── scripts/
│   ├── build_scene.py
│   ├── convert_stationery_asset.py
│   ├── dataset_writer.py
│   ├── domain_randomization.py
│   ├── episode_recorder.py
│   ├── leader_bridge.py
│   ├── leader_teleop.py
│   ├── replay_episode.py
│   └── teleop_protocol.py
├── run.sh
└── run_demo.sh
```

## References

- [Seeed reBot Isaac Sim repository](https://github.com/Seeed-Projects/reBot-Isaacsim)
- [Seeed Isaac Sim joint receiver](https://github.com/Seeed-Projects/reBot-Isaacsim/blob/main/reBotArm_Isaacsim/isaacsim_joint_receiver.py)
- [NVIDIA SO-101 domain-randomization teleoperation tutorial](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/09-strategy1-dr-teleop.html)
- [NVIDIA Sim-to-Real SO-101 Workshop](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop)

## License and third-party assets

Project-authored source code is available under the MIT License. Bundled robot,
camera-mount, stationery, and workspace assets may use different licenses or
require separate redistribution permission. Review
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) before publishing a fork or
redistributing the asset bundle.
