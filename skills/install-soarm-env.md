# SO-ARM101 Environment Setup Skill

Copy this entire file to your AI assistant (Claude, ChatGPT, etc.) and ask it to execute the setup.

## What this installs

- NVIDIA Isaac Sim 6.1 (simulation environment)
- LeRobot framework (robot learning)
- The SO-ARM101 teleoperation project (white SO-ARM101 in Isaac Sim)

## Prerequisites

- Ubuntu 22.04+ with NVIDIA GPU (≥ 8 GB VRAM; 12 GB+ recommended)
- Python 3.12, `git`, `git-lfs` installed
- ~50 GB free disk space

## Setup commands

Execute these commands in order. Each block should succeed before moving to the next.

### Step 1: Install Isaac Sim 6.1

```bash
# Download and extract Isaac Sim to ~/isaacsim
# (adjust the path if your download is elsewhere)
cd ~/下载
unzip isaac-sim-standalone-6.1.0-linux-x86_64.zip -d ~/isaacsim
cd ~/isaacsim
./post_install.sh

# Verify
~/isaacsim/python.sh -c "print('Isaac Sim OK')"
```

If Isaac Sim is already installed at a different path, set `ISAACSIM_ROOT`:
```bash
export ISAACSIM_ROOT=/path/to/isaacsim
```

### Step 2: Set up LeRobot Python environment

```bash
mkdir -p ~/rebot_lerobot
cd ~/rebot_lerobot

# Clone repositories
git clone https://github.com/Seeed-Projects/lerobot.git
git clone https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102.git
git clone https://github.com/Seeed-Projects/lerobot-robot-seeed-b601.git

# Create virtual environment (Python 3.12)
uv venv --python 3.12 .venv
source .venv/bin/activate

# Install packages
uv pip install --upgrade pip
uv pip install -e "./lerobot[dataset]"
uv pip install -e ./lerobot-teleoperator-rebot-arm-102
uv pip install -e ./lerobot-robot-seeed-b601

# SO-101 leader/follower SDK (Feetech STS3215 servos)
uv pip install feetech-servo-sdk

# Verify
python -c "import lerobot; print('LeRobot', lerobot.__version__)"
python -c "import scservo_sdk; print('Feetech SDK OK')"
```

### Step 3: Clone the SO-ARM101 project

```bash
cd ~
git clone https://github.com/yuyoujiang/rebot-arm-dli-isaacsim.git
cd rebot-arm-dli-isaacsim
git lfs install
git lfs pull
```

### Step 4: Verify the SO-ARM101 scene (headless, no hardware needed)

```bash
cd ~/rebot-arm-dli-isaacsim
export ISAACSIM_ROOT=~/isaacsim
./run_so101.sh --test
```

Expected output: all 5 `[test-PASS]` lines.

### Step 5 (Optional): Calibrate the SO-101 leader arm

Only needed if you have a physical leader arm connected:

```bash
cd ~/rebot_lerobot
source .venv/bin/activate

# Find the serial port
ls /dev/ttyACM* /dev/ttyUSB*

# Calibrate (follow the on-screen prompts)
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so101_leader
```

### Step 6 (Optional): Start teleoperation

```bash
cd ~/rebot-arm-dli-isaacsim
export ISAACSIM_ROOT=~/isaacsim

# Sim only (no real arm)
./run_so101.sh --leader-port /dev/ttyACM0 --no-follower --single-viewport

# Real arm + sim linked
./run_so101.sh --leader-port /dev/ttyACM0 --follower-port /dev/ttyUSB1 --single-viewport
```

## Keyboard teleoperation (no leader arm needed)

Start the app, click the 3D viewport, press `Y` to toggle keyboard mode.

| Key | Function |
|-----|----------|
| W/S/A/D | Move gripper forward/back/left/right |
| ↑/↓ | Raise/lower gripper |
| Z/C | Tilt gripper down/up |
| Q/E | Rotate wrist |
| G | Toggle jaw open/close |
| B | Toggle step size (5/15mm) |
| Y | Toggle leader/keyboard mode |
| T | Start/stop recording |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| GPU freezes | Use `--single-viewport`; ensure ≥8GB VRAM; close browser |
| `No module named 'scservo_sdk'` | `uv pip install feetech-servo-sdk` in the LeRobot venv |
| `Address already in use` (UDP 5015) | Kill the old process: `pkill -f so101_teleop` |
| Leader not calibrated | Run Step 5 above |
| Scene looks outdated | `./run_so101.sh --rebuild-scene --test` |
