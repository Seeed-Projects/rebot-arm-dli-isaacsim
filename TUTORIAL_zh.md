# Isaac Sim 6.1 数据采集教程（reBot Arm B601 / LeRobot Dataset v3）

> 本教程基于 **Isaac Sim 6.1** 编写与验证（原项目基于 Isaac Sim 4.5，仓库代码已完成 6.1 适配）。
> 没有实体主臂也可以完成第 2~4、8 节的环境与资产验证。

## 目录

1. [环境要求](#1-环境要求)
2. [安装 Isaac Sim 6.1](#2-安装-isaac-sim-61)
3. [获取代码（需要 Git LFS）](#3-获取代码需要-git-lfs)
4. [配置 LeRobot 环境](#4-配置-lerobot-环境)
5. [连接主臂与校准](#5-连接主臂与校准)
6. [启动遥操作与数据采集](#6-启动遥操作与数据采集)
7. [数据集检查、可视化与回放](#7-数据集检查可视化与回放)
8. [无硬件环境验证](#8-无硬件环境验证)
9. [与 Isaac Sim 4.5 的差异说明](#9-与-isaac-sim-45-的差异说明)
10. [已知问题](#10-已知问题)
11. [常见问题排查](#11-常见问题排查)

---

## 1. 环境要求

> ⚠️ **显卡要求（重要）**：本工程必须使用**显存 ≥ 12GB 的 NVIDIA RTX 显卡**（如 RTX 3080 12G / 4070 Ti 12G 及以上），并安装较新的驱动。
> **显存不足会直接导致电脑死机**：本工程同时运行机械臂、三路相机视口和物理仿真，8GB 显存（如 8GB 笔记本 GPU）在默认渲染器下超出余量，运行几分钟到几十分钟后可能拖死桌面合成器——画面静止、键鼠无响应、只能强制重启（应用本身不一定崩溃，Isaac 日志里通常能看到完整的运行记录）。这不是可以忽略的警告：本工程开发过程中在一台 8GB 笔记本 GPU 上**确实复现过两次整机死机**（包括加了各项防护、使用 `--single-viewport` 的情况），因此**换用显存 ≥ 12GB 的显卡是唯一可靠的解决办法**。
> 如果你只有 8GB 显卡，可以先试第 10 节的两项内置防护与进一步缓解措施，但要清楚仍有死机风险。

- Ubuntu 22.04（Isaac Sim 6.1 同时支持 24.04）
- NVIDIA RTX 显卡，**显存 ≥ 12GB**（8GB 会死机，见上方警告与第 10 节）
- 较新的 NVIDIA 驱动
- Isaac Sim 6.1 standalone 包
- 实际采集时需要 reBot Arm 102 Leader 主臂（USB-UART 连接）

推荐目录结构：

```text
~
├── isaacsim/                 # Isaac Sim 6.1 安装目录
└── rebot_lerobot/
    └── .venv/bin/python      # LeRobot 虚拟环境
```

## 2. 安装 Isaac Sim 6.1

从 NVIDIA 官网下载 `isaac-sim-standalone-6.1.0-linux-x86_64.zip`（约 12.4GB），解压到 `~/isaacsim` 并执行安装脚本：

```bash
unzip isaac-sim-standalone-6.1.0-linux-x86_64.zip -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
```

运行兼容性检查确认环境满足要求：

```bash
./isaac-sim.compatibility_check.sh     # 应输出 System checking result: PASSED
```

几个说明：

- 首次启动（`./isaac-sim.sh`）需要几分钟做着色器编译，属于正常现象；
- 如果 Isaac Sim 安装在其它目录，运行时用 `ISAACSIM_ROOT` 指向它即可（见第 6 节）；
- 启动日志中的 IOMMU、DLSS-RR、PCIe link width 警告均不影响使用。

## 3. 获取代码（需要 Git LFS）

> ⚠️ 本项目的机器人、相机支架、工作台、小瓶和瓶架 **USD 资产使用 Git LFS 存储**。
> 如果本机没有安装 `git-lfs`，克隆下来只会得到几百字节的指针文件，Isaac Sim 将无法打开场景。

```bash
sudo apt update
sudo apt install -y git-lfs

cd ~
git clone https://github.com/Seeed-Projects/rebot-arm-dli-isaacsim.git
cd rebot-arm-dli-isaacsim
git lfs install
git lfs pull
```

验证资产完整：

```bash
du -sh assets/
# 应约为 157M

head -c 8 assets/rebot_physx/configuration/00-arm-rs_asm-v3_base.usd
# 应显示 PXR-USDC；如果显示 "version https://git-lfs..." 说明资产没有拉全，请重新执行 git lfs pull
```

## 4. 配置 LeRobot 环境

```bash
mkdir ~/rebot_lerobot
cd ~/rebot_lerobot
sudo apt update
sudo apt install -y ffmpeg

# 下载仓库
git clone https://github.com/Seeed-Projects/lerobot.git
git clone https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102.git
git clone https://github.com/Seeed-Projects/lerobot-robot-seeed-b601.git

# 创建虚拟环境（Python 3.12）
uv venv --python 3.12 .venv
source .venv/bin/activate

# 安装 lerobot（注意 [dataset,viz] 扩展：数据集写入与可视化依赖）
uv pip install -e "./lerobot[dataset,viz]"

# 安装本地依赖包
uv pip install -e ./lerobot-teleoperator-rebot-arm-102
uv pip install -e ./lerobot-robot-seeed-b601
uv pip install motorbridge
```

> `[dataset,viz]` 会安装 `datasets`、`Pillow`、`av`、`rerun` 等依赖。
> 缺少它们时，录制的 episode 会转换失败（终端出现 `[dataset-error]`）。

## 5. 连接主臂与校准

主臂通过 USB-UART 连接电脑。串口权限有两种方式：

```bash
sudo chmod 666 /dev/ttyUSB0        # 临时（重启后失效）
sudo usermod -aG dialout "$USER"   # 永久（需要重新登录）
```

使用 LeRobot 校准主臂（默认校准 ID 为 `rebot_arm_102_leader`）：

```bash
cd ~/rebot_lerobot
source .venv/bin/activate

lerobot-calibrate \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader
```

## 6. 启动遥操作与数据采集

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --leader-port /dev/ttyUSB0
```

- 如果电脑上只有一个 `/dev/ttyUSB*` 设备，可以省略 `--leader-port`，程序会自动检测；
- 如果 Isaac Sim 不在 `~/isaacsim`：

```bash
ISAACSIM_ROOT=/path/to/isaacsim ./run.sh --leader-port /dev/ttyUSB0
```

启动完成后，等待 Isaac Sim 窗口出现（约 1~2 分钟），确认视口中能看到机械臂、围栏、三个小瓶和瓶架。

### 采集流程

> 键盘操作要求 Isaac Sim 视口获得焦点（用鼠标点一下视口）。

| 按键 | 功能 |
|---|---|
| `R` | 重置三个小瓶并重新随机化 |
| `S` | 开始录制；录制中再按一次结束并保存当前 episode |
| `C` | 丢弃当前 episode（不写入数据集） |

标准流程：

1. 按 `R` 重置场景：三个小瓶会从空中落到工作面上；
2. 检查三路相机画面与小瓶的初始位置（见下方「相机画面预览」）；
3. 按 `S` 开始录制，终端出现 `[record] Episode started at 30 FPS -> ...`；
4. 操作主臂，依次把三个小瓶放入瓶架（四个插槽任选，任意角度站放均可）；
5. 松开夹爪，等小瓶在瓶架中稳定；终端出现 `[SUCCESS]` 后按 `S` 结束并保存；
6. 等待后台编码完成，终端出现 `[dataset] Episode saved; total completed: N` 后可继续下一轮。

录制过程中如果小瓶掉落、抓取失败或画面被遮挡，按 `C` 丢弃当前 episode，再按 `R` 开始下一轮。

### 相机画面预览

`R` 重置后的三路相机画面如下（左：腕部 `front`；中：侧视 `side`；右：顶部 `external`）。数据集实际记录的是 `front` 与 `external` 两路视频流，`side` 仅用于观察：

| ![front](media/camera_previews/preview_front_seed601.png) | ![side](media/camera_previews/preview_side_seed601.png) | ![external](media/camera_previews/preview_external_seed601.png) |
|---|---|---|
| 腕部相机 `front`（随手臂移动，录制） | 侧视相机 `side`（围栏内固定，仅观察） | 顶部相机 `external`（俯视，录制） |

> 腕部相机画面下方是夹爪与相机支架硬件，属于正常现象；可用 `./run.sh --single-viewport` 只保留主视口、隐藏三个浮动相机窗口（录制不受影响）。
> 图片可复现：`"$ISAACSIM_ROOT/python.sh" scripts/capture_camera_previews.py`（headless，截图存到 `media/camera_previews/`）。

默认数据输出目录：

```bash
~/rebot-arm-dli-isaacsim/datasets/rebot_stationery_front_side
```

系统同时记录 `action`、`observation.state`、`observation.images.front`（腕部相机）、`observation.images.external`（顶部相机）与任务成功状态，频率 30 FPS，相机分辨率 640 × 480。

## 7. 数据集检查、可视化与回放

### 可视化数据集

```bash
cd ~/rebot_lerobot
source .venv/bin/activate

lerobot-dataset-viz \
  --repo-id local/rebot_stationery_front_side \
  --root ~/rebot-arm-dli-isaacsim/datasets/rebot_stationery_front_side \
  --mode local \
  --episode-index 0
```

依次检查不同 episode，确认动作/状态/两路视频时间同步，画面没有黑帧、冻结帧或严重遮挡。

### 在 Isaac Sim 中回放动作

先启动一个"只接收"的 Isaac Sim（不连接主臂、不录制、不做随机化）：

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --no-start-bridge --no-recording --no-dr
```

再开一个终端回放第 0 个 episode：

```bash
cd ~/rebot-arm-dli-isaacsim
../rebot_lerobot/.venv/bin/python scripts/replay_episode.py --episode 0
```

加 `--loop` 可循环回放。回放脚本发送的是数据集中的 `action` 轨迹，用于检查机械臂运动轨迹；相机画面请用数据集可视化工具查看。

## 8. 无硬件环境验证

没有主臂时，可以用以下方式验证 Isaac Sim、资产与整个采集链路：

**方式一：自动回归测试（推荐）**

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --test                     # 物理与抓取回归，约 3 分钟
./run.sh --test --test-recording    # 附加相机录制与数据集写入验证
```

看到 `[SUCCESS]` 即代表：全部 USD 资产正常加载、场景构建正常、物理/夹爪/抓取正常、数据集写入正常。该模式使用内置的 mock 主臂，不需要任何硬件。

**方式二：GUI 目视检查**

```bash
cd ~/rebot-arm-dli-isaacsim
./run.sh --no-start-bridge --no-recording --no-dr
```

`--no-start-bridge` 表示不连接主臂。启动后可查看 Side / Front / External 三个相机视图与场景树；在视口中按 `R` 可观察小瓶掉落，验证物理正常。

**方式三：Newton 资产包验证**

仓库内 `assets/newton_rebot_devarm/` 是 Newton 官方适配的 reBot DevArm 资产包（URDF + STL + 面向 Newton/MuJoCo 物理栈的结构化 USD，MIT 许可；当前遥操场景不使用它，详见该目录下的 `PORTING_NOTES.md`）。可独立验证这个包是否完整可加载：

```bash
cd ~/rebot-arm-dli-isaacsim
"${ISAACSIM_ROOT:-$HOME/isaacsim}/python.sh" scripts/verify_newton_asset.py
```

脚本逐项输出 PASS/FAIL（入口组合、115 个网格、8 个关节、夹爪 1:1 耦合、自碰撞过滤对、材质、单位与包围盒），全部通过时退出码为 0、打印 `ALL CHECKS PASSED`。加 `--render` 可额外输出一张预览图到 `/tmp/newton_rebot_preview/newton_rebot.png`（需要 GPU，约 1 分钟）。

## 9. 与 Isaac Sim 4.5 的差异说明

本仓库代码从 4.5 迁移到 6.1 时修改了以下内容（详见 git 提交记录）：

- **PhysX 110 的 kinematic 语义**：6.1 会立即应用对已初始化刚体的 kinematic 标志切换（4.5 会静默忽略），因此 mock 回归测试补充了显式冻结与姿态搬运逻辑；
- **测试时序**：测试模式的模拟数据改为按仿真时间消费，渲染较慢时轨迹不会被压缩；
- **LeRobot API**：视频编码参数向新版 lerobot 主分支对齐——直接传 `vcodec="h264"`（上游已移除旧的 `RGBEncoderConfig` / `rgb_encoder=` 写法，`dataset_writer.py` 已同步适配），`replay_episode.py` 兼容新版数据集加载；
- **运行环境**：Isaac Sim 6.1 需要 `lerobot[dataset,viz]` 完整依赖（见第 4 节）。

## 10. 已知问题

**画面噪点（"雪花"）**

Isaac Sim 6.1 默认使用 `RTX - Real-Time 2.0` 渲染器，其降噪依赖 DLSS Ray Reconstruction（目前仅 RTX 40 系及以上显卡支持）。
在 RTX 30 系等不支持该特性的显卡上，**GUI 视口与录制的相机图像会出现明显的逐像素噪点**。功能不受影响（采集、保存、可视化、回放均正常），但图像的训练数据质量会受影响。该问题正在持续排查中。

**注意：不要使用 `RTX - Interactive (Path Tracing)` 渲染模式。** 该模式画面干净但负载极高，在显存较小的显卡（如 RTX 3060 12GB）上可能导致系统死机。请保持默认渲染模式。

**系统卡死（桌面无响应）**

> **建议直接使用显存 ≥ 12GB 的显卡（见第 1 节）。** 以下防护只是降低风险，不能保证 8GB 显卡不发生死机（实测已发生过两次，含 `--single-viewport` 模式）。

在显存较小的显卡（如 8GB 笔记本 GPU）上，重负载渲染可能拖死桌面合成器，表现为"电脑卡死"，而应用本身通常没有崩溃（Isaac 日志里能找到完整的 26 分钟运行记录和正常退出）。本工程默认做了两项防护：

- **主循环限帧到 60 Hz**（`scripts/leader_teleop.py`：把 `app.runLoops.main.rateLimitFrequency` 从默认 120 调到 60，与主臂数据 60 Hz 对等），给 GPU 留出余量；
- 关闭 **`RTX - Interactive (Path Tracing)`** 的持久开关（`Preferences → Rendering → RTX Renderers` 里不要勾选；勾选状态会跨启动保留）。

进一步缓解：用 `./run.sh --single-viewport` 少开相机视口、关闭其它占显卡的程序（浏览器/微信等）、或把屏幕刷新率调低到 60 Hz。若仍卡顿，可在 Preferences 中改勾 `Enable RTX - Real-Time (Deprecated)` 并取消 `Real-Time 2.0`（旧实时渲染器用 OptiX 降噪，不依赖 DLSS）。

## 11. 常见问题排查

**`[error] Isaac Sim Python was not found`**

Isaac Sim 不在默认的 `~/isaacsim`，用 `ISAACSIM_ROOT=/path/to/isaacsim ./run.sh` 指定。

**`The current user cannot read or write the serial port`**

串口权限不足，执行 `sudo chmod 666 /dev/ttyUSB0` 或把用户加入 `dialout` 组后重新登录。

**Isaac Sim 打不开场景 / 报资产缺失**

Git LFS 资产没有拉全。执行 `git lfs pull`，并用第 3 节的 `du -sh assets/` 与文件头检查确认。

**录制结束但出现 `[dataset-error]`**

LeRobot 环境缺少数据集依赖，执行：

```bash
cd ~/rebot_lerobot && source .venv/bin/activate
uv pip install -e "./lerobot[dataset,viz]"
```

**终端出现 `[watchdog] Leader data timed out`**

主臂数据中断（USB 松动或 bridge 进程退出），sim 会保持最后的目标位姿。检查主臂连接后重新操作即可。

**启动日志中的警告**（IOMMU / DLSS-RR / PCIe link width / 无法访问 NVIDIA 资产服务器）：均不影响本项目使用，可忽略。
