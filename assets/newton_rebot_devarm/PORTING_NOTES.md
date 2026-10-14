# reBot DevArm —— Newton 官方适配资产（已移植）

本目录是 [newton-physics/newton-assets](https://github.com/newton-physics/newton-assets/tree/main/seeed_rebot_devarm)
中 `seeed_rebot_devarm` 包的完整拷贝，未经修改（MIT License，见 `LICENSE`）。

- 来源仓库：https://github.com/newton-physics/newton-assets
- 克隆时 main 分支提交：`a0547548eaa966c2f5478bee496c3cfba1fa98fc`（2026-10-13 移植）
- 上游原始数据：MuJoCo Menagerie 的 reBot DevArm MJCF（经 mujoco-usd-converter 转 USD；
  URDF 则来自 Seeed-Projects/reBot-Isaacsim，执行器限值已按实机核对）

## 目录内容

| 路径 | 说明 |
|---|---|
| `meshes/` | 34 个 STL，供 URDF 使用（`package://seeed_rebot_devarm/meshes/...`） |
| `urdf/seeed_rebot_devarm.urdf` | 供应商 URDF，含完整惯量；6 旋转关节 + 2 直线夹爪指 |
| `usd_structured/seeed_rebot_devarm.usda` | **结构化 USD 的入口文件**（本体在 Payload 后延迟加载） |
| `usd_structured/Payload/Physics.usda` | 全部物理数据（UsdPhysics + Newton + Mjc 三套 schema），调参只需改这一层 |

## 结构化 USD 的要点

- **面向 Newton 物理栈**：执行器是 8 个 `MjcActuator`（`mjc:gainPrm`/`biasPrm`/`forceRange`），
  夹爪 1:1 耦合表达为 `joint_left` 上的 `NewtonMimicAPI` + `MjcEqualityJointAPI`。
- **嵌套刚体层级**（MJCF 风格，子刚体相对父刚体），需要 USD ≥ 25.11 —— 本机 Isaac Sim 6.1
  自带 USD 25.11，已验证可组合。
- **自碰撞**：每个 link 是 8–12 块凸分解（共 92 个碰撞体），11 对互相干涉的部件用
  `physics:filteredPairs` 排除（含夹爪左右指、link2/link5，间隙仅 0.15/0.77 mm）。
- 所有引用均为相对路径，整个包自包含、可直接搬用。

## 已验证（2026-10-13，本机 Isaac Sim 6.1）

- 入口文件在工程内组合成功：115 个网格、10 个关节（joint1–6、joint_left/right、2 个固定
  关节）、4 个材质、9 个刚体带 `filteredPairs`；包围盒 561.3 × 200 × 320.8 mm，底面在 z = 0，
  单位米。
- 预览渲染：白/绿/黑 Seeed 官方配色正常出图。

## 用法与注意

- 一键验证（组合、关节、夹爪耦合、单位与包围盒逐项 PASS/FAIL，失败时退出码非零）：

  ```bash
  cd ~/rebot-arm-dli-isaacsim
  "${ISAACSIM_ROOT:-$HOME/isaacsim}/python.sh" scripts/verify_newton_asset.py
  # 需要预览图时加 --render（输出 /tmp/newton_rebot_preview/newton_rebot.png，需要 GPU）
  ```

- 单独查看：Isaac Sim 里直接打开 `usd_structured/seeed_rebot_devarm.usda`。
- **遥操场景未替换**：`scenes/rebot_stationery_sorting.usda` 用的仍是 PhysX 资产
  `assets/rebot/00-arm-rs_asm-v3.usda`，配 `build_scene.py` 里一整套 PhysX 调参（驱动器、
  SDF 接触、抓取辅助、腕部相机挂点都依赖 PhysX）。本资产是 Newton 栈的（PhysX 不认识
  `newton:*`/`mjc:*` schema 和夹爪耦合），直接放进 PhysX 场景会变成没有驱动器的被动模型。
  要在 Newton 后端上跑，需改用 Newton 启动方式（安装目录下 `isaac-sim.newton.sh` /
  `apps/isaacsim.exp.full.newton.kit`）并单独适配。
- USD 组合提示：入口文件带 `defaultPrim`，引用它时内容**直接组合到引用点自身**（不会多出
  一层同名子节点）——写引用代码时按引用点取 prim，不要去找 `.../seeed_rebot_devarm`。
