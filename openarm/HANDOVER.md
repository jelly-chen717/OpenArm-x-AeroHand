本机统一记录（2026-11-15）

当前工程为 `D:\桌面\open CyMon\弹钢琴\openarm`，使用自带 `.python311` 的 Python 3.11.9 和 `.venv`。统一操作与验证见 [本机复现指南](REPRODUCE_WINDOWS.md) 和 [合并核对记录](../docs/guides/仿真统一核对.md)。

下方为交接同学的原始记录，其中 `D:\py311-full`、RTX 4060、148/149 测试结果描述其交接电脑。当前机器的验证结果以合并核对记录为准。

---

# OpenArm V1 Windows 仿真项目 — 交接文档

交接日期：2026-11-15
项目路径：`D:\openarm\openarm`
配套文档：`REPRODUCE_WINDOWS.md`（从零复现步骤）、`README.md` / `CONTROL.md` / `WEB_CONTROL.md` / `SIMULATION.md` / `VALIDATION.md`（原作者遗留，内容仍然有效）

---

## 1. 这是什么项目

OpenArm V1 双臂机械臂的 **MuJoCo 离线仿真控制台**，Windows 本机运行，不接任何硬件：

- 双臂 7 轴 ×2 = 14 轴 + 固定夹爪，官方 V1 模型（`vendor/openarm_mujoco/v1/scene.xml`，nq=18）
- 官方 `openarm-control==0.4.0` IK/FK + 五次多项式关节轨迹（零端点速度/加速度）
- 架构：**仿真服务进程**（MuJoCo 窗口 + `http://127.0.0.1:8769` JSON API）+ 任意数量的**控制端**
  （Tkinter 面板 / 网页上位机 8088 / 曲谱页 8090 / 调试页 8089 / Python 脚本）
- 彩蛋场景：双臂换装 Aero 灵巧手 + 88 键悬空钢琴，指尖真实接触驱动琴键发声

## 2. 这台机器的环境现状（与原始包的差异）

原始压缩包来自作者机器（`C:\Users\maker\anaconda3\envs\brainlink`），venv 是目录拷贝，
换机后解释器路径全断。本机已修复：

| 项 | 现状 |
|---|---|
| Python | `D:\py311-full`（conda，3.11.18，含 tkinter；创建命令见 REPRODUCE_WINDOWS.md 3.2） |
| venv | `D:\openarm\openarm\.venv`，`pyvenv.cfg` 已改为 `home = D:\py311-full`（原文件备份 `pyvenv.cfg.bak`） |
| 源码挂载 | `__editable__.openarm_v1_controller-0.1.0.pth` 已改为 `D:/openarm/openarm/src` 与 `/skills` |
| 显卡 | `graphics --configure` 已写入偏好，MuJoCo 实测走 **RTX 4060 Laptop GPU**（原来走 Intel 核显） |
| pip | `pip 24.0` 可用，安装新包用 `python -m pip`（venv 里 uv 装的 exe 启动器路径失效，勿用） |

**注意 `aero-open-sdk==0.1.0.dev1` 是 dev 版，PyPI 装不到，只存在于这个 venv 里**——
所以 `.venv` 目录要跟着包走，别删。

## 3. 日常操作（三条入口）

| 想做什么 | 双击 / 命令 | 页面 |
|---|---|---|
| 控制双臂（面板） | `start_mujoco.cmd` → `start_controller.cmd` | Tkinter |
| 控制双臂（网页 3D） | `start_mujoco.cmd` → `start_web_controller.cmd` | http://127.0.0.1:8088 |
| **弹钢琴** | `弹钢琴-一键启动.cmd`（本机新增） | http://127.0.0.1:8090 |
| 手动按琴键/单指调试 | `skills\paino_skill\start_piano.cmd` + `start_piano_controller.cmd` | http://127.0.0.1:8089 |
| CSV 回放验证 | `verify_simulation.cmd` | 无页面 |

无窗口调试：`python -m openarm_app.sim_server --headless --port 8769`，然后 `GET /state`。
端口被占：仿真端/控制端都用 `--port 8770`，两端一致。

## 4. 必须懂的机制（交接核心）

1. **理想位置跟随**：关节直接"吸"到目标位置，无重力补偿、无动力学积分——松手不会掉。
2. **同一时刻只接受一个动作**：运动中/暂停时新目标一律 HTTP 400。双臂同时动必须 `arm="both"` 一次提交。
3. **默认速度/加速度上限**：仿真循环 60 Hz；8088 网页速度倍率 100% = 0.2 rad/s、0.4 rad/s²；
   8090 曲谱页默认 **2.5 rad/s、50 rad/s²**；8089 调试页 0.2/0.4。
4. **整曲预检公式**（`skills/paino_skill/score_runner.py:195`）：
   `v_peak = 1.875·δ/T`，`a_peak = 5.7735·δ/T²`，δ 为该次换键最大关节变化量。
   任何一条 move 超限，**整曲拒绝，不自动放慢**（绝对微秒时间轴，放慢会错位）。
5. **触键发声**：指尖碰撞垫压键 >0.6 mm 发 note-on，<0.2 mm 发 note-off；事件字段是
   `type: "on"/"off"`（不是 `kind`），带 `midi`。音源 `samples/tone (1..88).wav` 按 A0–C8。
6. **无避碰、无力控、无真机**：两臂各自 IK，可能相撞；CSV 不是上机许可。

## 5. 已知问题与踩过的坑

- **《起风了》预检失败**（用户遇到）：第 464 行 `right_arm_move(26, 0.316666)`，
  跨 2 白键 δ=0.651 rad → 需要 3.8556 rad/s > 2.5 上限（加速度 37.49 没超）。
  解法：速度上限调到 **4.0**；或改该行时长为 0.488370 并把后面 `sleep` 改 0.178374（整段顺延 0.17 s）。
  曲库 55 首里只有《小苹果》《月亮代表我的心》被作者验证过预检通过。
- 路径含中文会让 MuJoCo 原生接口报错（项目已用 `MjModel.from_xml_string(assets=...)` 规避，
  但项目目录本身必须是纯英文路径）。
- **项目目录不能随意搬**：`.venv` 和 `.pth` 都是绝对路径，搬完要按 REPRODUCE_WINDOWS.md 3.4 重指。
- Python 必须 3.11（`requires-python >=3.11,<3.12`），且 base 必须带 tkinter（控制面板依赖）。
- pytest 148/149：唯一失败 `test_both_arm_fields_fit_window`（tkinter 面板宽度 986>940），
  是 DPI/字体缩放断言，非功能问题。

## 6. 本机已验证清单

- `cli inspect` / `cli demo`：FK 误差 ~1e-8，CSV 导出正常
- `sim_server`：有窗口 + headless 均正常，`/state`、`/piano/state` 正常
- 关节运动端到端：`IDLE → EXECUTING(1.2s) → COMPLETED`，终值精确命中
- 钢琴自动演奏：《小苹果》预检 passed（259 音符/431 动作/69.9 s），开演 11 s 内
  20 次 note-on / 18 次 note-off（MIDI 45/72/69/41），真实接触驱动
- 曲库 55 首可被扫描

## 7. 打包说明

- 本包制作时间：2026-11-15。为控制体积，已剔除：`.uv-cache`、`.test-temp*`、`.build-temp`、
  `.pytest_cache`、`__pycache__`、`skills/paino_skill/logs`（约 900 MB 均为测试临时产物）。
- `vendor/`（309M，官方模型+许可证）、`skills/paino_skill/samples`（129M 音源）、
  `skills/paino_skill/piano_standalone.html`（103M，内嵌音频的独立播放页）**必须保留**。
- `logs/` 保留了 GPU 偏好备份等运行记录。

## 8. 未完成事项（原作者遗留）

- 阶段 3 真机验收未完成：无 COM/CAN 电机映射、PosVel 下发、安全状态机（README 末节）
- 双臂无碰撞检查/避碰规划；架子鼓（`skills/drums`）与绘画（`skills/drawing`）只是空模板
- 无逐帧历史轨迹记录；`start_score_right_real.cmd` 是空文件（真机右臂入口未实现）
- `tools/check_web_ui.py` 测试需要 Playwright + Node + Edge，本机未装

## 9. 接手后第一步建议

1. 双击 `start_mujoco.cmd` → `start_controller.cmd`，确认 3D 窗口和控制面板都活着
2. 双击 `弹钢琴-一键启动.cmd`，选《小苹果》跑一遍，听声音
3. 读 `REPRODUCE_WINDOWS.md`（换机器/重装时照抄即可），再按需读 CONTROL.md / WEB_CONTROL.md
4. 要编程控制，从 `CONTROL.md` 的 `SimClient` 代码片段开始，只有 5 个动作：
   `move_joints / move_pose / preview_pose / home / pause / resume / stop`
