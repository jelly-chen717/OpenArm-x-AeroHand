当前仿真统一在 `openarm/`，原始备份为 `backups/OpenArm-original-20261115.zip`。

- [关节详细资料](关节详细资料.md)：双臂 14 轴统一正方向、位置范围、官方力矩/速度、软件时序及逐轴运动保护。2026-11-16 起采用新逻辑坐标；旧轨迹需转换，运行中的服务需重启加载。

- 双臂调试使用 `launchers/Start-OpenArm-Sim.cmd`。
- 钢琴演奏使用 `launchers/Start-OpenArm-Piano.cmd`。
- [合并核对与验证结果](docs/guides/仿真统一核对.md)。

OpenArm 统一调试入口：`launchers/Start-OpenArm-Debug.cmd`；[控制资料与调试指南](openarm/调试指南.md)。

# 项目工作区导航

Windows OpenArm 实机 CAN FD 接入使用 `apps/openarm_driver.py`，状态检查入口为 `launchers/Check-OpenArm.cmd`，配置和 API 见 [OpenArm驱动使用说明](docs/guides/OpenArm驱动使用说明.md)。

当前统一状态见 [项目总进度](docs/project/项目总进度.md)，交接文档见 [文档索引](openarm/交接文档索引.md)，目录整理见 [工作区整理记录](docs/project/工作区整理记录.md)。

- 负责人交接工程位于 `openarm/`，包含 V1 双臂、88 键、TXT 演奏与独立手指测试工具。环境状态详见各模块维护文档。
- 目录分级为 `launchers/` 启动入口、`apps/` 应用、`docs/` 文档与截图，仿真与 MIDI 按模块维护。

以下为原固定手位与 MIDI 工程说明。

# 机械手弹琴

固定琴位三指/四指演奏，以及七音 MIDI 规划模拟。

## 七音双琴位离线联动

新增 `midi文件/七音双琴位离线预览.cmd`：四指 A/B 两琴位计划预览、动作时间表导出和可停止的实时模拟。使用与实验室标定表见 `midi文件/七音双琴位使用与标定.md`。换位耗时为模拟假设，真实机械臂运行仍阻断。

## 日常使用

- 官方机械手调试：`launchers/Start-Aero-Hand.cmd`。
- 四指固定循环：`midi文件/四指do rei mi fa.cmd`，默认三轮。
- 七音 MIDI 模拟：`midi文件/midi_piano/Start-MIDI.cmd`。真实机械臂尚未接入，实物运行仍阻止。

实物演奏前断开官方上位机，保持标定的手掌与琴键位置。模拟不发声、不连接设备。指令发送不等于实物触键或回弹成功。

## 本机依赖

运行入口依赖本地 `.venv-aero`、`midi文件/midi_piano/.venv` 和 `aero-hand-open/sdk/src`。这些环境、官方源码及原始 ZIP 不纳入本仓库；本仓库不是可独立复制运行的发布包。

官方 SDK：https://github.com/Chestnut-Robotics/aero-hand-open.git
当前本地官方仓库提交：`7bb2f23485a7d440960c21046da642de132b7745`，本次检查工作区干净。Python 3.12；MIDI 环境依赖见 `midi文件/midi_piano/requirements.txt`。

标定角度由官方界面归一化滑块值换算，不能直接将滑块值当角度。配置仅适用于当前标定硬件和摆放。

工作记录见 `docs/project/工作日志.md`。

## 工作区分级

| 目录 | 内容 |
| --- | --- |
| `launchers/` | 官方手调试、固定手位弹奏与V2查看器启动脚本 |
| `apps/fixed_position/` | 固定手位程序、配置与测试 |
| `docs/project/` | 总进度、计划日志与整理记录 |
| `docs/guides/` | 操作说明 |
| `docs/learning/` | 学习文档 |
| `docs/screenshots/` | 界面截图 |
| `midi文件/` | MIDI工具、曲目及标定说明 |
| `openarm/` | 交接工程及其独立运行环境 |
| `resources/` | 仿真模型资源 |
| `aero-hand-open/`、`达妙电机/` | 官方源码与电机资料 |

在工作区根目录运行固定手位测试可使用 `.venv-aero/Scripts/python.exe -m unittest discover -s apps/fixed_position -v`。


