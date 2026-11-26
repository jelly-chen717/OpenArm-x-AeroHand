# 项目工作区导航

## 双击启动

| 根目录入口 | 启动后 |
| --- | --- |
| [01-实机控制台.cmd](01-实机控制台.cmd) | 打开 [8092 控制台](http://127.0.0.1:8092/)，自动连接左臂 `0x51`、右臂 `0x52` 并读取反馈。再次双击会打开现有实机会话。使能由操作员在网页上选择。 |
| [02-钢琴弹奏+MuJoCo.cmd](02-钢琴弹奏+MuJoCo.cmd) | 启动 MuJoCo 钢琴场景，自动打开 [8090 演奏网页](http://127.0.0.1:8090/)，可导入 TXT。演奏服务和配套仿真已运行时会打开现有演奏页。 |
| [03-Aero Hand 调试.cmd](<03-Aero Hand 调试.cmd>) | 打开 Aero Hand 官方上位机。在列表中选手部实际 COM 口，再点 Connect。 |

旧调试菜单和专用维护脚本位于 `launchers/`。[停止旧控制台](launchers/停止旧控制台-8092.cmd)根目录的 `piano_standalone.html` 是 8090 演奏网页加载的前端文件。

Windows OpenArm 实机 CAN FD 接入使用 `apps/openarm_driver.py`，状态检查入口为 `launchers/Check-OpenArm.cmd`，配置和 API 见 [OpenArm驱动使用说明](docs/guides/OpenArm驱动使用说明.md)。

当前统一状态见 [项目总进度](docs/project/项目总进度.md)，交接文档见 [文档索引](openarm/交接文档索引.md)，目录整理见 [工作区整理记录](docs/project/工作区整理记录.md)。

- 实琴为 **C2–C7（61 键）**，白键中心距 22.5 mm。左食指原位 **A3（实琴白键 13）**，基座系 XYZ 为 (250.19, 56.1, −170.00) mm。右食指原位 **C4（实琴白键 15）**，XYZ 为 (250.25, −55.0, −170.00) mm。左食指从 A3 向中央一格到 B3。[实琴编号与标定表](monitor-ui/README.md#实琴琴键与食指原位) · [末端位姿版本索引](docs/archive/piano-calibration/README.md)
- 8092 控制台用同一张表定位、估算八指音名、预览并试弹曲库。整曲从 A3/C4 原位移至首腕位，等待 2 秒后按该曲倍率播放。[控制台操作](monitor-ui/README.md#钢琴定位与试弹) · [实琴点位与验收记录](docs/project/仿真与实机联通_两日实施计划.md#piano-a3-c4-xz)
- 8090 选歌页搜索「实琴分手」可打开《槿》的主旋律、纯和弦和副歌纯旋律。这里使用 A0–C8 的 88 键 MuJoCo 场景和仿真触键坐标。[仿真操作](openarm/skills/paino_skill/README.md)
- 负责人交接工程位于 `openarm/`，包含 V1 双臂、A0–C8 的 88 键 MuJoCo 仿真、TXT 演奏与独立手指测试工具。
- 仿真位于 `openarm/`，原始备份为 `backups/OpenArm-original-20261115.zip`。双臂 14 轴从 2026-11-16 起采用统一逻辑坐标，旧轨迹需转换，运行中的服务需重启加载。[关节详细资料](关节详细资料.md) · [仿真合并验证](docs/guides/仿真统一核对.md) · [控制资料与调试指南](openarm/调试指南.md)

以下为原固定手位与 MIDI 工程说明。

# 机械手弹琴

固定琴位三指/四指演奏，以及七音 MIDI 规划模拟。

## 七音双琴位离线联动

新增 `midi文件/七音双琴位离线预览.cmd`：四指 A/B 两琴位计划预览、动作时间表导出和可停止的实时模拟。使用与实验室标定表见 `midi文件/七音双琴位使用与标定.md`。换位耗时为模拟假设，真实机械臂运行仍阻断。

## 固定琴位与 MIDI 实验

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
| `launchers/` | 旧调试菜单、状态检查和各模块启动脚本 |
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


