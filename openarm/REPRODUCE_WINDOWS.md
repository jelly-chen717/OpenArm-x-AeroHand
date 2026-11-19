# Windows 本机运行与恢复

统一日期为 2026-11-15。工程目录为 `D:\桌面\open CyMon\弹钢琴\openarm`。

## 日常入口

在工作区 `launchers` 中双击以下脚本。

| 操作 | 入口 | 页面 |
|---|---|---|
| 双臂调试 | `Start-OpenArm-Sim.cmd` | `http://127.0.0.1:8088` |
| 钢琴演奏 | `Start-OpenArm-Piano.cmd` | `http://127.0.0.1:8090` |
| 资料、自检、硬件状态查询 | `Start-OpenArm-Debug.cmd` | 菜单 |

前两项自动启动 MuJoCo 窗口和对应网页服务。关闭 MuJoCo 窗口，或在启动窗口按 Ctrl+C，结束这次启动的两个进程。两个场景共用 8769 端口，切换时先退出当前场景。端口已占用时，入口显示错误和日志位置。

钢琴页选歌后点击“开始演奏”。首次点击允许浏览器播放音频，保持页面开启。单指调试使用 `skills/paino_skill/start_piano_controller.cmd`，在钢琴仿真运行时打开 8089。

独立离线网页统一保存在 `skills/paino_skill/piano_standalone.html`，可直接打开。维护源文件为同目录的 `web/standalone*` 和 `build_standalone.py`。

## 本机环境

- 基础解释器位于 `.python311/python.exe`，版本为 Python 3.11.9，包含 Tk。
- 运行环境位于 `.venv`，MuJoCo 3.17.0、NumPy 2.4.6、openarm-control 0.4.0、openarm-mujoco 2.0.1。
- V1 场景来自 `vendor/openarm_mujoco/v1/scene.xml`，模型通过内存资源加载，已在当前中文路径下验证。
- `.venv/pyvenv.cfg` 与 editable 包映射绑定当前工程路径。运行命令统一采用 `.venv/Scripts/python.exe -m ...`。
- Aero SDK dev 版已安装在本地环境。恢复环境时可使用工作区 `aero-hand-open/sdk` 的源码和备份包中的安装副本。

## 命令行验证

在本工程目录执行。

```powershell
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m openarm_app.cli inspect
.\.venv\Scripts\python.exe -m openarm_app.cli --arm left inspect
.\.venv\Scripts\python.exe -m pytest -q --basetemp=.test-temp
.\.venv\Scripts\python.exe tools\start_simulation.py --scene default --headless --no-browser --check
```

本机 HTTP 测试需要允许访问回环地址。全套自动测试和整曲接触事件的结果见 [统一核对记录](../docs/guides/仿真统一核对.md)。

## 恢复原始备份

唯一的原始备份位于工作区 `backups/OpenArm-original-20261115.zip`，保存合并前的工程、Python、运行依赖、模型、曲库、采样和原有网页。ZIP 内顶层目录为 `openarm`。恢复到当前工作区路径后使用其 `.venv/Scripts/python.exe`。

如恢复到其他路径，需要修改 `.venv/pyvenv.cfg` 中的基础解释器路径，以及 `.venv/Lib/site-packages` 中本工程 editable 包的路径映射，再运行上述检查。

交接同学电脑的 Python 3.11.18、`D:/py311-full` 和 RTX 4060 记录保存在 [HANDOVER](HANDOVER.md)。本机环境采用上方列出的版本和路径。
