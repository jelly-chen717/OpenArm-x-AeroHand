# 机械手弹琴

固定琴位三指/四指演奏，以及七音 MIDI 规划模拟。

## 日常使用

- 官方机械手调试：`Start-Aero-Hand.cmd`。
- 三指固定循环：`Start-Piano.cmd`，配置 `piano_config.json`。
- 四指 MIDI 界面：`midi文件/四指MIDI上位机.cmd`。选择 C4/D4/E4/F4 单旋律 MIDI，用滑轨调节速度，支持模拟、实物演奏和停止。每次导入默认 50%，播放时锁定设置。
- 四指固定循环：`midi文件/四指do rei mi fa.cmd`，默认三轮。
- 七音 MIDI 模拟：`midi文件/midi_piano/Start-MIDI.cmd`。真实机械臂尚未接入，实物运行仍阻止。

实物演奏前断开官方上位机，保持标定的手掌与琴键位置。模拟不发声、不连接设备。指令发送不等于实物触键或回弹成功。

## 本机依赖

运行入口依赖本地 `.venv-aero`、`midi文件/midi_piano/.venv` 和 `aero-hand-open/sdk/src`。这些环境、官方源码及原始 ZIP 不纳入本仓库；本仓库不是可独立复制运行的发布包。

官方 SDK：https://github.com/Chestnut-Robotics/aero-hand-open.git
当前本地官方仓库提交：`7bb2f23485a7d440960c21046da642de132b7745`，本次检查工作区干净。Python 3.12；MIDI 环境依赖见 `midi文件/midi_piano/requirements.txt`。

标定角度由官方界面归一化滑块值换算，不能直接将滑块值当角度。配置仅适用于当前标定硬件和摆放。

工作记录见 `工作日志.md`。
