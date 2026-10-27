# 七音单旋律 MIDI 演奏

导入 `.mid` 文件，识别音符、时值和速度，生成三指配合机械臂换位的演奏计划。当前可完整模拟曲目；OpenArm 尚未接入，实物运行被明确阻止。

## 快速使用

本机已配置独立环境。双击本目录 `Start-MIDI.cmd`，推荐先选 **3：快速模拟**，路径留空运行《欢乐颂》。菜单也支持输入其他 MIDI 路径，以及音轨、通道、移调和速度参数。

- **1 查看计划**：显示识别出的音符与时间表，保存 JSON。
- **2 按节奏模拟**：按降速后的真实时间运行，Ctrl+C 停止。
- **3 快速模拟**：虚拟时钟执行同一份完整计划，无需等待数分钟。
- **4 列出音轨**：用于选择多轨 MIDI 中的旋律声部。
- **5 实物运行**：当前返回“OpenArm 真实适配器尚未实现”，不会打开任何设备。
- **0 退出**：不执行曲目。

模拟仅输出动作记录，不产生钢琴声音，也不会连接串口。菜单中的“按节奏”指动作时间，不是音频播放。

复制到另一台电脑后，先安装 Python 3.10 或更新版本，再运行 `Setup-MIDI.cmd` 安装独立依赖。启动菜单不会自动安装或修改原有环境。

## PowerShell 命令

以下命令在 `D:\桌面\open CyMon\弹钢琴\midi文件` 中执行：

```powershell
# 默认曲目：查看音符和演奏计划
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py inspect

# 默认曲目：完整快速模拟
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py simulate --fast

# 七音上下行音阶：覆盖全部七音与三组位置
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py simulate .\midi_piano\examples\seven_note_scale.mid --fast

# 导入自己的 MIDI（含空格的路径用引号包裹）
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py inspect "D:\音乐\旋律.mid" --list
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py inspect "D:\音乐\旋律.mid" --track 1 --channel 1

# 例如把 C5–B5 移到 C4–B4；不自动折叠八度或删除音符
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py simulate "D:\音乐\旋律.mid" --track 1 --channel 1 --transpose -12 --speed 0.5 --fast

# 按实际调度时间模拟
.\midi_piano\.venv\Scripts\python.exe .\midi_piano\player.py simulate

# 测试
.\midi_piano\.venv\Scripts\python.exe -m unittest -v midi_piano.tests.test_player
```

计划保存在本目录 `output/<曲名>.plan.json`；再次运行同名曲目会更新该计划。可用 `--output <路径>.plan.json` 另存。操作不会改写输入 MIDI。

## 音域与指法

默认七个自然音为 C4–B4，中央 C 使用 MIDI 编号 60；不同软件的八度名称可能不同，以 MIDI 编号为准。

| 位置 | 食指 | 中指 | 无名指 |
|---|---|---|---|
| A | 60 / C4 / do | 62 / D4 / re | 64 / E4 / mi |
| B | 65 / F4 / fa | 67 / G4 / sol | 69 / A4 / la |
| C | 71 / B4 / si | 释放 | 释放 |

同组内仅切换手指；跨组先释放全部琴键，再移动手掌，到位后按压。A/B/C 需要各自标定手掌位置和姿态，不能仅凭理论白键间距推算。

支持常见 PPQN 时间制 Type 0/1 MIDI，读取全曲速度变化。轨道编号从 **0** 开始，通道编号从 **1** 开始。有多个音轨或通道包含音符时，需显式选择唯一旋律；没有自动选择最高音、删除伴奏或改编功能。

超出七音范围、黑键、和弦、音符重叠、打击乐通道、弯音及缺失 Note Off 会报错。若钢琴 MIDI 有连奏造成的微小重叠，也需要先在音乐编辑软件中整理为不重叠的单旋律。Type 2 和 SMPTE 时间制暂不支持。

音符力度被保留并显示，但不用于力矩或触键力度控制。延音踏板不参与演奏，预览会提示；时值以 Note On/Off 为准。

## 节奏如何转换为动作

1. 解析原曲每个音符的开始时间和时值，保留速度变化与休止。
2. 为每个音符分配手指与手掌位置。
3. 使用断奏，为释放和换位留出空隙：除最后一个音外，保持时长取原时值与下一音起始间隔的 `articulation` 倍中较小者。默认 `articulation=0.7`，最后一个音保持完整时值。
4. 根据按压、保持、释放、间隔和换位时间，求出整首曲子共同的时间倍率。只会降低请求速度，不会在局部挤压动作追赶节拍。
5. 初始准备与首次定位后，按绝对时间调度；计划中的 `onset` 是完成按压指令的目标时刻，并非传感器测得的实际发声时刻。

当前默认参数：按压 0.12 秒，释放 0.15 秒，最短保持 0.05 秒，释放后间隔至少 0.05 秒，初始准备 2 秒，每次换位估计 2 秒。`--speed` 是请求速度，例如 `0.5` 表示请求半速；实际仍可能更慢。

以这些估计值计算，《欢乐颂》原始约 21.33 秒，计划约 **251.74 秒**；七音上下行音阶计划约 **104.80 秒**。慢速来自跨组移动和预留释放空间。使用快速模拟可立即检查整个流程；真实演奏能力必须等实测换位时间后重新评估。

明显迟到、移动失败或串口异常会停止后续动作，尝试停止机械臂并释放手指，然后关闭连接。释放失败单独报告，不宣称琴键已经释放。

## 配置与硬件接入边界

`config.json` 独立于原有 `piano_config.json`，未知实物参数均为 `null`：

- `hand.port`：串口。
- `hand.rest_pose`：七个关节角度，依次为拇指外展、拇指屈曲、拇指联动屈曲、食指、中指、无名指、小指。
- `notes.<MIDI编号>.press_degrees`：该位置、该音符的按压角度；各组分别标定。
- `arm.poses.A/B/C`：待真实机械臂适配器定义并验证的命名位姿。当前不猜测坐标系、单位或关节排列。
- `arm.move_seconds`：六个有向组间移动时间，初始为模拟估计；`timing_is_measured=false`。

模拟使用抽象动作状态，不把未知角度替换为零。SDK 的关节角度不是上位机原始滑块值、电机角度或腱绳位移。

机械手串口后端已准备角度转换、当前位置读取、平滑插值与写入错误检查，并用模拟串口进行测试。实际机械臂适配器仍是明确抛错的占位实现。即使把 JSON 全部填满或把 `backend` 改成其他字符串，`run` 仍会拒绝连接。

后续接入真实 OpenArm 需要完成：

1. 根据实际设备确定 SDK/ROS 接口、连接参数、坐标系、单位和路径约束。
2. 实现 `connect / move_to / wait_until_arrived / stop / close`；到位必须依赖设备反馈，不能用固定等待假装成功。
3. 校验全部标定并连接适配器后，才替换 `validate_hardware` 中的无条件阻断，并接入 `run` 分支。
4. 实测 A/B/C 换位耗时、逐音触键与回弹，再低速演奏整曲。

运行真实设备前关闭其他占用同一机械手串口的程序；文件和环境独立不意味着可以同时控制同一硬件。

## 示例与文件组织

`examples/ode_to_joy.mid` 是自行编码的贝多芬《欢乐颂》开头八小节 C 大调单旋律简化版，30 个音，仅使用 do–sol；`examples/seven_note_scale.mid` 是 13 音上下行音阶，覆盖全部七音。生成脚本为 `make_examples.py`。

源码分为 MIDI 读取、纯动作规划、设备适配、计划执行和命令行入口。测试使用虚拟时钟、模拟机械臂和模拟串口，不访问实物。原三指代码、启动脚本、配置和虚拟环境均未改动。
