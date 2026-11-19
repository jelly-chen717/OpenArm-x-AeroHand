# Windows OpenArm CAN FD 驱动

入口为 `apps/openarm_driver.py`，使用已安装的 `PCANBasic.dll`。Python 需要 3.11 或更新版本。协议和默认电机参数参照 `resources/openarm_control_extract/openarm_can/`，位时序参照提取目录中的 `reference/restore_transport.reference.py`。

## 查询状态

在工作区根目录运行以下命令，或双击 `launchers/Check-OpenArm.cmd`。

```powershell
.\.venv-aero\Scripts\python.exe apps/openarm_driver.py status
.\.venv-aero\Scripts\python.exe apps/openarm_driver.py status --side right
.\.venv-aero\Scripts\python.exe apps/openarm_driver.py monitor --side both --duration 10
```

默认逐臂查询七个关节，使用 `--gripper` 增加每臂 ID 8 的原装夹爪。状态命令发送状态刷新请求，返回位置、速度、力矩、驱动状态、两项温度、反馈年龄和原始字节。每次查询等待新回复，超时项显示 offline。适配器状态通知经过单独过滤，总线错误会终止该轮后续请求。

每个状态请求在超时后最多重试一次，适配器刚初始化时出现的首次请求丢失可由此恢复。总线错误直接报告。

结果写入 `logs/openarm/` 中的 JSONL 文件，可通过 `--log` 指定新文件路径。返回码 0 表示本次所有查询关节在线、状态为 DISABLED 或 ENABLED 且总线状态为零，返回码 2 表示检测存在异常。未知状态保留原始编号。

## 通道与位时序

| 配置 | 值 |
| --- | --- |
| 外壳 CAN0 | 右臂，用户已核对 |
| 外壳 CAN1 | 左臂 |
| 默认 Windows 句柄 | 右 0x51，左 0x52 |
| 仲裁速率 / 数据速率 | 1 Mbps / 5 Mbps |
| 仲裁 / 数据采样点 | 75% / 75% |
| 仲裁 / 数据 SJW | 1 / 2 |
| 帧格式 | 标准 ID，CAN FD，BRS，8 字节 |

Windows 句柄与物理插口的对应关系仍需通过单侧连线实测。可用 `--right-channel 0x52 --left-channel 0x51` 显式交换软件绑定。通道只能由一个驱动实例拥有，使用前关闭其他占用同一通道的程序。

## Python 控制接口

```python
from apps.openarm_driver import OpenArmDriver

with OpenArmDriver(side="right", channel=0x51) as arm:
    state = arm.query_motor(1)
    print(state.position_rad, state.velocity_rad_s, state.torque_nm)
    print(arm.read_parameter(1, 10))  # CTRL_MODE，1 表示 MIT
```

驱动另提供显式 `enable_motor(motor_id)`、`disable_motor(motor_id)` 和 `command_mit(motor_id, position, velocity, kp, kd, torque=0.0)`。使能前查询 CTRL_MODE 和失能反馈，MIT 发送前检查协议量程、有限值、反馈新鲜度和 CTRL_MODE。控制异常时尽力请求当前关节失能，无法确认时在异常备注中报告。多轴协调停止由上层调用者管理。

enable_motor 可传入 kp、kd，缺省采用参考库的关节增益。该方法以新读取的位置构建零速度、零前馈目标，在使能前和紧随使能后发送，并在有界时间内确认状态切换。使能和失能时在途的旧状态帧会继续等待新状态确认。

位置单位为电机 rad，速度 rad/s，力矩 Nm。默认型号为 J1/J2 DM8009、J3/J4 DM4340、J5/J6/J7 DM4310。协议量程参与编码和解码，模型关节限位、实机零位方向、轨迹规划、载荷和重力补偿需要由上层工程处理。使能会接通关节输出，使用控制 API 前应完成这些实机核对。command_mit 同步等待反馈并读取模式，适用于接入调试，周期性能需实测。

打开连接只初始化适配器。退出上下文释放适配器。显式使能后的调用方负责调用 disable_motor 并核对反馈，再结束连接。通信中断时的软件失能请求可能无法送达。

## 验证

离线回归命令如下。

```powershell
.\.venv-aero\Scripts\python.exe -m unittest discover -s apps -p test_openarm_driver.py -v
```

测试覆盖精确位时序、FD+BRS 封装、不同电机量程、无效帧、反馈超时、旧反馈隔离、模式检查、部分使能失败清理和正常命令行入口。离线通过后还需运行状态查询完成设备通信验证。

2026-11-14 验证记录：新驱动 20 项离线测试通过，原单电机程序 18 项离线回归通过。真实状态查询和短时连续监测均读到每臂 7 个关节，全部状态为 DISABLED，两通道状态为 0x0，记录见 logs/openarm。首次查询两臂 J1 各有一次超时，后续连续读取全部成功，驱动已加入一次超时重试。实机验证覆盖反馈通信，使能、MIT 运动、机械臂零位方向、负载能力及物理端口与左右臂反馈归属仍待单独核对。

### 2026-11-14 逐关节使能验证补记

用户确认机械臂已固定并授权逐关节验证后，运行 apps/check_openarm_enable.py。以当前反馈位置为目标，Kp=1、Kd=0.2、速度与前馈力矩为零，每个关节保持约 0.5 秒，逐个确认失能后再测试下一关节。每关节使能前读取 CTRL_MODE、PMAX、VMAX、TMAX 并对照型号量程，每关节结束后重新核对双臂 14 个关节均已失能。

首轮右 J1 因在途失能状态帧导致使能确认失败，清理已确认失能。修正状态切换等待并增加回归用例后，新驱动 22 项离线测试通过，完整第二轮 14/14 关节通过。最大反馈位置变化为 0.0003815 rad，最终两条总线状态为 0x0、14 个关节均为 DISABLED。完整记录为 logs/openarm/enable_check_1794663192665752300.json。

此次验证范围为机械固定条件下、低增益、当前位置的逐关节使能与失能。自由运动、运动方向、轨迹、重力补偿及带载保持能力需继续实测。
