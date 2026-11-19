# OpenArm 控制资料提取

来源为 RobotControl-Windows11-WSL2-x64-1.0.0-rc2-20261111.zip 内的 payload/sources.tar.gz。提取日期为 2026-11-14。

## 当前 Windows 接入优先入口

| 文件或目录 | 用途 |
| --- | --- |
| openarm_can/include/openarm/config/openarm_v10_config.hpp | V1 七轴电机型号、发送和反馈 ID、默认 MIT 增益 |
| openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp | MIT 指令编码、参数读取、状态刷新和反馈解码 |
| openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp | 电机协议量程、控制模式、寄存器和状态定义 |
| openarm_can/src/openarm/damiao_motor/dm_motor_device.cpp | 经典 CAN 与 CAN FD 帧封装，FD 使用 BRS |
| openarm_can/setup/cli/commands/monitor_motor_status_commands.cpp | 状态监测流程 |
| openarm_can/setup/cli/commands/diagnose_commands.cpp | 通信诊断流程 |
| openarm_can/examples/health.cpp | 关节健康检查参考 |
| reference/can_observer.py | 双臂反馈过滤、新鲜度和在线状态统计 |
| reference/restore_transport.reference.py | 实际部署的适配器身份和位时序参考 |
| reference/prepare_can_20261020.py | 接口身份、位时序与错误状态校验参考 |

## 已核对的传输参数

- USB VID/PID 为 0c72:0011。
- 物理通道 CAN0 对应右臂，CAN1 对应左臂。Windows SDK 通道索引与物理插口仍需实测核对。
- CAN FD 仲裁速率为 1000000 bit/s，数据速率为 5000000 bit/s。
- 两阶段采样点均为 0.75，仲裁 SJW 为 1，数据 SJW 为 2。
- 状态刷新帧使用标准 ID 0x7FF，8 字节负载为 [电机ID低字节, 电机ID高字节, 0xCC, 0, 0, 0, 0, 0]。
- 包内 FD 发送实现设置 BRS。
- 七轴发送 ID 为 0x01 至 0x07，反馈 ID 为 0x11 至 0x17。夹爪使用 0x08 / 0x18，适用设备需核对。

## 依赖与适用范围

openarm_can 的 CMake 版本为 1.4.0，底层使用 Linux SocketCAN 和 POSIX 头文件。其 Python 包通过 pybind11 绑定 C++ 库，构建依赖见各自 CMakeLists.txt 和 pyproject.toml。Windows 原生接入可复用协议、解析规则与设备配置，传输由 resources/savvycanfd/SDK 中的 Windows/Python SDK 承接。

reference 中的脚本依赖 Linux、原 Docker 镜像或既有项目路径，用于审查原始实现。单独提取后需要适配运行环境。can_observer.py 的观察模型包含每臂七关节及一只夹爪。

ros2_reference/openarm_hardware 保存包内 ROS 2 硬件接口，包含反馈新鲜度、总线错误处理、可选重力补偿和激活逻辑。其 on_activate 会使能电机，return_to_zero 默认 false，激活后采用当前位置保持流程。该模块依赖 ROS 2、OpenArmCAN 及其构建文件列出的依赖。

model_config_reference 保存 V1/V2 的模型参数、关节限位和增益参考。当前机械臂的型号、零位、方向及增益仍需按实物确认。已有运动学工程位于 openarm/openarm，现有模型资源位于 resources/openarm_description。

## 校验

extraction_manifest.json 记录每个提取文件的原始成员路径、大小和 SHA-256。提取内容保持原字节，原许可证随源码保留。校验针对提取完整性，编译、Windows 适配和设备通信验证需要后续完成。
