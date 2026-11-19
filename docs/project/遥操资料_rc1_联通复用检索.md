# 遥操资料 rc1：仿真与实机联通复用检索

检索日期：2026-11-15。目标：实机反馈同步 MuJoCo，以及 MuJoCo 关节目标驱动实机。

## 结论与来源

用户指定的 `RobotControl-Windows11-WSL2-x64-1.0.0-rc1-20261111.zip` 含 33 个 ZIP 成员，源码位于 `payload/sources.tar.gz`，其中有 926 个普通文件。本次将选定文本、URDF/Xacro 共 519 个文件保存至 `resources/teleop_rc1_review/`，清单和 SHA-256 见该目录 `review_manifest.json`。未解包 Docker 镜像与 WSL 根文件系统，未运行安装、启动、使能或运动程序。

原来的 `resources/openarm_control_extract/` 来源是 rc2，不能作为 rc1 的同版本证据。本次以下结论来自 rc1 实际读取的源文件。用户确认来源项目已完成遥操实机控制；发布包的 `VALIDATION.json` 则主要记录部署与离线检查，并明确新机器标定、复位和实际运动未验收，两者适用范围不同。

下表路径以 `resources/teleop_rc1_review/` 为根；`R` 表示 `sources/runtime`，`H` 表示 `R/ros2_ws/src/openarm_ros2`，`D` 表示 `R/ros2_ws/src/openarm_description`。

## 已定位的在用链路

```text
start.py → runtime/ops/manage.py（安装时由 portable_manage.py 替换）
  → ops/supervisor.py
      ├─ app/raw_standby.launch.py
      │    ├─ V1 双臂模型 + ros2_control_node
      │    └─ app/archive_bridge_adapter.py → 原 jt_bridge.py
      └─ python -m teleop_xr.ros2（saved_custom_python 优先）

XR 输入 → IK/输出许可/短滤波 → /joint_trajectory
  → jt_bridge 按关节名重排
  → /left_forward_position_controller/commands
    /right_forward_position_controller/commands
  → OpenArmHW::write → mit_control_all → CAN FD → 电机

电机 → OpenArmHW::read → ros2_control 状态接口
  → joint_state_broadcaster → /joint_states
```

启动证据：`start.py`、`adapt_runtime.py`、`R/ops/supervisor.py:86,106,115`、`R/app/raw_standby.launch.py`。实际启用哪些控制器仍由生命周期/人工准入决定，不能从 YAML 声明认定已激活。

`archive_bridge_adapter.py` 校验已安装桥源码 SHA；本次提取出的 install 目录 `jt_bridge.py` SHA-256 为 `1e5d17c0dcaecd2ce46e1005d878d750978f9930180b1c348719f739b2612ccb`，与适配器常量一致，确认桥版本引用相符。这里确认的是文件版本，不是本机运行状态。

## 对我们最有用的接口

| 内容 | 具体依据 | 复用方法与边界 |
| --- | --- | --- |
| 实测反馈 | `H/openarm_hardware/src/openarm_simple_hardware.cpp:421` 的 `read()`；`R/app/ros_status.py:180` | 臂七轴直接读取电机位置、速度和力矩；ROS 路线订阅 `/joint_states` 并按名称映射到 MuJoCo，Windows 路线采用现有驱动反馈。需自行保留逐关节接收时间与过期判断 |
| 关节目标入口 | `H/openarm_teleop_bridge/openarm_teleop_bridge/jt_bridge.py:48,58,166` | `/joint_trajectory` 为 `trajectory_msgs/JointTrajectory`；按 `joint_names` 提取每侧 J1～J7，转成 `Float64MultiArray`。每侧缺关节则该侧不发布；只读取 `points[-1].positions`，不执行整段轨迹时间 |
| 底层位置命令 | 同桥及 `H/openarm_bringup/config/controllers/openarm_bimanual_controllers.yaml` | 每侧 `/left_forward_position_controller/commands` 或右侧对应话题，7 个位置值依次 J1～J7，角度 rad。直接发布会绕过桥许可，因此必须有专用执行状态机、独占写入者与预检 |
| MIT 执行 | `H/openarm_hardware/src/openarm_simple_hardware.cpp:526,595,598` | 位置直接使用 `pos_commands_[i]`，目标速度为 0，前馈为可选重力项；不是带时间轨迹执行器，也不是 POS_VEL。仿真输出速度不能被误认为这里已用于电机控制 |
| 电机配置 | `R/ros2_ws/src/openarm_can/include/openarm/config/openarm_v10_config.hpp` | J1/J2=DM8009，J3/J4=DM4340，J5～J7=DM4310；发送 ID 1～7，反馈 0x11～0x17；夹爪另用 8/0x18，不适用于 Aero 手指 |
| MIT 增益 | 同上；`D/assets/robot/openarm_v1.0/config/arm/control_gains.yaml` | kp=[70,70,70,60,10,10,10]，kd=[2.75,2.5,2,2,0.7,0.6,0.5]；可作原项目默认依据，是否有运行时覆盖及是否适用当前负载需核对 |
| 总线参数 | `restore_transport.reference.py:51`；`RELEASE.json` | can0=右、can1=左；1/5 Mbps，采样点 0.75，仲裁 SJW=1，修复数据 SJW=2；Windows PCAN 句柄需单独映射，不把 Linux 名称直接等同于句柄 |
| 控制周期 | 上述 controllers YAML；`R/app/vr_smoothing_config.json` | controller_manager 配置 100 Hz；只是配置目标。实际本机采样、发送周期必须测量，不能据此声称稳定达到 100 Hz |

## 坐标、零位与限位：已找到什么

硬件层对臂的位置读写没有出现额外逐关节符号/偏移换算：反馈直接上报，位置命令直接发送。这说明原 ROS 模型坐标与其电机反馈坐标被当作一致使用；不能直接证明我们当前 MuJoCo 模型也一致。

关键依据为 `D/assets/robot/openarm_v1.0/urdf/arm/openarm_arm.xacro` 与 `openarm_macro.xacro`，而不只是零位标定脚本：

- 左右臂使用镜像几何；J1 左臂限位额外偏移 `-2.094396`；J2 限位有左右侧的 `±pi/2` 偏移和镜像处理。
- `config/arm/kinematics_offset.yaml` 中 J2 的 roll 为约 `pi/2`，这是模型固定坐标系旋转，不等于电机零点寄存器或可直接套用的 `p_ref`。
- `vr_smoothing_config.json` 列出的展开后限位：左 J1=[-3.490659,1.396263]、右 J1=[-1.396263,3.490659]；左 J2≈[-3.316125,0.174533]、右 J2≈[-0.174533,3.316125]；J3/J5/J7=±pi/2，J4=[0,2.443461]，J6=±pi/4。必须和当前 MuJoCo 的轴、固定变换及限位逐项比较。
- `RELEASE.json` 的 `motor_calibration_imported=false` 表示发布包未导入设备标定。已检索文件未提供当前设备逐关节 `m_off/dir` 读数与标定姿态表，部分零位关系可能已在电机内部保存，需后续查询与姿态验证。

因此，接入优先比较两套模型并利用已有反馈，不先重跑寻找止挡/写零程序。相同关节名字、同为 V1，并不足以确认可以直接一比一发送。

## 可复用的保护，以及不能误认为已有的保护

- `raw_standby.launch.py` 将两侧硬件置为 unconfigured，`return_to_zero=false`；硬件 `on_activate()` 会使能，默认锁存当前位置保持。第一天若走 ROS 状态链路，必须审查 configure 阶段能否获得状态，不能为了获取反馈随意调用 activate。
- `archive_bridge_adapter.py` 已有独占动作、完整新鲜反馈检查、过期事务拒绝、关闭许可取消复位；`staged_arm_motion.py/json` 实现 J2 先侧抬、其他关节后动、按反馈到位推进。复用状态机思路，但原收臂路径与钢琴/Aero 场景是否相容需另验，不自动调用。
- `vr_smoothing_config.json` 当前 `mode=source`，采用权重 [0.6,0.3,0.1]、历史最长 0.08 秒的短滤波。`vr_output_runtime.py:729` 明确该模式未应用 OTG 速度/加速度/jerk 限制，也未应用 OTG 跟踪误差锁。JSON 中存在这些参数不代表当前链路启用它们；profile 也标记 `validated_for_hardware=false`。
- `OpenArmHW::monitor_health()` 能识别陈旧数据和总线故障，但所读实现主要记录 `RCLCPP_ERROR`，`read()` 仍返回 OK；不能把故障日志当作自动停止保证。我们的执行入口必须明确超时、误差和断链后的动作。
- 桥仅停止发布时，前向位置控制器仍可能保持最后命令；关闭桥输出许可与电机失能不同。

## 推荐落地方式

第一天沿用本工作区 Windows PCAN 驱动作为唯一总线拥有者，将实测反馈写入独立 MuJoCo 镜像入口；借鉴包内 `can_observer.py` 的过滤与新鲜度统计，并比较包内 V1 模型和当前模型。这样无需为反馈镜像先部署完整 WSL、Docker、相机和 VR。

第二天在专用仿真实机入口中，将仿真关节目标经已验证映射、插补和限位后发给现有驱动，复用包内 MIT 配置、原控制规律及状态机经验。如果原 ROS/WSL 环境已经可用且决定继续使用，则可替换 `/joint_trajectory` 的生产者，保留适配桥和底层执行；须停用原 VR 输出、定时发布单点且保证唯一命令来源。两条路线择一拥有总线，不并行写 CAN。

下一步产物是左右各七轴的“包内模型 → 当前 MuJoCo → 电机反馈”对应表，以及本机环境检查结果；A01 尚未满足环境导入和模型加载验收，保持进行中。随后 A02 核对总线归属，再做 A03 现场反馈查询。此检索未改变硬件状态，也未完成任何实机阶段验收。
