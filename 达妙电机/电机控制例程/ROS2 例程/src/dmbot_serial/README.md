#  达妙电机 ROS2 例程

本包提供 ROS2 控制节点示例，基于控制达妙电机 SocketCAN 通信库。

##### 注意：以下内容均基于SocketCAN的CANFD模式、5M波特率，电机、模块、代码务必统一，请根据实际情况调整。

## 使用准备
模块的SocketCAN专用固件 [gs_usb固件安装](https://gitee.com/kit-miao/dm-tools/tree/master/USB2CANFD/%E5%9B%BA%E4%BB%B6/socketcan/gsusb%E5%85%8D%E9%A9%B1%E5%9B%BA%E4%BB%B6)，烧录方式与普通固件一致；
##### 注意：模块安装SocketCAN专用固件后，将无法连接使用上位机！若要使用上位机，请先刷回普通固件
如需刷回普通固件或升级SocketCAN固件，请下载 [USB2CANFD通用升级工具](https://gitee.com/kit-miao/dm-tools/tree/master/USB2CANFD%E9%80%9A%E7%94%A8%E5%8D%87%E7%BA%A7%E5%B7%A5%E5%85%B7)。

##### 若主控为arm架构，需安装驱动 [gs_usb_drives](https://gitee.com/kit-miao/dm-tools/tree/master/gs_usb_drives)

## 运行环境
- Ubuntu 22.04 LTS
- ROS2 Humble

## 目录

```
dmbot_serial/
├── CMakeLists.txt                      # 编译配置
├── package.xml                         # 包描述与依赖声明
├── README.md                           # 本说明文档
├── include/dmbot_serial/
│   ├── dm_ros2_utils.hpp               # 参数解析工具
│   └── protocol/
│       ├── damiao.h                    # 达妙电机协议头文件
│       └── socketcan.h                 # SocketCAN 通信头文件
├── launch/
│   ├── test_motor_node.launch.py       # 电机控制节点启动文件
└── src/
    ├── test_motor_node.cpp             # 电机控制节点
    └── protocol/
        ├── damiao.cpp                  # 达妙电机协议实现
        └── socketcan.cpp               # SocketCAN 通信实现
```

## test_motor_node 电机控制节点示例

`test_motor_node` 通过一个 `std_msgs/msg/Float64MultiArray` 话题控制同一 CAN 接口下的多个电机。

- 电机由 `motor_types`、`motor_can_ids`、`motor_master_ids` 三个等长数组定义，数组顺序一一对应。
- 订阅 `~/target_command`，节点完整话题名为 `/test_motor_node/target_command`。
- `target_command` 中的指令按电机顺序拼接：先填第 1 个电机的参数，再填第 2 个电机的参数，依此类推。
- 单个电机的参数格式由 `control_mode` 决定：
  - `mit`：`[kp, kd, position, velocity, effort]`
  - `pos_vel`：`[position, velocity]`
  - `vel`：`[velocity]`
  - `pos_force`：`[position, velocity, effort]`
- `target_command` 总长度 = 电机数量 × 单个电机参数数量。
- 周期下发控制指令，并在终端打印当前位置、速度、力矩和错误码反馈。
- 支持 `pos_vel`、`vel`、`pos_force`、`mit` 控制模式。
- 可通过修改 launch 文件参数配置电机型号、CAN ID、MASTER ID、控制模式等参数。

启动示例：
##### 注意：运行控制前请确保环境安全。

```bash
cd "ROS2 例程"
colcon build --packages-select dmbot_serial
source install/setup.bash

# 激活can0接口 ，并设置为CANFD模式，波特率为5M
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
```

发送指令（--once 只发送一次）：

```bash
ros2 launch dmbot_serial test_motor_node.launch.py control_mode:=mit
# 默认两个电机：
# 电机1：kp=0.0, kd=0.0, position=0.0, velocity=0.0, effort=0.6
# 电机2：kp=0.0, kd=0.0, position=0.0, velocity=0.0, effort=0.6
ros2 topic pub /test_motor_node/target_command std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.6]}" --once
```

如果使用速度模式：

```bash
ros2 launch dmbot_serial test_motor_node.launch.py control_mode:=vel
# 默认两个电机：电机1 velocity=1.0，电机2 velocity=-1.0
ros2 topic pub /test_motor_node/target_command std_msgs/msg/Float64MultiArray \
  "{data: [2.0, -2.0]}" --once
```

如果使用位置速度模式：

```bash
ros2 launch dmbot_serial test_motor_node.launch.py control_mode:=pos_vel
# 默认两个电机：
# 电机1：position=0.5, velocity=1.0
# 电机2：position=0.5, velocity=1.0
ros2 topic pub /test_motor_node/target_command std_msgs/msg/Float64MultiArray \
  "{data: [1.0, 1.0, 1.0, 1.0]}" --once
```

如果使用位置力矩模式：

```bash
ros2 launch dmbot_serial test_motor_node.launch.py control_mode:=pos_force
# 默认两个电机：
# 电机1：position=0.5, velocity=1.0, effort=0.3
# 电机2：position=0.5, velocity=1.0, effort=0.3
ros2 topic pub /test_motor_node/target_command std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 1.0, 0.3, 0.5, 1.0, 0.3]}" --once
```

如果只有一个电机，把launch的三个数组都改为长度为 1，并只发送该电机的参数：

```bash
ros2 launch dmbot_serial test_motor_node.launch.py \
  motor_types:="['DM4310']" \
  motor_can_ids:="[0x01]" \
  motor_master_ids:="[0x11]" \
  control_mode:=vel
ros2 topic pub /test_motor_node/target_command std_msgs/msg/Float64MultiArray \
  "{data: [2.0]}" --once
```


## 参数配置

常用参数如下：

- `can_interface`：CAN 网卡名，默认 `can0`。
- `can_mode`：`can` 或 `canfd`，默认 `canfd`。
- `motor_types`：电机型号数组，例如 `['DM4310', 'DM4310']`。
- `motor_can_ids`：电机 CAN ID 数组，统一使用十六进制，例如 `[0x01, 0x02]`。
- `motor_master_ids`：反馈主站 ID 数组，统一使用十六进制，例如 `[0x11, 0x12]`。
- `control_mode`：`pos_vel`、`vel`、`pos_force`、`mit`。
- `send_rate_hz`：控制循环频率。
- `initial_position`、`initial_velocity`、`initial_effort`：各电机的初始目标值，所有电机共用。
- `mit_kp` / `mit_kd`：MIT 模式的初始刚度/阻尼系数，也可通过 `target_command` 话题动态下发。

注意：`motor_types`、`motor_can_ids`、`motor_master_ids` 三个数组长度必须一致，并按相同顺序一一对应。

## 新增电机型号

新增一款电机型号需要同时修改以下三个文件，缺一不可：

1. `include/dmbot_serial/protocol/damiao.h`：在 `DM_Motor_Type` 枚举中添加新型号。
2. `src/protocol/damiao.cpp`：在 `limit_param` 数组中添加对应的限位参数。
3. `include/dmbot_serial/dm_ros2_utils.hpp`：在 `parse_motor_type()` 中添加型号名映射，ROS2 参数里才能填新型号。

### 添加枚举类型

在 `include/dmbot_serial/protocol/damiao.h` 的 `DM_Motor_Type` 枚举中，在 `Num_Of_Motor` 之前添加新型号：

```cpp
enum DM_Motor_Type
{
    DM3507,
    DM4310,
    // ... 已有型号
    DMG6220,
    DMxxxx,       // 新增型号
    Num_Of_Motor
};
```

注意：新型号要加在 `Num_Of_Motor` 之前，并且不要插在已有型号中间，否则已有型号的下标会改变，`limit_param` 会对应错。

### 添加 limit_param

在 `src/protocol/damiao.cpp` 的 `limit_param` 数组末尾，按与枚举完全相同的顺序添加一组参数：

```cpp
Limit_param limit_param[Num_Of_Motor] =
{
    {12.566, 50, 5},     // DM3507
    {12.5, 30, 10},      // DM4310
    // ... 已有型号
    {12.5, 45, 10},      // DMG6220
    {Q_MAX, DQ_MAX, TAU_MAX},   // DMxxxx
};
```

`Q_MAX`、`DQ_MAX`、`TAU_MAX` 分别对应该电机的位置、速度、力矩限位，可在上位机查看或修改，不能填 `0`。

### 添加字符串映射

在 `include/dmbot_serial/dm_ros2_utils.hpp` 的 `parse_motor_type()` 中添加映射：

```cpp
if (token == "dmxxxx") return damiao::DMxxxx;
```

### 注意

- 三个文件必须一起改：漏改枚举会编译失败；漏改 `limit_param` 会导致限位参数缺失；漏改字符串映射会导致启动时报“未知电机型号”。
- 枚举顺序与 `limit_param` 顺序必须一一对应，不要插队或换序。
- 限位参数必须根据实际情况填写，不要填 `0`。
- 修改完成后重新编译：

```bash
cd "ROS2 例程"
colcon build --packages-select dmbot_serial
source install/setup.bash
```

## 常见问题

- 电机控制指令未生效：检查电机是否连接正确，是否已启动，是否已配置好 CAN 网卡。
- 如使用的电机不在当前例程型号列表中，请按照上述教程手动添加。
