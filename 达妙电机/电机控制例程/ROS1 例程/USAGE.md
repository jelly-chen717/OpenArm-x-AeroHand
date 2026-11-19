# ROS1 例程使用说明

## 概述
- 本文保留原 README 中的 ROS1 环境准备、USB 转 CANFD / USB 转 CAN 两条接入流程、权限配置和多电机修改说明。
- 测试环境为 Ubuntu 20.04 + ROS1 Noetic，最低 C++ 版本为 C++11。

## 环境准备
- 创建工作区：

```shell
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```

- `u2canfd` 路线使用达妙 USB 转 CANFD 设备。
- `u2can` 路线使用达妙 USB 转 CAN 设备。

## 使用 USB 转 CANFD

### 目录准备
- 将 `u2canfd/src` 放到 `~/catkin_ws/src` 下。

### 依赖与权限
- 安装 `libusb`：

```shell
sudo apt update
sudo apt install libusb-1.0-0-dev
```

- 检查设备连接，原 README 中给出的 VID / PID 为 `34b7 / 6877`：

```shell
lsusb
```

- 配置 udev 权限：

```shell
sudo nano /etc/udev/rules.d/99-usb.rules
```

```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
```

```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 获取设备序列号
- 编译并运行 `dmbot_serial`：

```shell
cd ~/catkin_ws
catkin build dmbot_serial
source devel/setup.bash
roslaunch dmbot_serial dev_sn.launch
```

- 将输出的 `Serial_Number` 写回 `dm_hw/src/DmHW.cpp`。

### 编译与运行
- 编译整个工作区：

```shell
cd ~/catkin_ws
catkin build
```

- 将工作区环境写入 `~/.bashrc`：

```shell
source ~/catkin_ws/devel/setup.bash
```

- 重新载入环境：

```shell
cd
source .bashrc
```

- 默认示例为 5M 波特率下的单个电机控制，运行：

```shell
roslaunch dm_controllers load_dm_hw.launch
```

### 多电机修改
- 原 README 给出的多电机示例是在 5M 波特率下，同时控制多个 DM4310 / DM4340 / DM8006。
- 需要分别修改：
  - `dm_hw/src/DmHW.cpp`
  - `dm_controllers/src/DmController.cpp`
- 原始截图仍保留在 `u2canfd/src/docs/` 下，可继续按图修改。
- 当 5M 波特率下连接多个电机时，原说明要求末端增加 `120` 欧终端电阻。

## 使用 USB 转 CAN

### 目录准备
- 将 `u2can/src` 放到 `~/catkin_ws/src` 下。

### 初始化与编译

```shell
cd ~/catkin_ws
catkin init
catkin build
```

- 同样把工作区环境加入 `~/.bashrc` 并重新载入。

### 设备检查与权限
- 检查串口设备：

```shell
cd
ls /dev/ttyACM*
```

- 配置串口权限：

```shell
sudo chmod -R 777 /dev/ttyACM*
```

### 运行与多电机修改
- 默认示例使用 `roslaunch dm_controllers load_dm_hw.launch`。
- 原 README 给出的多电机路径需要修改：
  - `dm_controllers/config/dm_motor.yaml`
  - `dm_controllers/src/DmController.cpp`
- 设备检查、权限设置和运行命令保持与单电机流程一致。

### 新增电机型号

新增一款电机型号需要同时修改：

1. `damiao.h`：在 `DM_Motor_Type` 枚举中添加新型号。
2. `damiao.cpp`：在 `limit_param` 数组中添加对应的限位参数。

#### 添加枚举类型

在 `damiao.h` 的 `DM_Motor_Type` 枚举中，在 `Num_Of_Motor` 之前添加新型号：

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

#### 添加 limit_param

在 `damiao.cpp` 的 `limit_param` 数组末尾，按与枚举完全相同的顺序添加一组参数：

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

#### 注意

- 漏改枚举会编译失败；漏改 `limit_param` 会导致限位参数缺失。
- 枚举顺序与 `limit_param` 顺序必须一一对应，不要插队或换序。
- 限位参数必须根据实际情况填写，不要填 `0`。
- 修改完成后重新编译：

## 参考
- `ros-control`: http://wiki.ros.org/ros_control
- `legged_control`: https://github.com/qiayuanl/legged_control
- 代码来源入口保留在原 README 提到的电机控制例程仓库路径中。
