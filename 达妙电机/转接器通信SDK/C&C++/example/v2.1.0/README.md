# DM_Device_SDK v2.1.0 示例程序（C）

## 概述

本示例演示 v2.1.0 的最小接入流程：**打开设备 → 配置通道（1M@5M CAN FD + BRS）→ 注册收帧回调 →
发一帧 → 启动周期发送 → 停止 → 关闭**。用 **纯 C** 编译，顺带兜住"对外是 C 接口"这条承诺。

> 与 v1.1.0 示例的区别：接口是 v2.x 那一套（`dmcan_rx_frame_t`、按指针传的波特率配置、
> `dmcan_device_stop_send()` 停周期发送）。升级要点见 [`../../UPDATE.md`](../../UPDATE.md)。

## 硬件支持

- USB2CANFD_S（单路，`0x34B7 / 0x6877`）
- USB2CANFD_D（双路，`0x34B7 / 0x6632`）
- LinkX4C（四路，`0x34B7 / 0x4543`）

## 目录

```
example/v2.1.0/
├─ README.md
└─ src/
   ├─ main.c              ← 示例源码（纯 C）
   └─ dmcan.h / dmcan_types.h / dmcan_version.h / dmcan_export.h   ← 对外头文件
```

## 编译与运行

库文件在 `../../lib/v2.1.0/<平台>/`，头文件可直接用本目录 `src/` 里的这一份（与库同版本）。

```bash
# Windows（MinGW）
gcc src/main.c ../../lib/v2.1.0/windows/mingw/libdmcan.dll.a -o demo.exe
#  运行前把 ../../lib/v2.1.0/windows/mingw 加进 PATH（或在同目录）

# Linux（目标机需装 libusb-1.0；普通用户需要 udev 规则或用 root）
gcc src/main.c ../../lib/v2.1.0/linux/x86_64/libdmcan.so -Wl,-rpath,$PWD/../../lib/v2.1.0/linux/x86_64 -o demo

# macOS（按架构选 arm64 / x86_64）
clang src/main.c ../../lib/v2.1.0/macos/arm64/libdmcan.dylib -Wl,-rpath,$PWD/../../lib/v2.1.0/macos/arm64 -o demo
```

运行：`./demo [通道号]`（缺省 0）——会在该通道发两帧（单次 + 周期），并在回调里打印收到的帧。

## 说明

- MSVC 用 `../../lib/v2.1.0/windows/msvc/dmcan.lib`，并确保目标机装了 VC++ 2015-2022 可再发行组件；
- 完整使用说明（各平台前提、常用接口清单、使用注意）见
  [`../../lib/v2.1.0/README.md`](../../lib/v2.1.0/README.md)。
