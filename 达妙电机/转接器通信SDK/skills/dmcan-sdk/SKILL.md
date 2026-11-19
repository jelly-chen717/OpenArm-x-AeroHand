---
name: dmcan-sdk
description: 用达妙 USB2CANFD（dmcan）SDK 写上位机代码、做版本迁移或现场排错时使用——C/C++、C#、Python、Rust 四种封装各自拿哪份产物、原生库要放在哪里，以及库/头/固件配套要求、打开与配置通道的顺序约束、现场问题的排查顺序。
metadata:
  short-description: 达妙 USB2CANFD SDK（四种语言封装）移植与排错
---

# 达妙 USB2CANFD SDK（`dmcan`）——按语言平台移植与排错

本仓库是 SDK 的**对外发布仓**：C / C++ 的库与头文件按版本放在 `C&C++/lib/<版本>/`；
C#、Python、Rust 三种封装在各自目录下（`CSharp/` / `Python/` / `Rust/`），发布产物在 `<语言>/pack/`。

## 何时用

- 用任意一种封装（C/C++、C#、Python、Rust）写或改上位机代码；
- 把 v1.x 的代码迁到 v2.x；
- 排查“打不开设备 / 收不到帧 / 丢帧 / 读不到版本 / 原生库加载失败”这类问题；
- 判断“库与头是否配套”“模组固件够不够新”。

不适用：固件升级、恢复出厂、读写 SN 等设备管理接口（不在公开包里）。

## 第一步：按语言平台拿产物

| 语言 | 产物 | 自带原生库？ | 还需要什么 |
| --- | --- | --- | --- |
| **C / C++** | `C&C++/lib/v2.1.0/`（四个头在该目录根，库按平台分子目录） | ✅ Windows / macOS 连运行期依赖一起带；Linux 用系统 `libusb-1.0` | MSVC 目标机装 VC++ 2015-2022 可再发行组件 |
| **C# / .NET** | `CSharp/pack/DM_DeviceSDK_CSharp.2.1.1.nupkg`（NuGet 同名包，当前 **2.1.1**，目标 `net8.0`） | ✅ 按 RID：`win-x64` / `linux-x64` / `linux-arm64` / `osx-arm64` / `osx-x64` | 直接 `dotnet add package DM_DeviceSDK_CSharp`，**不用**另装 C SDK |
| **Python** | `Python/pack/dmcan_sdk-2.1.1-py3-none-any.whl` | ❌ 纯 ctypes（wheel 是 `py3-none-any`） | **必须自备 `libdmcan` 及其依赖**（见第二步） |
| **Rust** | `Rust/pack/dmcan-2.1.0.crate` | ✅ 含各平台 `libdmcan`；❌ **不含 libusb** | macOS / Windows 需自备 libusb，或设 `DMCAN_SDK_DIR` 指向完整 C 交付包 |

> **版本口径**：原生库与头是 **2.1.0.0**（`dmcan_version.h` 是唯一真源）。C# `2.1.1`、Python `2.1.1`
> 是**封装包**的版本（相对 2.1.0 只有元数据 / 文档变化），绑定的后端仍是 2.1.0；Rust crate 版本 2.1.0。
> 迁移前看 `C&C++/UPDATE.md`：**v2.x 与 v1.1.0 的接口不兼容**，v1.1.0 的老例程（`usb_rx_frame_t`、
> `dmcan_device_fill_can_queue` 等）不要照抄。

## 第二步：把原生库放到正确位置

- **C / C++**：Windows → `windows/mingw|msvc/`（导入库与运行期依赖齐全）；Linux → `linux/<arch>/libdmcan.so`；
  macOS → `macos/<arch>/` 下的两个 dylib **必须同目录**。
- **C#**：NuGet 自动铺到 `runtimes/<rid>/native/`，不要手工改；win/osx 的包内带 libusb，Linux 用系统 libusb。
- **Python**：查找顺序 = 环境变量 `DMCAN_LIBRARY_PATH`（给完整路径）→ 包内 `dmcan/dlls/` →
  程序目录 `dlls/` → 程序目录 → 系统路径；Windows 上 `libusb-1.0.dll`、`libstdc++-6.dll`、
  `libgcc_s_seh-1.dll`、`libwinpthread-1.dll` **必须与 `libdmcan.dll` 同目录**（`Python/demo/dlls/` 已按这个摆好，可照抄）。
- **Rust**：`tar -xf Rust/pack/dmcan-2.1.0.crate` 后当本地依赖（`dmcan = { path = "dmcan-2.1.0" }`）；
  crate 内 `native/<平台>/` 自带 `libdmcan`，**但 macOS 的 `libusb-1.0.0.dylib`、Windows 的 `libusb-1.0.dll`
  要自备并放到与 `libdmcan` 同目录**（LGPL，crate 不含；缺了运行期直接加载失败）。
- **平台硬约束（四种封装通用）**：
  - macOS 最低 **11.0**；`libdmcan.dylib` 按 `@loader_path/libusb-1.0.0.dylib` 找 libusb，两者**必须同目录**；
  - Linux 普通用户默认**打不开**设备节点（`/dev/bus/usb/...` 是 `root:root 0644`，只读不写）→
    加 udev 规则（`SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666"`）或用 root 跑；
  - Windows / MSVC 目标机装 VC++ 可再发行组件；MinGW 那套要连 `libgcc_s_seh-1` / `libstdc++-6` /
    `libwinpthread-1` 三个 DLL 一起带；
  - 架构对齐：arm64 机器用 arm64 的库（macOS 上跑 x86_64 库需要 Rosetta）。

## 第三步：库、头、固件三者的配套

1. **库与头**：C 侧用 `dmcan_check_version(DMCAN_VERSION_INT)` 校验；C# / Python / Rust 封装在创建上下文时
   会自动校验（主版本必须相同、次版本不低于 2.1），不配套会明确报错，不会静默读到错位结构体。
2. **模组固件**：要 **V2.0.0 及以上**，且构建标识不低于 单路 `d64e840` / 双路 `b5c2574` / 四路 `d5bc7db`。
   固件版本号一直是 `V2.0.0`，**判据是构建标识**（`dmcan_device_get_firmware_info()`；Rust 是 `dev.firmware_info()`）。
   `version_valid == false` 表示设备在 boot（不是设备坏了）；判“升级是否生效”看 **`build_id`**。
3. **v2.1.0 删掉的旧接口**：`dmcan_device_get_version*()` / `_print_version()` / `version_info_t` **已删除**，
   改用 `dmcan_device_get_firmware_info()` + `dmcan_firmware_info_to_string()`。

## 非显然的约束（四种封装都是 C ABI 的 1:1 映射，下面这些同样成立）

- **顺序**：先 `dmcan_device_set_channel_baudrate*()`，再 `dmcan_device_enable_channel()`；
  设备复位或重新插拔后要 `close()` → `open()` 并**重新使能**（复位后通道默认关闭）。
- **长度自动取整**：`dlen` 会自动向上取整到最近的 DLC（9 → 线上 12 字节，不足补 0）；
  接收侧读到的是**线上长度**，别拿请求长度去比。
- **回调线程**：收帧回调跑在 SDK 的通知线程上、同一设备串行；里面不要阻塞、不要做重活，
  **不要在回调里 `dmcan_device_close()`**（会被拒并置 `DMCAN_ERR_REENTRANT`）。
- **错误码**：返回 `false` 时用 `dmcan_get_last_error()`（**线程局部**）取原因，取完可 `dmcan_clear_last_error()`。
- **统计是 1 Hz 快照**：`dmcan_device_get_channel_stats()` 刚打开或刚发完立刻读可能返回 false，等一拍再读；
  判丢包要**连着读两次看增量**，单次绝对值可能是很久以前的累计。
- **会话级设置不落盘**：软件终端电阻开关、通道 echo 开关、UART 线参数在重新枚举后要重新下发；
  用户订阅（`dmcan_device_subscribe`）跨 close / 重连保持。
- **不接线自检**：把通道配成 `DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL`（`mode = 2`，经 `..._set_channel_baudrate_details()`）
  就能单机自发自收，用来验证整条收发链路。

## 按需展开

| 任务 | 看哪里 |
| --- | --- |
| C / C++ 集成、最小可编译程序、接口地图、FAQ | `C&C++/USAGE.md`、`C&C++/lib/v2.1.0/README.md` |
| C / C++ 可直接编译的示例 | `C&C++/example/v2.1.0/` |
| v1.x → v2.x 迁移对照 | `C&C++/UPDATE.md` |
| C# 包安装（含离线 `pack/` 源）与示例 | `CSharp/README.md` |
| Python 安装、后端库查找顺序、示例 | `Python/README.md` |
| Rust `.crate` 引入方式、libusb 自备、最小示例 | `Rust/README.md` |
| 权威接口说明（调用时机 / 失败语义 / 线程约定） | `C&C++/lib/v2.1.0/dmcan.h`、`dmcan_types.h` |

## 现场排错顺序

1. **原生库加载失败**：先看是否缺 libusb 及其它运行期依赖，以及它们是否与 `libdmcan` **同目录**
   （macOS 最典型；Python / Rust 自备 libusb 时最容易漏）；
2. **打不开设备**：看错误码——Linux 多半是设备节点权限（udev / root），Windows 多半是被别的进程独占；
3. **发了收不到**：两端波特率与采样点、通道是否已使能、终端电阻、经典 CAN 与 FD 是否搞混；
   先用外部回环模式确认“自己能收自己的帧”，再怀疑对端；
4. **丢帧**：读通道统计的丢弃细分，区分“设备侧队列满”与“主机读得不够快”（回调里别做重活 / 落盘）；
5. **读不到版本**：多半是设备在 boot（`version_valid == false`），或固件没带构建标识（`build_id_valid == false`）。

## 仓库操作约定

- 这个仓库是对外发布仓：**默认只改本地**；推送（尤其强推改写历史）必须先跟仓库主人确认。
- 四种语言的产物由各自会话更新：**任何一种语言换了原生库，都要确认四个目录里的 `libdmcan` 是同一版**
  （比对文件 SHA256，基准是 `C&C++/lib/v2.1.0/` 下的同名库），否则用户按语言移植时会拿到旧库。
- 改文档 / 产物后同步更新 `C&C++/UPDATE.md` 与对应语言的 `README.md`，保持四处口径一致。
