# DM USB2CANFD SDK v2.1.0 使用说明（C / C++）

本目录是 **v2.1.0** 的对外交付物：C API 动态库 + 头文件。覆盖经典 CAN、CAN FD（BRS / TDC）、
周期发送、批量与队列发送、多订阅者收帧、总线状态与统计读取，以及模块自带的 USB2UART 串口透传。

> 支持平台：**Windows（x64，MinGW / MSVC 两套）**、**Linux（x86_64 / aarch64）**、
> **macOS（Apple Silicon / Intel）**。
> **升级前请看 [`../../UPDATE.md`](../../UPDATE.md)**：v2.x 相对 1.x 的接口**不兼容**，本页只讲怎么用。

---

## 1. 支持的模块与固件版本

| 模块 | USB 标识 | 固件版本要求 |
| --- | --- | --- |
| **USB2CANFD_S**（单路） | `VID 0x34B7 / PID 0x6877` | **V2.0.0 及以上** |
| **USB2CANFD_D**（双路） | `VID 0x34B7 / PID 0x6632` | **V2.0.0 及以上** |
| **LinkX4C**（四路） | `VID 0x34B7 / PID 0x4543` | **V2.0.0 及以上** |

- 固件信息用 `dmcan_device_get_firmware_info()` 读取（型号 + 发布版本号 + 构建标识），
  再用 `dmcan_firmware_info_to_string()` 格式化成人读的一行；构建标识由固件追加在
  `CMD_GET_VERSION` 应答尾部，老固件读不到时 `build_id_valid = false`（不算失败）。
- **固件构建标识要求**：单路 `d64e840` 起、双路 `b5c2574` 起、四路 `d5bc7db` 起
  （这是"与上位机接口兼容"的最低固件；固件版本号没变，只能按构建标识判——不确定就先升级到最新固件）。

---

## 2. 本目录有什么

```
lib/v2.1.0/
├─ dmcan.h            ← 对外头文件（4 个要一起用：dmcan.h / dmcan_types.h / dmcan_version.h / dmcan_export.h）
├─ dmcan_types.h
├─ dmcan_version.h
├─ dmcan_export.h
├─ windows/mingw/     libdmcan.dll、导入库 libdmcan.dll.a、运行期依赖（libusb-1.0.dll + 3 个 GCC 运行库）
├─ windows/msvc/      dmcan.dll、导入库 dmcan.lib、运行期依赖 libusb-1.0.dll
├─ linux/x86_64/      libdmcan.so        （libusb-1.0 用目标机系统自带的）
├─ linux/arm64/       libdmcan.so
├─ macos/arm64/       libdmcan.dylib + libusb-1.0.0.dylib（随包，解压即用）
└─ macos/x86_64/      libdmcan.dylib + libusb-1.0.0.dylib
```

---

## 3. 快速开始

### 3.1 集成

```cmake
# 头文件目录直接用 lib/v2.1.0（代码里写 #include "dmcan.h"）
target_include_directories(app PRIVATE <repo>/C&C++/lib/v2.1.0)

# Windows MinGW：
target_link_libraries(app PRIVATE <repo>/C&C++/lib/v2.1.0/windows/mingw/libdmcan.dll.a)
# Windows MSVC：
# target_link_libraries(app PRIVATE <repo>/C&C++/lib/v2.1.0/windows/msvc/dmcan.lib)
# Linux：
# target_link_libraries(app PRIVATE <repo>/C&C++/lib/v2.1.0/linux/x86_64/libdmcan.so)
# macOS：
# target_link_libraries(app PRIVATE <repo>/C&C++/lib/v2.1.0/macos/arm64/libdmcan.dylib)
```

- **Windows**：把 `windows/<工具链>/` 目录加进运行期搜索路径（或把其中的 DLL 拷到 exe 旁边）；
  MSVC 版的目标机需装 **VC++ 2015-2022 可再发行组件**；
- **Linux**：`-L<...>/linux/x86_64 -ldmcan`，运行时用 `LD_LIBRARY_PATH` 或把 `.so` 装到系统库目录；
  目标机需装 `libusb-1.0`；
- **macOS**：`-L<...>/macos/arm64 -ldmcan -Wl,-rpath,<...>/macos/arm64`，`.dylib` 与它依赖的 `libusb`
  同目录、一起分发即可（不加 `rpath` 就把两个 `.dylib` 拷到可执行文件旁）。

### 3.2 最小示例（打开 → 配置 → 收帧 → 发送）

```c
#include "dmcan.h"

static void on_recv(dmcan_device_handle *handle, dmcan_rx_frame_t *frame)
{
    (void)handle;
    printf("recv: ch=%u id=0x%X dlen=%u ts=%llu us\n", frame->channel, frame->can_id, frame->dlen,
        (unsigned long long)frame->timestamp);
}

int main(void)
{
    dmcan_context *ctx = NULL;
    dmcan_context_create(&ctx);
    if (ctx == NULL || dmcan_find_devices(ctx) <= 0) return -1;

    dmcan_device_handle *dev = NULL;
    if (!dmcan_device_get(ctx, &dev, 0) || !dmcan_device_open(dev)) return -1;

    dmcan_device_hook_recv_callback(dev, on_recv);

    dmcan_channel_can_info_t cfg = {0};
    cfg.canfd = true;   cfg.can_baudrate = 1000000;  cfg.can_sp = 0.75f;
    cfg.retrans = true; cfg.canfd_baudrate = 5000000; cfg.canfd_sp = 0.75f;
    dmcan_device_set_channel_baudrate(dev, 0, &cfg);
    dmcan_device_enable_channel(dev, 0);

    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    dmcan_device_send_can(dev, 0 /*通道*/, 0x123 /*ID*/, true /*CANFD*/, false /*扩展帧*/,
        false /*远程帧*/, true /*BRS*/, 8 /*长度*/, payload);

    /* ... 运行期间由回调收帧 ... */

    dmcan_device_close(dev);
    dmcan_context_destroy(ctx);
    return 0;
}
```

完整可编译版本见 [`../../example/v2.1.0/`](../../example/v2.1.0/)（纯 C）。

### 3.3 常用能力（都在 `dmcan.h` 里，按名字即可找到）

| 想做的事 | 用哪个接口 |
| --- | --- |
| 找设备 / 打开 / 关闭 | `dmcan_find_devices`、`dmcan_device_get`、`dmcan_device_open`、`dmcan_device_close` |
| 配置通道（波特率、采样点、CAN FD / BRS、回环测试模式） | `dmcan_device_set_channel_baudrate`（更细的时序/滤波用 `..._details`） |
| 开 / 关通道 | `dmcan_device_enable_channel`、`dmcan_device_disable_channel` |
| 发一帧 / 周期发 / 批量发 | `dmcan_device_send_can`、`dmcan_device_send_can_details`、`dmcan_device_send_can_batch`、`dmcan_device_send_can_bulk` |
| 收帧 | `dmcan_device_hook_recv_callback`（单个回调）或 `dmcan_device_subscribe`（多订阅者） |
| 读总线状态与统计 | `dmcan_device_get_bus_state`、`dmcan_device_get_channel_stats` |
| 读固件信息 | `dmcan_device_get_firmware_info` + `dmcan_firmware_info_to_string` |
| 日志 | `dmcan_set_log_level`；要落盘就注册日志回调自己写文件 |

---

## 4. 各平台运行前提

| 平台 | 前提 |
| --- | --- |
| Windows（MinGW 版） | `windows/mingw/` 里已带所需运行库，解压即用 |
| Windows（MSVC 版） | 目标机需装 **VC++ 2015-2022 可再发行组件** |
| Linux | 装 `libusb-1.0`；**设备权限**：普通用户默认打不开 USB 设备节点，加 udev 规则（`SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666"`）或用 root；用 USB2UART 串口时把用户加入 `dialout` 组 |
| macOS | 解压即用（`libusb` 随包）；**最低 macOS 11.0（Big Sur）**；按机器架构选 `macos/arm64`（M 系列）或 `macos/x86_64`（Intel） |

---

## 5. 使用注意

- **重新插拔或设备复位后**：要重新 `dmcan_device_close()` → `dmcan_device_open()`，并**重新使能通道**
  （复位后通道默认关闭）；
- **会话级设置不落盘**：软件终端电阻开关、通道 echo 开关等在重新枚举后要重新下发；
  用户订阅（`dmcan_device_subscribe`）跨重连保持；
- **USB2UART**：按"本机设备的序列号"匹配自动打开；**LinkX4C 没有**这个虚拟串口；
- **多设备**：用 `dmcan_find_devices` 的数量配合 `dmcan_device_get_info`（型号 / VID / PID / 序列号）区分，别写死下标 0；
- 更细的行为约定（回调线程、失败语义、返回值含义）以 `dmcan.h` / `dmcan_types.h` 的注释为准。

---

## 6. 版本与兼容

- 头文件 `dmcan_version.h` 里是本包版本号；运行时用 `dmcan_check_version(DMCAN_VERSION_INT)` 校验
  "库与头文件是否配套"（本版为 `0x02010000`）；
- **本版（v2.1.0）是 2.x 的首个对外版本**：与 1.1.0 及更早的接口**不兼容**，
  升级要点、迁移对照表见 [`../../UPDATE.md`](../../UPDATE.md)；
- 2.1.x 系列内部后续版本会保持"对外接口与结构体布局一致、换库即可升级"的口径。
