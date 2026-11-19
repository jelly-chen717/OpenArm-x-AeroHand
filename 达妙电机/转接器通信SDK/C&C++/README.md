# DM_Device_SDK — C / C++ SDK

达妙 USB 类 CAN(FD) 设备的 **C 接口动态库 + 头文件**（纯 C ABI，C 与 C++ 都能直接调）。

| 模块 | USB 标识 | 通道 | 固件要求 |
| --- | --- | --- | --- |
| **USB2CANFD_S**（单路） | `VID 0x34B7 / PID 0x6877` | 1 | V2.0.0 及以上，构建标识 ≥ `d64e840` |
| **USB2CANFD_D**（双路） | `VID 0x34B7 / PID 0x6632` | 2 | V2.0.0 及以上，构建标识 ≥ `b5c2574` |
| **LinkX4C**（四路） | `VID 0x34B7 / PID 0x4543` | 4 | V2.0.0 及以上，构建标识 ≥ `d5bc7db` |

支持平台：**Windows x64**（MinGW / MSVC 两套）、**Linux**（x86_64 / aarch64）、
**macOS**（Apple Silicon / Intel，最低 11.0）。

> 当前版本 **v2.1.0**（2026-10-11）。**它与 v1.1.0 的接口不兼容**——
> 老项目升级前先看 [`UPDATE.md`](UPDATE.md) 里的迁移对照。

---

## 1. 目录导航

| 我想… | 去哪 |
| --- | --- |
| **把 SDK 跑起来、查接口怎么用、写业务** | [`USAGE.md`](USAGE.md)（完整使用教程） |
| 拿库文件与头文件 | [`lib/v2.1.0/`](lib/v2.1.0/)——版本目录；其 [`README.md`](lib/v2.1.0/README.md) 是这一版包的文件清单与接入前提 |
| 找一个可直接编译的示例 | [`example/v2.1.0/`](example/v2.1.0/)（纯 C） |
| 看本版改了什么、从 v1.1.0 怎么升级 | [`UPDATE.md`](UPDATE.md) |
| 看历史版本 | [`lib/v1.1.0/`](lib/v1.1.0/)、[`lib/v1.0.0/`](lib/v1.0.0/)、[`example/v1.1.0/`](example/v1.1.0/) |

---

## 2. 30 秒上手

```c
#include <stdio.h>
#include "dmcan.h"          /* 头文件在 lib/v2.1.0/ 下，四个一起用 */

static void on_recv(dmcan_device_handle *dev, dmcan_rx_frame_t *f)
{
    (void)dev;
    printf("recv ch=%u id=0x%X len=%u\n", f->channel, f->can_id, f->dlen);
}

int main(void)
{
    dmcan_context *ctx = NULL;
    dmcan_context_create(&ctx);
    if (ctx == NULL || dmcan_find_devices(ctx) <= 0) return -1;   /* 找设备 */

    dmcan_device_handle *dev = NULL;
    if (!dmcan_device_get(ctx, &dev, 0)) return -1;               /* 取第 0 台 */
    if (!dmcan_device_open(dev)) return -1;                       /* 打开 */

    dmcan_device_hook_recv_callback(dev, on_recv);                /* 收帧回调 */

    dmcan_channel_can_info_t cfg = {0};                           /* 配通道：1M@5M CAN FD + BRS */
    cfg.canfd = true;   cfg.can_baudrate = 1000000;  cfg.can_sp = 0.75f;
    cfg.retrans = true; cfg.canfd_baudrate = 5000000; cfg.canfd_sp = 0.75f;
    dmcan_device_set_channel_baudrate(dev, 0, &cfg);
    dmcan_device_enable_channel(dev, 0);                          /* 顺序：先配参数，再使能 */

    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    dmcan_device_send_can(dev, 0, 0x123, true, false, false, true, 8, payload);

    /* ... 运行期间由回调收帧 ... */

    dmcan_device_close(dev);
    dmcan_context_destroy(ctx);
    return 0;
}
```

编译（库在 `lib/v2.1.0/<平台>/`，完整命令与运行期说明见 [`USAGE.md`](USAGE.md) §2）：

```bash
# Windows（MinGW）
gcc app.c lib/v2.1.0/windows/mingw/libdmcan.dll.a -o app.exe
# Linux
gcc app.c lib/v2.1.0/linux/x86_64/libdmcan.so -Wl,-rpath,$PWD/lib/v2.1.0/linux/x86_64 -o app
# macOS
clang app.c lib/v2.1.0/macos/arm64/libdmcan.dylib -Wl,-rpath,$PWD/lib/v2.1.0/macos/arm64 -o app
```

---

## 3. 接口地图（按需求找接口）

| 想做的事 | 接口（都在 `dmcan.h`） | 教程 |
| --- | --- | --- |
| 找设备 / 取句柄 / 打开 / 关闭 | `dmcan_find_devices`、`dmcan_device_get`、`dmcan_device_open`、`dmcan_device_close` | §4 |
| 读固件信息（型号 / 版本号 / 构建标识） | `dmcan_device_get_firmware_info` + `dmcan_firmware_info_to_string` | §4.2 |
| 配置通道（波特率、采样点、CAN FD / BRS、回环测试模式） | `dmcan_device_set_channel_baudrate`（`..._details` 给完整时序） | §5 |
| 开 / 关通道 | `dmcan_device_enable_channel`、`dmcan_device_disable_channel` | §5 |
| 发一帧 / 周期发 / 一次交一批 | `dmcan_device_send_can`、`..._send_can_details`、`..._send_can_batch`、`..._send_can_bulk`、`..._stop_send` | §7 |
| 收帧 | `dmcan_device_hook_recv_callback`（单回调）或 `dmcan_device_subscribe`（多订阅者） | §6 |
| 总线状态 / 通道统计 / 清 bus-off | `dmcan_device_get_bus_state`、`dmcan_device_get_channel_stats`、`dmcan_device_clear_bus_off` | §9 |
| 日志（级别、自己落盘） | `dmcan_set_log_level`、`dmcan_add_log_callback` | §10 |
| 热插拔通知 | `dmcan_context_register_hotplug` | §11 |
| 模块自带的 USB2UART 串口透传 | `dmcan_device_uart_open`、`..._uart_write`、`..._uart_get_port_name` | §12 |
| 取错误原因 / 校验库与头配套 | `dmcan_get_last_error`、`dmcan_check_version` | §13 |

---

## 4. 六条最常踩的约定

1. **先配参数、再使能**；设备复位或重新插拔后要 `close()` → `open()`，并**重新使能通道**（复位后通道默认关闭）；
2. **长度自动向上取整到最近的 DLC**（请求 9 字节 → 线上 12 字节，不足补 0），不用自己换算；
   接收侧读到的是**线上长度**；
3. 收帧回调跑在 SDK 的**通知线程**上、同一设备串行：里面别做重活、别阻塞，**也不要在回调里 `close()`**；
4. 返回 `false` 时用 `dmcan_get_last_error()` 取原因（参数错 / 设备没开 / 与设备交互失败…）；
5. 用 `dmcan_check_version(DMCAN_VERSION_INT)` 校验"手里的库"和"编译用的头"是否配套；
6. **软件终端电阻开关、通道 echo 开关、UART 线参数是会话级设置**，重新枚举后要重新下发。

---

## 5. 参考

- 接口的权威说明在 [`lib/v2.1.0/dmcan.h`](lib/v2.1.0/dmcan.h) 与 `dmcan_types.h`：
  每个接口都写了调用时机、失败语义、返回值含义与回调线程约定；
- 固件升级、恢复出厂、读写 SN 等**设备管理类**接口不在本套公开 SDK 里（内部工具另有交付）。
