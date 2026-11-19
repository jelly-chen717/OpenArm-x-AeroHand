# DM_Device_SDK C / C++ 使用教程（v2.1.0）

面向"把 达妙 CAN(FD) 模块接进自己的程序"的使用者。**接口的权威说明在
[`lib/v2.1.0/dmcan.h`](lib/v2.1.0/dmcan.h) 与 `dmcan_types.h`**（每个接口都写了调用时机、
失败语义、返回值含义与回调线程约定）；本教程讲怎么把它们串起来用。

> 对应 **v2.1.0**（库与头文件在 [`lib/v2.1.0/`](lib/v2.1.0/)）。从 v1.1.0 升级见
> [`UPDATE.md`](UPDATE.md)（**接口不兼容**）。只想先跑起来：看完 §1 ~ §3 就够。

## 目录

1. [前置条件](#1-前置条件)
2. [集成：怎么链接、运行期要带什么](#2-集成怎么链接运行期要带什么)
3. [起步：完整最小程序](#3-起步完整最小程序)
4. [设备发现、设备信息与多设备](#4-设备发现设备信息与多设备)
5. [通道配置](#5-通道配置)
6. [收帧](#6-收帧)
7. [发送](#7-发送)
8. [Echo 与发送统计](#8-echo-与发送统计)
9. [诊断接口](#9-诊断接口)
10. [日志](#10-日志)
11. [热插拔通知](#11-热插拔通知)
12. [USB2UART 串口透传](#12-usb2uart-串口透传)
13. [错误处理与版本配套](#13-错误处理与版本配套)
14. [使用注意汇总](#14-使用注意汇总)
15. [常见问题](#15-常见问题)
16. [参考](#16-参考)

---

## 1. 前置条件

| 平台 | 需要准备 | 说明 |
| --- | --- | --- |
| Windows x64（MinGW） | 无 | `lib/v2.1.0/windows/mingw/` 已带 libusb 与 GCC 运行库 |
| Windows x64（MSVC） | **VC++ 2015-2022 可再发行组件** | 包内只带 `libusb-1.0.dll` |
| Linux x86_64 / aarch64 | `libusb-1.0`；**设备权限** | 普通用户默认打不开 USB 设备节点，加 udev 规则或用 root（见 §2.3） |
| macOS（Apple Silicon / Intel） | 无（最低 macOS 11.0） | `libusb` 随包，解压即用 |

头文件：`lib/v2.1.0/dmcan.h`、`dmcan_types.h`、`dmcan_version.h`、`dmcan_export.h`（四个一起用）。
库文件：`lib/v2.1.0/<平台>/`（见本仓库 [`README.md`](README.md) §1 的目录导航）。

---

## 2. 集成：怎么链接、运行期要带什么

### 2.1 CMake

```cmake
# 头文件目录直接用 lib/v2.1.0
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/C&C++/lib/v2.1.0)

if(WIN32 AND MINGW)
    target_link_libraries(app PRIVATE ${CMAKE_SOURCE_DIR}/C&C++/lib/v2.1.0/windows/mingw/libdmcan.dll.a)
elseif(WIN32)
    target_link_libraries(app PRIVATE ${CMAKE_SOURCE_DIR}/C&C++/lib/v2.1.0/windows/msvc/dmcan.lib)
elseif(APPLE)
    target_link_libraries(app PRIVATE ${CMAKE_SOURCE_DIR}/C&C++/lib/v2.1.0/macos/arm64/libdmcan.dylib)   # Intel 换成 macos/x86_64
else()
    target_link_libraries(app PRIVATE ${CMAKE_SOURCE_DIR}/C&C++/lib/v2.1.0/linux/x86_64/libdmcan.so)      # arm64 换成 linux/arm64
endif()
```

### 2.2 手工链接

```bash
# Windows（MinGW）
gcc app.c lib/v2.1.0/windows/mingw/libdmcan.dll.a -o app.exe
# Windows（MSVC）
cl app.c /I lib/v2.1.0 /link lib/v2.1.0/windows/msvc/dmcan.lib /out:app.exe
# Linux
gcc app.c lib/v2.1.0/linux/x86_64/libdmcan.so \
    -Wl,-rpath,$PWD/lib/v2.1.0/linux/x86_64 -o app
# macOS
clang app.c lib/v2.1.0/macos/arm64/libdmcan.dylib \
    -Wl,-rpath,$PWD/lib/v2.1.0/macos/arm64 -o app
```

### 2.3 运行期

- **Windows**：把 `lib/v2.1.0/windows/<工具链>/` 加进 `PATH`，或把其中的 DLL 拷到 exe 旁边
  （MinGW 版要带上 4 个：`libusb-1.0.dll` + 3 个 GCC 运行库）；
- **macOS**：`libdmcan.dylib` 与 `libusb-1.0.0.dylib` 要**在一起**（前者按 `@loader_path` 找后者）。
  用 `-Wl,-rpath` 指到该目录，或把两个 dylib 拷到可执行文件旁；
- **Linux**：目标机装 `libusb-1.0`；权限二选一——
  - 加 udev 规则（推荐，一次配好）：

    ```
    # /etc/udev/rules.d/99-damiao-usb.rules
    SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666"
    ```
    然后 `sudo udevadm control --reload-rules && sudo udevadm trigger`；
  - 或者直接用 root 运行。
  用 USB2UART 串口时，把用户加进 `dialout` 组。

---

## 3. 起步：完整最小程序

```c
#include <stdio.h>
#include <unistd.h>            /* Windows 用 <windows.h> 的 Sleep() */
#include "dmcan.h"

static void on_recv(dmcan_device_handle *dev, dmcan_rx_frame_t *f)
{
    (void)dev;   /* 回调里不要做重活、不要阻塞 */
    printf("recv ch=%u id=0x%X len=%u ts=%llu\n", f->channel, f->can_id, f->dlen,
        (unsigned long long)f->timestamp);
}

int main(void)
{
    /* 1) 上下文 + 找设备 */
    dmcan_context *ctx = NULL;
    dmcan_context_create(&ctx);
    if (ctx == NULL) return -1;
    const int n = dmcan_find_devices(ctx);
    if (n <= 0) { printf("没找到设备\n"); dmcan_context_destroy(ctx); return -1; }

    /* 2) 取句柄 + 打开 + 读固件信息 */
    dmcan_device_handle *dev = NULL;
    if (!dmcan_device_get(ctx, &dev, 0) || !dmcan_device_open(dev)) return -1;

    dmcan_firmware_info_t fw;
    char line[192];
    if (dmcan_device_get_firmware_info(dev, &fw))
    {
        dmcan_firmware_info_to_string(&fw, line, sizeof(line));
        printf("固件: %s\n", line);        /* 形如 USB2CANFD_D V2.0.0 [b5c2574 ...] */
    }

    /* 3) 配通道（先配参数）→ 使能 */
    dmcan_channel_can_info_t cfg = {0};
    cfg.canfd = true;   cfg.can_baudrate = 1000000;  cfg.can_sp = 0.75f;
    cfg.retrans = true; cfg.canfd_baudrate = 5000000; cfg.canfd_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(dev, 0, &cfg))
        printf("配置通道失败 err=%d\n", (int)dmcan_get_last_error());
    dmcan_device_enable_channel(dev, 0);

    /* 4) 收帧回调 + 发一帧 */
    dmcan_device_hook_recv_callback(dev, on_recv);
    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    dmcan_device_send_can(dev, 0 /*通道*/, 0x123 /*ID*/, true /*CANFD*/, false /*扩展帧*/,
        false /*远程帧*/, true /*BRS*/, 8 /*长度*/, payload);

    sleep(2);   /* Windows: Sleep(2000) */

    /* 5) 收尾 */
    dmcan_device_close(dev);
    dmcan_context_destroy(ctx);
    return 0;
}
```

可直接编译的版本见 [`example/v2.1.0/`](example/v2.1.0/)。

---

## 4. 设备发现、设备信息与多设备

| 接口 | 用途 |
| --- | --- |
| `dmcan_find_devices(ctx)` | 枚举设备，返回**句柄表里的设备数** |
| `dmcan_find_devices_with_type(ctx, type)` | 只要某一型号（`DMCAN_DEVICE_USB2CANFD_S` / `_D` / `LINKX4C`） |
| `dmcan_show_all_devices(ctx)` | 打印设备清单（调试用） |
| `dmcan_device_get(ctx, &dev, index)` | 按**下标**取句柄（下标含义见下） |
| `dmcan_device_get_info(dev, &type, &pid, &vid, &sn)` | 型号 / PID / VID / 序列号 |
| `dmcan_device_is_present(dev, &present)` | 这台设备当前在不在（热插拔后） |
| `dmcan_device_open(dev)` / `dmcan_device_close(dev)` | 打开 / 关闭 |

**多设备**：别写死下标 0。用 `dmcan_find_devices()` 的数量遍历，并用
`dmcan_device_get_info()` 的**序列号/型号**认设备：

```c
const int n = dmcan_find_devices(ctx);
for (int i = 0; i < n; i++)
{
    dmcan_device_handle *dev = NULL;
    dmcan_device_type_t type; int pid = 0, vid = 0; unsigned char *sn = NULL;
    if (!dmcan_device_get(ctx, &dev, i)) continue;
    dmcan_device_get_info(dev, &type, &pid, &vid, &sn);
    printf("[%d] type=%d pid=0x%04X vid=0x%04X sn=%s\n", i, (int)type, pid, vid, sn ? (const char *)sn : "");
}
```

> 已经拔掉的设备**会留在句柄表里**（用 `dmcan_device_is_present()` 判断），
> 所以下标在插拔前后是**稳定**的：插回来的还是同一个句柄，不需要重新 `dmcan_device_get()`。

### 4.2 读固件信息（型号 / 版本号 / 构建标识）

```c
dmcan_firmware_info_t fw;
dmcan_device_get_firmware_info(dev, &fw);
/* fw.device（型号）、fw.version + fw.version_valid、fw.build_id + fw.build_id_valid */
char line[192];
dmcan_firmware_info_to_string(&fw, line, sizeof(line));
```

- 判"**板子还停在 boot**"：`fw.version_valid == false`（boot 也能回构建标识，所以**不要**用接口返回值判）；
- 判"**固件升级是否生效**"：看 `fw.build_id`（发布版本号在连续几版之间常常不变）；
- 本 SDK 要求固件带两处与上位机接口相关的修正，构建标识要求见 [`lib/v2.1.0/README.md`](lib/v2.1.0/README.md) §1。

---

## 5. 通道配置

接口分两档：**简略**（波特率 + 采样点，覆盖绝大多数场景）与**详细**（完整位定时 + TDC + 节点模式）。

| 接口 | 用途 |
| --- | --- |
| `dmcan_device_set_channel_baudrate(dev, ch, &info)` | 简略：仲裁/数据段波特率 + 采样点 + CAN FD / BRS / 重传 |
| `dmcan_device_get_channel_baudrate(dev, ch, &info)` | 读回简略配置 |
| `dmcan_device_set_channel_baudrate_details(dev, ch, &conf)` | 详细：`dmcan_ch_baud_conf_t`（含 BRP/SEG/SJW、TDC、节点模式、bus-off 自动恢复） |
| `dmcan_device_get_channel_baudrate_details(dev, ch, &conf)` | 读回详细配置（下发值逐字节回显） |
| `dmcan_device_get_channel_tdc(dev, ch, &status)` | 读**硬件实际生效**的 TDC 值（与"下发值"不同） |
| `dmcan_device_enable_channel(dev, ch)` / `dmcan_device_disable_channel(dev, ch)` | 开 / 关通道 |

简略配置的字段（`dmcan_channel_can_info_t`）：`canfd`、`can_baudrate`、`canfd_baudrate`、
`can_sp`（仲裁域采样点，如 0.75）、`canfd_sp`（数据域采样点）、`retrans`（是否自动重传）。

```c
dmcan_channel_can_info_t cfg = {0};
cfg.canfd = true;   cfg.can_baudrate = 1000000;  cfg.can_sp = 0.75f;
cfg.retrans = true; cfg.canfd_baudrate = 5000000; cfg.canfd_sp = 0.75f;
dmcan_device_set_channel_baudrate(dev, 0, &cfg);
dmcan_device_enable_channel(dev, 0);
```

### 5.1 经典 CAN 与"数据段字段"

固件是按**数据段**的取值范围校验那几个字段的，所以**经典 CAN（`canfd = false`）时数据段四个字段也不能留 0**。
简略接口已经替你处理（会按设备当前配置补齐）；只有直接用详细接口时才需要注意填非 0 值。

### 5.2 测试模式（回环 / 只听）

`dmcan_ch_baud_conf_t::mode`（枚举 `dmcan_channel_mode_t`）：

| 值 | 模式 | 说明 |
| --- | --- | --- |
| 0 | `DMCAN_CHANNEL_MODE_NORMAL` | 正常收发（默认） |
| 1 | `DMCAN_CHANNEL_MODE_LOOPBACK_INTERNAL` | 内部回环：不外发，帧在芯片内回到接收路径 |
| 2 | `DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL` | 外部回环：帧照常驱动总线并回环到自己，**不接对端也能自发自收**（自检/性能基线用） |
| 3 | `DMCAN_CHANNEL_MODE_LISTEN_ONLY` | 只听不发（不驱动总线、不回 ACK） |

### 5.3 生效时机

配置下发后**硬件是异步重载**的：实测下发到生效约 **0.1 ~ 1 s**（固件按秒级任务重载）。
所以**下发后立刻回读**可能还是旧值——这不是失败；等一拍再读，或直接开始收发验证。

---

## 6. 收帧

两条路，按需要选：

| 方式 | 接口 | 适用 |
| --- | --- | --- |
| 单回调（简单） | `dmcan_device_hook_recv_callback(dev, cb)` | 一个程序里只想要一路收帧 |
| 多订阅者 | `dmcan_device_subscribe(dev, &cfg, &sub)` / `dmcan_device_unsubscribe(dev, sub)` | 想按"事件类型 + 通道"分给多个消费者（最多 4 个订阅） |

### 6.1 单回调

```c
static void on_recv(dmcan_device_handle *dev, dmcan_rx_frame_t *f) { /* ... */ }
dmcan_device_hook_recv_callback(dev, on_recv);      /* 传 NULL 可注销 */
```

另有 `dmcan_device_hook_sent_callback()`（发送完成）、`dmcan_device_hook_err_callback()`（通道错误记录）。

### 6.2 多订阅者

```c
static void on_event(dmcan_device_handle *dev, const dmcan_event_t *ev, void *user_data)
{
    (void)dev; (void)user_data;
    switch (ev->type)
    {
    case DMCAN_EVENT_RX_FRAME:      /* ev->frame：总线收到的帧 */            break;
    case DMCAN_EVENT_TX_COMPLETED:  /* ev->frame：这笔 USB 提交已完成（假 echo） */ break;
    case DMCAN_EVENT_TX_ON_BUS:     /* ev->frame：确实上了总线（真 echo） */   break;
    case DMCAN_EVENT_CHANNEL_ERROR: /* ev->error：通道错误记录 */            break;
    case DMCAN_EVENT_UART_RX:       /* ev->uart：串口收到一段字节（见 §12） */  break;
    default: break;
    }
}

dmcan_subscribe_config_t cfg = {0};
cfg.events = DMCAN_EVENT_RX_FRAME | DMCAN_EVENT_TX_COMPLETED;   /* 事件位图 */
cfg.channel_mask = DMCAN_CHANNEL_MASK_ALL;                      /* 关心的通道位图（0 = 什么都没选，参数错） */
cfg.callback = on_event;
cfg.user_data = my_context;

dmcan_subscription_t *sub = NULL;
dmcan_device_subscribe(dev, &cfg, &sub);
/* ... 不再需要时 ... */
dmcan_device_unsubscribe(dev, sub);
```

**回调约定**（与单回调相同）：

- 跑在 SDK 的**通知线程**上（每设备一条），**同一设备串行**；不同设备是不同线程；
- 里面别做重活（落盘、网络、长时间加锁），要留数据就自己拷贝——`ev->frame` / `ev->error` / `ev->uart`
  指向内部缓冲，**只在本次回调期间有效**；
- 不要在回调里 `close()`（SDK 会拒绝并置 `DMCAN_ERR_REENTRANT`），要关设备请回自己的线程；
- 订阅可以跨 `close()` / 重连保持。

### 6.3 帧结构（`dmcan_rx_frame_t`）

| 字段 | 含义 |
| --- | --- |
| `can_id` | 帧 ID |
| `channel` | 通道号 |
| `dlen` | 数据长度（**线上长度**，已按 DLC 取整） |
| `payload[64]` | 数据 |
| `ext` / `rtr` / `canfd` / `brs` / `esi` / `ack` | 帧类型与状态位 |
| `timestamp` | 单位 µs。**注意两种时基**：接收帧 / 真 echo / 错误记录是**设备时间戳**，假 echo 是**上位机系统时间**（UTC epoch）——两者只能各自排序，不能相减 |

---

## 7. 发送

| 接口 | 用途 |
| --- | --- |
| `dmcan_device_send_can(dev, ch, id, canfd, ext, rtr, brs, dlen, payload)` | 发一帧 |
| `dmcan_device_send_can_details(dev, ch, interval_ms, send_times, id, canfd, ext, rtr, brs, dlen, payload)` | 周期发送（`send_times = 0` 表示一直发；间隔单位 ms） |
| `dmcan_device_stop_send(dev, ch)` | 停止该通道的周期发送（调度在 SDK 侧） |
| `dmcan_device_send_can_batch(dev, ch, frames, count)` | **一次交一批**（数组版，替代 v1.1.0 的两段式 fill/queue） |
| `dmcan_device_send_can_bulk(dev, ch, &config)` | 超大批量（配置结构体见头文件） |
| `dmcan_device_set_send_profile(dev, profile)` | 发送聚合档位：吞吐优先（默认）/ 延迟优先 |

```c
/* 单次发一帧 */
uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
dmcan_device_send_can(dev, 0, 0x123, true, false, false, true, 8, data);

/* 每 10 ms 发一帧、发 100 次（send_times = 0 表示一直发到 stop_send） */
dmcan_device_send_can_details(dev, 0, 10, 100, 0x200, true, false, false, true, 8, data);
dmcan_device_stop_send(dev, 0);

/* 一次交一批 */
dmcan_frame_t batch[3] = {0};
batch[0].can_id = 0x300; batch[0].canfd = true; batch[0].brs = true; batch[0].dlen = 4;
batch[1].can_id = 0x301; batch[1].canfd = true; batch[1].brs = true; batch[1].dlen = 4;
batch[2].can_id = 0x302; batch[2].canfd = true; batch[2].brs = true; batch[2].dlen = 4;
dmcan_device_send_can_batch(dev, 0, batch, 3);
```

**三条要点**：

1. **长度自动取整**：`dlen` 会向上取整到最近的 DLC（请求 9 → 线上 12，不足补 0），
   不用自己换算；只有"装不下"的组合（经典帧 `dlen > 8`、`dlen > 64`）才拒收；
2. 返回 `true` 只表示**已受理（入队）**，不代表已经上了总线；"确实上总线"用真 echo 事件确认（§8）；
3. 调用方不用自己攒批量——发送引擎会按当前档位把队列里的帧合并成少量 USB 提交；
   批量接口的好处是**一次调用交完、没有跨调用的残留状态**。

---

## 8. Echo 与发送统计

| 事件 / 接口 | 含义 |
| --- | --- |
| `DMCAN_EVENT_TX_COMPLETED`（假 echo） | SDK 侧"这笔 USB 提交已完成"，由上位机产生；**不代表总线确认** |
| `DMCAN_EVENT_TX_ON_BUS`（真 echo） | 设备回报"这帧确实在总线上被确认"；payload 由 SDK 用发送时的原始数据补齐 |
| `dmcan_device_set_channel_echo(dev, ch, enable)` | 开关该通道的真 echo 上报（会话级设置） |
| `dmcan_device_get_echo_stats(dev, &stats)` | 真 echo 统计（超时条数、迟到丢弃、在途、当前超时阈值） |
| `dmcan_device_set_echo_timeout_ms(dev, ms)` | 真 echo 超时阈值（默认 1000 ms，内部 clamp） |

---

## 9. 诊断接口

| 接口 | 用途 |
| --- | --- |
| `dmcan_device_get_bus_state(dev, ch, &st)` | 按需读一次总线状态：`bus_state`（0 active / 1 warning / 2 error passive / 3 bus-off）、`tec`、`rec`、`credits`、`in_flight` |
| `dmcan_device_get_channel_stats(dev, ch, &cs)` | 通道统计快照：收发计数、丢弃计数（含细分）、`enabled`、队列深度 |
| `dmcan_device_clear_bus_off(dev, ch)` | 清除 bus-off 并重建发送账目 |
| `dmcan_device_get_module_features(dev, &features)` | 模块能力位图（**表示"能做什么"，不是当前状态**；例如软件终端电阻开关） |
| `dmcan_device_enable_can_res(dev, ch, state)` / `dmcan_device_get_can_res(dev, ch, &enabled)` | 软件终端电阻（需能力位支持，且通道已使能；**重置后要重新下发**） |
| `dmcan_device_set_channel_filter(dev, ch, mode, rules, count, &usage)` / `dmcan_device_get_channel_filter(dev, ch, &table)` | 硬件过滤表（要求通道未使能） |

> `dmcan_device_get_channel_stats()` 读的是**周期快照**（设备每秒推一条）：**刚打开或刚发完立刻读可能还没有
> 数据**（返回 false），等一拍再读即可；判"有没有在丢包"要**连着读两次看增量**，单次绝对值可能是很久以前的累计。

---

## 10. 日志

- **级别**：`dmcan_set_log_level(level)` / `dmcan_get_log_level()`，`0 = trace` … `5 = critical`，
  越大越安静；默认 `info`。Release 构建默认把 `trace/debug` 编译裁剪掉；
- 也可以用环境变量 `DMCAN_LOG_LEVEL`（`trace/debug/info/warn/error/critical/off`）控制终端输出；
- **要把日志落盘**：注册回调自己写文件（SDK 不替你落盘）：

```c
static void on_log(int level, unsigned long long seq, const char *line, void *user_data)
{
    (void)level; (void)user_data;
    FILE *f = (FILE *)user_data;
    fprintf(f, "%llu %s\n", seq, line);        /* 自己攒队列、由自己的线程写更稳妥 */
}

dmcan_log_subscription_t *log_sub = NULL;
dmcan_add_log_callback(on_log, /*min_level=*/2, my_file, &log_sub);   /* 该回调有自己的阈值 */
/* 不再需要：dmcan_remove_log_callback(log_sub); */
```

- 约定：回调**同步跑在产生日志的那条线程上**，别在里面做重活、别再调 SDK 的日志接口；
  终端级别与回调级别是**分开**的（终端安静不代表回调收不到）。

---

## 11. 热插拔通知

```c
static void on_hotplug(dmcan_context *ctx, dmcan_hotplug_event_t event,
                       const dmcan_hotplug_device_t *device, void *user_data)
{
    (void)ctx; (void)user_data;
    printf("%s type=%d pid=0x%04X\n",
        event == DMCAN_HOTPLUG_ATTACHED ? "接入" : "拔出", (int)device->type, device->product_id);
}

dmcan_context_register_hotplug(ctx, on_hotplug, NULL);
/* 退出前：dmcan_context_unregister_hotplug(ctx); */
```

- `dmcan_context_hotplug_supported(ctx)` 可先探测当前后端支不支持；
- 回调里**不要**注销监听或销毁 context（SDK 会拒绝并置 `DMCAN_ERR_REENTRANT`）；
- 热插拔事件**不给句柄**：要操作设备请自己走 `dmcan_find_devices()` + `dmcan_device_get()`——
  这条路径不会让已有句柄失效。

---

## 12. USB2UART 串口透传

模块除了 CANFD 的厂商接口，还有一路 **CDC 虚拟串口**（Windows 上是 COM 口，Linux 是 `/dev/ttyACM*`，
macOS 是 `/dev/cu.usbmodem*`），SDK 可以替你打开它并收发字节（**LinkX4C 没有这一路**）。

| 接口 | 用途 |
| --- | --- |
| `dmcan_device_set_uart_auto_open(dev, enable)` | 开关"打开设备时自动打开串口"（**默认开**） |
| `dmcan_device_set_uart_config(dev, &cfg)` | 设线参数（波特率/数据位/校验/停止位），自动打开时用这套参数 |
| `dmcan_device_uart_open(dev, &cfg)` / `dmcan_device_uart_close(dev)` | 显式打开 / 关闭（`cfg` 传 `NULL` 用已存参数） |
| `dmcan_device_uart_is_open(dev, &is_open)` / `dmcan_device_uart_get_port_name(dev, buf, size)` | 查状态 / 取端口名 |
| `dmcan_device_uart_write(dev, data, len)` | 写字节 |
| `dmcan_device_get_uart_stats(dev, &stats)` | 收发字节数与丢弃计数 |

- **收**数据不用额外接口：订阅 `DMCAN_EVENT_UART_RX` 即可（§6.2），事件里 `ev->uart->data/length`
  是**这次操作系统给到的一段连续字节**（边界不是协议帧，要留存自己拷贝）；
- 端口按**本机设备的序列号**匹配自动打开；**打不开不影响 CANFD**（只告警）；
- 线参数是**会话级**设置，重新枚举后要重新下发；固件对超范围波特率是"拒绝并保留上一次配置"。

---

## 13. 错误处理与版本配套

- 返回 `false` / 空结果的接口，用 `dmcan_get_last_error()` 取原因（**线程局部**，多线程各自独立）：

  | 码 | 含义 |
  | --- | --- |
  | `DMCAN_OK` | 上一次调用成功 |
  | `DMCAN_ERR_INVALID_ARG` | 入参非法（空指针、通道号越界等） |
  | `DMCAN_ERR_NOT_OPEN` | 设备尚未打开（或已关闭） |
  | `DMCAN_ERR_DEVICE_NOT_FOUND` | 设备下标越界 / 当前没有设备 |
  | `DMCAN_ERR_NO_MEMORY` | 内存不足（例如订阅槽位满） |
  | `DMCAN_ERR_IO` | 与设备交互失败（USB 超时 / 固件拒绝等） |
  | `DMCAN_ERR_UNSUPPORTED` | 设备或固件不支持该能力 |
  | `DMCAN_ERR_INTERNAL` | 内部状态异常（不该发生） |
  | `DMCAN_ERR_REENTRANT` | 在 SDK 自己的回调线程上调用了会自锁的接口（调用被拒绝） |

  取完可用 `dmcan_clear_last_error()` 清掉。
- **库与头是否配套**：`dmcan_check_version(DMCAN_VERSION_INT)`（头文件里定义的版本号，与本版库一致）；
- SDK 自身版本：`dmcan_get_sdk_version(ctx, &v)`，或 `dmcan_print_version(ctx)` 直接打印。

---

## 14. 使用注意汇总

- 设备**复位 / 重新插拔后**：`dmcan_device_close()` → `dmcan_device_open()`，并**重新使能通道**；
- **会话级设置不落盘**：软件终端电阻、通道 echo 开关、UART 线参数，重新枚举后要重新下发；
- **订阅**（`dmcan_device_subscribe`）跨 close / 重连保持；
- 回调线程上：不阻塞、不做重活、不 `close`、不销毁 context；
- 多设备：用序列号/型号认设备，别写死下标；
- 长度按 DLC 自动取整，接收侧读到的是线上长度；
- 详细行为约定（失败语义、返回值、边界条件）以 `dmcan.h` / `dmcan_types.h` 的注释为准。

---

## 15. 常见问题

**Q1 打开设备失败（`dmcan_device_open` 返回 false）？**
看 `dmcan_get_last_error()`：`DMCAN_ERR_IO` 多为权限/占用问题。
Linux 是设备节点权限（加 udev 规则或用 root，见 §2.3）；Windows 上同一设备被别的进程独占时也会失败。

**Q2 发了帧但对端收不到？**
① 两端波特率/采样点是否一致；② 通道是否 `enable`；③ 总线两端终端电阻（120Ω）；
④ 是不是经典 CAN 却按 FD 发（或反之）；⑤ 用外部回环模式（§5.2）先确认自己能收自己的帧。

**Q3 收帧会丢？**
先看 `dmcan_device_get_channel_stats()` 的丢弃细分，区分"设备侧队列满"还是"主机读得不够快"；
回调里 `printf`/落盘这类重活会让 USB 读跟不上，改成攒内存队列、由自己的线程处理。

**Q4 DLC 和长度是什么关系？**
不用手工换算：下发时 `dlen` 会自动向上取整到最近的 DLC（不足补 0），
`dmcan_utils_get_dlc_from_len()` / `dmcan_utils_get_len_from_dlc()` 只是给需要精确对照的场景用。

**Q5 读版本/构建标识读不到？**
`fw.version_valid == false` 通常表示**设备跑在 boot**（不是坏了）；老固件可能没有构建标识
（`fw.build_id_valid == false`）。判"升级是否生效"看 `build_id`（§4.2）。

**Q6 多个线程能不能同时用同一个句柄？**
收帧回调固定在**该设备的通知线程**上、同一设备串行；稳妥做法是**每台设备由一个线程负责收发**、
回调里只做搬运。不要在回调里调用会阻塞的接口（SDK 会拒绝并给 `DMCAN_ERR_REENTRANT`）。

**Q7 设备拔掉再插回来，原来的句柄还能用吗？**
句柄仍在句柄表里且**不会失效**：先 `dmcan_device_is_present()` 确认回来，
再 `close()` → `open()` → 重新配置使能即可。也可以用热插拔通知（§11）驱动这套流程。

---

## 16. 参考

- [`lib/v2.1.0/README.md`](lib/v2.1.0/README.md)——本版包的文件清单、接入前提、各平台运行前提；
- [`UPDATE.md`](UPDATE.md)——版本变更与 v1.1.0 → v2.1.0 迁移对照；
- [`example/v2.1.0/`](example/v2.1.0/)——可直接编译的 C 示例；
- [`lib/v2.1.0/dmcan.h`](lib/v2.1.0/dmcan.h)、`dmcan_types.h`——接口与类型的权威说明。
