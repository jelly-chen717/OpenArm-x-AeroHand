# C / C++ SDK 更新说明

> 本文件记录**对外发布**的版本变更与升级要点。库与头文件在 [`lib/`](lib/) 下按版本分目录。
> 当前最新：**v2.1.0**（2026-10-11）

## 1. 版本一览

| 版本 | 目录 | 状态 | 一句话 |
| --- | --- | --- | --- |
| **v2.1.0** | [`lib/v2.1.0/`](lib/v2.1.0/) | **最新（推荐）** | 2.x 首个对外版本：接口全新（**与 1.x 不兼容**）、六平台交付、macOS 解压即用 |
| v1.1.0 | [`lib/v1.1.0/`](lib/v1.1.0/) | 仅维护 | 老接口（`dmcan.h`：`usb_rx_frame_t` + `head` 位域、`hook_recv_callback` 那一套） |
| v1.0.0 | [`lib/v1.0.0/`](lib/v1.0.0/) | 归档 | 更早的 `pub_user.h` 接口 |

> **v2.0.x 没有对外发布**（只有内部测试版本）。
> 从 1.x 升级请直接 **v1.1.0 → v2.1.0**，不用去找 v2.0.0 / v2.0.1。

---

## 2. v2.1.0（2026-10-11）

### 2.1 不兼容变更：设备版本信息合并成一个接口

老版本里"读设备版本 / 构建信息"是一堆接口，命名与返回方式各不相同，而且判"固件升级是否生效"
需要的构建标识不一定读得到。现在合并成两个：

```c
dmcan_firmware_info_t fw;
dmcan_device_get_firmware_info(dev, &fw);        /* 一条 CMD_GET_VERSION 拿全：型号 + 版本号 + 构建标识 */
char line[192];
dmcan_firmware_info_to_string(&fw, line, sizeof(line));   /* 形如 "USB2CANFD_D V2.0.0 [b5c2574 ...]" */
```

| 旧（v1.1.0 及更早） | 新（v2.1.0） |
| --- | --- |
| `dmcan_device_get_version(dev, &v)` | `dmcan_device_get_firmware_info(dev, &fw)`（看 `fw.version_valid` / `fw.version`） |
| `dmcan_device_get_version_string(dev, buf, size)` | 同上去掉：型号在 `fw.device`（老实现返回的是 boot 固件名） |
| `dmcan_device_get_build_info_string(dev, buf, size)` | 同上去掉：构建标识在 `fw.build_id`（看 `fw.build_id_valid`） |
| `dmcan_device_print_version(dev)` | `dmcan_firmware_info_to_string()` 之后自己打印 |

> 判"板子还在 boot"用 **`fw.version_valid == false`**（不要用接口返回值：boot 也可能带构建标识）。
> 判"固件升级是否生效"用 **`fw.build_id`**：发布版本号在连续几版之间常常不变。

### 2.2 其它常用接口的写法变化（v1.1.0 → v2.1.0）

| 想做的事 | v1.1.0 | v2.1.0 |
| --- | --- | --- |
| 波特率配置 | `dmcan_device_set_channel_baudrate(dev, ch, info)`（按值传结构体） | `dmcan_device_set_channel_baudrate(dev, ch, &info)`（**传指针**；更细的时序/滤波用 `..._details`） |
| 收帧 | `dmcan_device_hook_recv_callback(dev, cb)` | 保留（回调签名改为 `cb(dmcan_device_handle*, dmcan_rx_frame_t*)`）；多订阅者用 `dmcan_device_subscribe` |
| 帧结构 | `usb_rx_frame_t`（字段藏在 `head` 位域里） | `dmcan_rx_frame_t`（`can_id` / `channel` / `dlen` / `payload` / `timestamp` 直接字段） |
| 长度换算 | 自己调 `dmcan_utils_get_dlc_from_len()` | **不用手工换算**：`dmcan_device_send_can*()` 的 `dlen` 会自动向上取整到最近的 DLC |
| 队列发送（两段式） | `dmcan_device_fill_can_queue()` + `dmcan_device_can_queue_send()` | 数组版一次交进去：`dmcan_device_send_can_batch()`（大批量用 `dmcan_device_send_can_bulk()`） |
| 停止周期发送 | — | `dmcan_device_stop_send()` |

### 2.3 相对 v1.1.0 的新增能力

- **多订阅者收帧**：`dmcan_device_subscribe()` 最多 4 个订阅，按事件位图 + 通道掩码过滤，收帧零拷贝；
- **批量 / 递增发送**：ID 递增、数据递增、区间发送（`dmcan_device_send_can_details()`）；
- **诊断接口**：读总线状态（tec/rec/credits/in_flight）、清 bus-off、读通道统计（含丢包细分）；
- **日志系统**：`dmcan_set_log_level()` + 日志回调（可由使用方自己落盘），用户可见字符串全部英文；
- **热插拔通知**：`dmcan_context_register_hotplug()`；设备句柄在重新枚举后依然有效；
- **真 / 假 echo**：发送完成与"确实上了总线"两种事件；
- **通道测试模式**：内部回环 / **外部回环** / 只听（`dmcan_ch_baud_conf_t::mode`，不接线即可自发自收做自检）；
- **USB2UART 串口透传**：按序列号自动匹配打开（LinkX4C 没有这个虚拟串口）；
- **新增 Linux / macOS 支持**：x86_64 / arm64 / Apple Silicon / Intel 六套库。

### 2.4 设备 / 固件要求

| 模块 | USB 标识 | 固件要求 |
| --- | --- | --- |
| USB2CANFD_S（单路） | `0x34B7 / 0x6877` | V2.0.0 及以上，构建标识 ≥ `d64e840` |
| USB2CANFD_D（双路） | `0x34B7 / 0x6632` | V2.0.0 及以上，构建标识 ≥ `b5c2574` |
| LinkX4C（四路） | `0x34B7 / 0x4543` | V2.0.0 及以上，构建标识 ≥ `d5bc7db` |

- 这三个构建标识是**与上位机接口兼容的最低固件**。固件版本号一直是 `V2.0.0`，
  但接口相关的修正没有改版本号，所以只能按**构建标识**判断；
- 不确定手头设备是否达标时：用 `dmcan_device_get_firmware_info()` 读回构建标识找达妙确认，
  或直接升级到最新固件。

---

## 3. 怎么用 v2.1.0

- 头文件：`lib/v2.1.0/dmcan.h`（配套还有 `dmcan_types.h` / `dmcan_version.h` / `dmcan_export.h`，四个一起用）；
- 库：按平台在 `lib/v2.1.0/<平台>/` 下取，Windows / macOS 的运行期依赖也放在同一目录；
- **接入步骤、最小示例、各平台运行前提、使用注意**：见 [`lib/v2.1.0/README.md`](lib/v2.1.0/README.md)；
- 可编译示例工程：[`example/v2.1.0/`](example/v2.1.0/)。

---

## 4. 文档导航与已知事项

- 本目录三份文档的分工：[`README.md`](README.md)（入口与导航）、[`USAGE.md`](USAGE.md)（使用教程）、
  本文件（版本变更与升级）；v2.1.0 的库与头文件在 [`lib/v2.1.0/`](lib/v2.1.0/)，
  其中 [`lib/v2.1.0/README.md`](lib/v2.1.0/README.md) 是这一版包的文件清单与接入前提；
- 接口的**权威说明**始终是 `lib/<版本>/dmcan.h` / `dmcan_types.h` 的注释（调用时机、失败语义、
  返回值含义、回调线程约定都写在里面）；
- GitHub 英文展示仓的覆盖范围与 Gitee 不同，见仓库根 README 里的链接说明。
