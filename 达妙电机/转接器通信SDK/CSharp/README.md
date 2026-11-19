# 达妙 USB2CANFD .NET SDK（C#）

用 C# / .NET 驱动达妙 USB2CANFD 系列模块：经典 CAN 与 CAN FD 收发、周期与批量发送、真 / 假 echo、
硬件滤波表、看门狗与终端电阻、总线状态与统计、USB2UART 透传、热插拔与日志。
接口按底层 C SDK（`dmcan`）的语义一对一映射，不做额外抽象。

- 包：[`DM_DeviceSDK_CSharp`](https://www.nuget.org/packages/DM_DeviceSDK_CSharp)，当前 **2.1.1**，目标框架 `net8.0`
- 平台：**Windows x64、Linux x64 / arm64、macOS（Apple Silicon / Intel）**；原生库随包分发，**不用另外安装 C SDK**
- 许可证：**Apache-2.0**

本目录里还有：

- `pack/`：同一份 nupkg（另附 2.1.0、1.0.1 历史包）——断网或内网可直接从本地源安装
- `demo/`：最小可运行示例，已把 `pack/` 配成本地 NuGet 源，`cd demo && dotnet run` 即可
- `LICENSE`：Apache-2.0 全文

## 1. 安装

```bash
dotnet add package DM_DeviceSDK_CSharp
```

或写进工程文件：

```xml
<PackageReference Include="DM_DeviceSDK_CSharp" Version="2.1.1" />
```

离线 / 内网就从本目录安装：

```bash
dotnet add package DM_DeviceSDK_CSharp --version 2.1.1 --source pack
```

原生库会按运行时标识（RID）自动投放，不需要手工拷贝。

## 2. 最小可运行示例

```csharp
using DM_DeviceSDK_CSharp.core;

using var context = new DmCanContext();

if (context.FindAllDevice() == 0)   // 枚举（不传型号 = 全部型号）
    return;
context.ShowAllDevices();           // 打印每台设备的型号 / VID / PID / 序列号

var device = context.GetDevice(0)!;
device.ReceiveEchoEvent += args => Console.WriteLine($"RX {args.Frame}");   // 收帧回调

if (!device.Open())
{
    Console.WriteLine($"打开失败：{DmCanUtils.GetLastError()}");
    return;
}

// 顺序有要求：先配参数、再使能通道（参数下发会把通道恢复成未使能）
device.SetChannelBaudByInfo(0, new dmcan_channel_can_info
{
    canfd = true, retrans = true,
    can_baudrate = 1000000, canfd_baudrate = 5000000,
    can_sp = 0.75f, canfd_sp = 0.75f
});
device.EnableChannel(0, true);

device.Send(0, 0x123, canfd: true, ext: false, rtr: false, brs: true, dlen: 8,
            payload: new byte[] { 1, 2, 3, 4, 5, 6, 7, 8 });

Thread.Sleep(500);
device.Close();
```

没插模块也能跑：会打印库版本与枚举结果后正常结束。完整一点的示例见 `demo/`
（周期发送、批量发送、总线状态与统计、固件信息都有）：

```bash
cd demo
dotnet run
```

## 3. 模块与固件要求

| 模块 | USB 标识 | 固件要求 |
| --- | --- | --- |
| `USB2CANFD_S`（单路） | VID `0x34B7` / PID `0x6877` | V2.0.0 及以上 |
| `USB2CANFD_D`（双路） | VID `0x34B7` / PID `0x6632` | V2.0.0 及以上 |
| `LinkX4C`（四路） | VID `0x34B7` / PID `0x4543` | V2.0.0 及以上 |

固件的**版本号一直是 `V2.0.0`**，所以判断"升级有没有生效"要看**构建标识**：单路 `d64e840` 起、
双路 `b5c2574` 起、四路 `d5bc7db` 起。低于这些的固件上，通道开关与看门狗可能表现异常。

```csharp
if (device.GetFirmwareInfo(out var fw))
    Console.WriteLine($"{DmCanDevice.FormatFirmwareInfo(fw)} build={fw.BuildIdString()} valid={fw.build_id_valid}");
```

`build_id_valid == false` 表示固件没带构建标识；设备停在 boot 时版本信息也读不全（别当成设备坏了）。

## 4. 平台前提

| 平台 | 随包的原生库 | 你还要准备 |
| --- | --- | --- |
| Windows x64 | `dmcan.dll`、`libusb-1.0.dll` | 目标机装 VC++ 2015-2022 可再发行组件 |
| Linux x64 / arm64 | `libdmcan.so` | 系统 `libusb-1.0`（如 `apt install libusb-1.0-0`）；普通用户需要下面的 udev 规则 |
| macOS arm64 / x64 | `libdmcan.dylib`、`libusb-1.0.0.dylib` | macOS 11.0 及以上；两个 dylib 必须同目录（包内已摆好） |

Linux 普通用户打不开设备节点，需要两份 udev 规则（第二份只在用到 USB2UART 时要）：

```
# /etc/udev/rules.d/99-dmcan.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666"
# /etc/udev/rules.d/99-dmcan-tty.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="34b7", MODE="0666"
```

装好后先 `udevadm control --reload-rules`，再 `udevadm trigger --subsystem-match=tty --action=add`
（只跑 `udevadm trigger` 不会刷新已经存在的 tty 节点）。

## 5. 常用任务对照

| 想做的事 | 用哪个 |
| --- | --- |
| 找设备 / 打开 / 看设备信息 | `context.FindAllDevice()`、`GetDevice(i)`、`ShowAllDevices()`、`device.Open()`、`device.ToString()` |
| 配波特率（简略） | `GetChannelBaudInfo` / `SetChannelBaudByInfo`（`dmcan_channel_can_info`） |
| 配波特率（完整，含模式 / TDC） | `GetChannelBaudConfig` / `SetChannelBaudByConfig`（`dmcan_ch_baud_conf`） |
| 使能 / 关闭通道 | `EnableChannel(ch, state)` |
| 发一帧 | `Send(ch, id, canfd, ext, rtr, brs, dlen, payload)` |
| 周期发送 / 停止 | `SendPeriodic(ch, interval_ms, send_times, ...)`、`StopSend(ch)` |
| 一次发一批 | `SendBatch(ch, dmcan_frame[])`，帧用 `dmcan_frame.Create(id, data, ...)` 构造 |
| ID 递增 / 数据递增 / 区间发送 | `SendBulk(ch, dmcan_bulk_send_config, payload)`、`SetSendProfile(...)` |
| 收帧 | `device.ReceiveEchoEvent`（`args.Frame`） |
| 发送结果 echo | `device.SendEchoEvent`（假 echo `Ack=false`、真 echo `Ack=true`） |
| 通道错误 / UART 数据 | `device.ChannelErrorEvent`、`device.ErrorEchoEvent`、`device.UartDataEvent` |
| 硬件滤波表 | `SetChannelFilter(ch, mode, rules, out usage)` / `GetChannelFilter(ch, out table)` |
| 看门狗 | `EnableWatchdog(ms, on)` |
| 软件终端电阻 | `EnableCanRes(ch, on)` / `GetCanRes(ch, out on)` |
| TDC | `GetChannelTdc(ch, out status)` |
| 总线状态 / 统计 | `GetBusState(ch, out s)`、`GetChannelStats(ch, out s)`、`GetEchoStats(out s)`、`ClearBusOff(ch)` |
| 固件信息 | `GetFirmwareInfo(out fw)`、`GetFirmwareInfoString()`、`dmcan_firmware_info.BuildIdString()` |
| 把参数留到下次上电 | `SaveParam()` |
| USB2UART 透传 | `SetUartAutoOpen`、`SetUartConfig`、`OpenUart` / `CloseUart`、`WriteUart`、`GetUartPortName`、`GetUartStats` |
| 热插拔 | `context.RegisterHotplug()` + `context.HotplugEvent`（重插后要重新 `Open()` + `EnableChannel()`） |
| 日志 | `DmCanLog.SetLevel(level)`、`DmCanLog.Register(handler)` |
| 错误码 / DLC 换算 | `DmCanUtils.GetLastError()`、`GetDlcFromLen`、`GetLenFromDlc` |

## 6. 几个必须知道的约定

- **先配参数、再使能通道**：参数下发后通道会回到未使能状态，顺序反了之后发出的帧会被丢弃。
- **回调跑在 SDK 自己的线程上**（热插拔在 libusb 线程上）：里面必须快速返回，不要落盘、发网络、长时间加锁。
- **不要在回调里 `Close()`**：会被直接拒绝并置 `DMCAN_ERR_REENTRANT`，要关闭请回到自己的线程。
- **事件订阅是会话级的**：`Close()`、重新 `Open()`、热插拔重连都不会丢，不需要重新注册；一个设备最多 4 个订阅者。
- **设备复位或重新插拔后**：重新 `Open()` + `EnableChannel()`，终端电阻、通道 echo 这类会话级设置也要重新下发；
  波特率这类参数默认只改运行期，想留到下次上电要先 `SaveParam()`。
- **统计值来自固件每秒一条的周期上报**：刚打开或刚发完立刻读可能取不到值，判丢包要连着读两次看增量。
- **错误码是线程局部的**：失败后在同一线程立刻 `DmCanUtils.GetLastError()`，读完可 `ClearLastError()`。
- **长度按 DLC 向上取整**：`dlen` 会自动取整（9 → 线上 12 字节，不足补 0），接收侧读到的是线上长度。
- **改完通道模式要等约 1.3 s**：固件按秒重载配置（回环模式、波特率都这样）。

## 7. 常见问题

**打不开设备？** 先看 `DmCanUtils.GetLastError()`：Linux 多半是设备节点权限（见上面的 udev 规则），
Windows 多半是被别的进程独占（串口助手、另一个上位机）。

**发出去了但收不到？** 依次查：两端波特率与采样点是否一致、通道是否已使能、总线两端是否有终端电阻、
经典 CAN 与 CAN FD（`canfd` / `brs`）是否搞混。

**只有一块板子，怎么自测收发链路？** 把通道改成外部回环：帧照常驱动总线，同时回环到自己，
不依赖对端 ACK。

```csharp
dmcan_ch_baud_conf conf = default;
device.GetChannelBaudConfig(0, ref conf);
conf.Mode = dmcan_channel_mode.LOOPBACK_EXTERNAL;
device.SetChannelBaudByConfig(0, conf);
Thread.Sleep(1300);                        // 等固件重载
// 验证完记得改回 dmcan_channel_mode.NORMAL
```

**收不到"真 echo"？** 默认只有假 echo（`Ack=false`，USB 提交完成就算）。真正发上总线的帧（`Ack=true`）
要先打开 `device.SetChannelEcho(ch, true)`。

**看门狗把通道咬住了？** `EnableWatchdog(ms, on)` 是设备级的；打开后如果没人喂狗，会话结束后通道会
"使能了却留不住"（`EnableChannel` 返回 true，通道随后自动关闭）。把看门狗关掉或断电复位，再重新使能通道即可恢复。

## 8. 从 1.x 升上来（不兼容）

1.x 的封装（`libdm_device`、`usb_rx_frame`、`FillCanQueue`）在 2.x 里全部重写过，主要对照：

| 1.x 写法 | 2.x 写法 |
| --- | --- |
| `libdm_device` / `dm_device` 库 | **`dmcan` 库**（文件名变了；本包已随附） |
| `usb_rx_frame`（`canid_flags` / `dlc_flags` 位域） | `dmcan_rx_frame`：直读 `can_id`、`timestamp`、`channel`、`ext`、`rtr`、`canfd`、`brs`、`esi`、`ack`、`dlen` |
| `dmcan_ch_can_info` / `dmcan_ch_can_config` | `dmcan_channel_can_info` / `dmcan_ch_baud_conf` |
| `FillCanQueue(...)` + `SendCanQueue()` | `SendBatch(ch, dmcan_frame[])` |
| `Send(ch, interval_ms, step_id, stop_id, send_times, ...)` | `SendPeriodic(...)`；递增 / 区间发送改用 `SendBulk(...)` |
| `CanRxFrame` | `DmCanFrame`（收帧、发送 echo、错误帧都用它） |
| `DmCanDef` | `dmcan_def`（常量类） |
| 失败只看返回值 | 失败后立刻读 `DmCanUtils.GetLastError()`（线程局部） |

最容易踩的两条行为差异：**发送 echo 会来两次**（假 echo `Ack=false` → 真 echo `Ack=true`，
后者要先 `SetChannelEcho(ch, true)`）；**回调里不能 `Close()`**。

## 9. 版本与许可证

当前 **2.1.1**，绑定 **C SDK 2.1.0**。托管包的 major.minor 跟随所绑定的 C SDK，修订号记封装层自己的改动。

| 托管包 | 绑定的 C SDK | 说明 |
| --- | --- | --- |
| **2.1.1**（当前） | 2.1.0 | 相对 2.1.0 只补许可证与包元数据，代码相同 |
| 2.1.0 | 2.1.0 | 首个 2.x 正式发布 |
| 1.0.0 / 1.0.1 / 1.0.2 | 1.1.x | 旧接口，与 2.x **不兼容** |

已在 Windows x64（MSVC）、Linux x86_64、macOS arm64 三平台真机验证：枚举、通道配置与回读、
单帧 / 周期 / 批量 / 递增发送、收帧、真假 echo、跨设备收帧、硬件滤波与过滤行为、外部回环、TDC、
软件终端电阻、UART 回环、热插拔、多设备接入。

许可证 **Apache-2.0**（全文见 [LICENSE](LICENSE)）。
