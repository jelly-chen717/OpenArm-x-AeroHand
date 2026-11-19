# DM USB2CANFD SDK v2.1.0 使用说明（Rust）

本目录是 Rust 语言封装层的对外交付物：底层直接调用 C SDK 导出的 C ABI，
API 与 C 侧一一对应（枚举设备、通道配置、发送 / 接收、事件订阅、总线状态与统计、USB2UART 透传等）。

> 支持平台：**Windows（x64，MSVC）**、**Linux（x86_64 / aarch64）**、**macOS（Apple Silicon / Intel）**。
> crates.io 上尚未发布，现阶段请从本目录的 `pack/` 取包使用（见 §3）。

---

## 1. 支持的模块与固件版本

| 模块 | USB 标识 | 固件版本要求 |
| --- | --- | --- |
| **USB2CANFD_S**（单路） | `VID 0x34B7 / PID 0x6877` | **V2.0.0 及以上** |
| **USB2CANFD_D**（双路） | `VID 0x34B7 / PID 0x6632` | **V2.0.0 及以上** |
| **LinkX4C**（四路） | `VID 0x34B7 / PID 0x4543` | **V2.0.0 及以上** |

- **固件构建标识要求**：单路 `d64e840` 起、双路 `b5c2574` 起。低于这些构建标识的固件，
  通道开关与看门狗相关调用可能异常（固件版本号没变，只能按构建标识判——不确定就先升级到最新固件）。
  构建标识可用 `Device::firmware_info()` 读取（配合 `FirmwareInfo::to_string_line()` 打印成一行）。

---

## 2. 本目录有什么

```
Rust/
├─ pack/dmcan-2.1.0.crate    cargo 包：Rust 源码 + 各平台原生库（Windows / Linux / macOS）
└─ README.md                 本说明
```

包内**不含 libusb**（第三方库），需要另行准备，见 §4。

---

## 3. 快速开始

### 3.1 引入

**方式一：解包当本地依赖（离线，推荐）**

```bash
tar -xf pack/dmcan-2.1.0.crate          # 得到 dmcan-2.1.0/ 目录
```

```toml
[dependencies]
dmcan = { path = "路径/dmcan-2.1.0" }
```

**方式二：crates.io** —— 尚未发布，发布后在 `Cargo.toml` 里写 `dmcan = "2.1"` 即可。

### 3.2 最小示例

```rust
use dmcan::{ChannelBaudInfo, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

let ctx = Context::new()?;                       // 创建上下文（含版本配套校验）
if ctx.find_devices() == 0 { return Ok(()); }
let dev = ctx.device(0)?;
dev.open()?;
println!("{}", dev.firmware_info()?.to_string_line()?);

let (tx, rx) = std::sync::mpsc::channel();
let _sub = dev.subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
    if let Event::RxFrame(frame) = event { let _ = tx.send(frame.to_owned()); }
})?;

// 注意顺序：先配参数、再使能通道（见 §5）
dev.set_channel_baudrate(0, &ChannelBaudInfo {
    canfd: true, retrans: true, can_baudrate: 1_000_000, can_sp: 0.75,
    canfd_baudrate: 1_000_000, canfd_sp: 0.75, ..Default::default()
})?;
std::thread::sleep(std::time::Duration::from_millis(1200));
dev.enable_channel(0)?;
dev.send_can(0, &Frame::fd(0x123, &[1, 2, 3], true)?)?;
# Ok::<(), dmcan::Error>(())
```

### 3.3 示例程序

解包后可直接跑（需要接模块；没接模块时 `basic` 会提示并正常退出）：

```bash
cargo run --example basic       # 枚举 → 打开 → 读固件信息 → 配参数 → 发一帧 → 读统计
cargo run --example subscribe   # 订阅事件（收帧 / 真假 echo / 通道错误）观察回调
```

---

## 4. 运行前提

| 平台 | libusb（随包不含） | 其它 |
| --- | --- | --- |
| Windows | 自备 `libusb-1.0.dll`，放到 `PATH` 或可执行文件同目录 | — |
| Linux | 装系统包，如 `apt install libusb-1.0-0` | 普通用户需要 udev 规则才能打开设备节点 |
| macOS | 自备 `libusb-1.0.0.dylib`，**与 `libdmcan.dylib` 放在同一目录** | — |

---

## 5. 使用注意

- **先配参数、再使能通道**：参数下发后通道会回到未使能状态，先使能再配会导致之后发送的帧被丢弃；
- **回调在 SDK 自己的线程上同步执行**：闭包要快速返回、不要在里面做落盘 / 阻塞操作；回调里的帧数据只在回调期间有效，要留存请 `to_owned()`；
- **CAN FD 高数据段波特率（4~5 Mbps）需要总线两端都有端接**：只有一侧端接时表现为"帧提交了但一直不完成"；单模块自测可用外部回环模式（不依赖外部 ACK）；
- **运行期配置不落盘**：波特率、终端电阻、echo 开关等拔插后回默认值；要保留请显式调用保存接口，拔插后还需要重新使能通道。

---

## 6. 版本

当前交付 **v2.1.0**：封装层版本与它依赖的 C SDK 版本保持一致。
更详细的行为约定与限制以包内源码注释为准（每个方法都注明了它对应的 C 接口）。

已完成真机验证：Windows x64（MSVC）、Linux x86_64、macOS arm64 三平台，覆盖枚举、通道配置、
发送 / 接收、真假 echo、跨设备收帧、硬件过滤表、外部回环、看门狗、UART 回环等用例。
