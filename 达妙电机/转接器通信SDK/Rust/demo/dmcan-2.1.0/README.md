# USB2CANFD-SDK-Rust

USB2CANFD SDK 的 **Rust 封装层**：底层直接调用 C SDK（`dmcan`）导出的 C ABI，
`unsafe` 全部收在 `dmcan::ffi` 一个模块里，外面只做句柄生命周期（RAII）、错误码（`Result`）
与回调（闭包转发）三件事——不重新实现协议、调度或缓存逻辑。

上游 C++ SDK：`dm_device_sdk_new`（本仓库只读它，对应交付包 `dmcan_v2.1.0`）。

## 快速开始

```toml
[dependencies]
dmcan = { git = "http://gitlab.dmbot.cn/module/canfd-tools/usb2canfd-sdk-rust.git", branch = "main" }
```

```rust
use dmcan::{ChannelBaudInfo, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

let ctx = Context::new()?;                        // dmcan_context_create + 版本配套校验
if ctx.find_devices() == 0 {
    return Ok(());
}
let dev = ctx.device(0)?;                         // dmcan_device_get
dev.open()?;                                      // dmcan_device_open

let (tx, rx) = std::sync::mpsc::channel();
let _sub = dev.subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
    if let Event::RxFrame(frame) = event {
        let _ = tx.send(frame.to_owned());        // 帧只在回调内有效，要留存必须拷贝
    }
})?;

// 顺序：先配参数、再使能通道（见下面"使用注意"里的说明）
dev.set_channel_baudrate(0, &ChannelBaudInfo {
    canfd: true,
    retrans: true,
    can_baudrate: 1_000_000,
    can_sp: 0.75,
    canfd_baudrate: 1_000_000,
    canfd_sp: 0.75,
    ..Default::default()
})?;
std::thread::sleep(std::time::Duration::from_millis(1200));   // 等固件重载位定时
dev.enable_channel(0)?;
dev.send_can(0, &Frame::fd(0x123, &[1, 2, 3], true)?)?;
# Ok::<(), dmcan::Error>(())
```

可直接跑的示例：

```bash
cargo run --example basic       # 枚举 → 打开 → 读版本 → 配置 1M/4M → 发一帧 → 读统计
cargo run --example subscribe   # 订阅四类事件 + 真 echo，观察回调形状
```

## 原生库从哪来

`build.rs` 按目标平台挑选原生库，优先级：`DMCAN_SDK_DIR` 环境变量 > 本仓库 `native/<平台>/`。
它还会把该平台的运行期库投放到 `target/<profile>/` 与 `deps/`，所以 `cargo run` / `cargo test`
不用手工配 `PATH` / `LD_LIBRARY_PATH` / `DYLD_*`：

- Windows：`bin/*.dll`（Loader 只在 exe 同目录或 `PATH` 找）；
- Linux：`lib/*.so*`（`libdmcan.so` 的 SONAME 是 `libdmcan.so`，靠 `rpath=$ORIGIN` 找）；
- macOS：`lib/*.dylib`（install name 是 `@rpath/libdmcan.dylib`，靠 `rpath=@loader_path` 找）。

| 平台目录 | 目标三元组 | 状态 |
| --- | --- | --- |
| `native/msvc` | `x86_64-pc-windows-msvc` | **随包携带**（真机验证） |
| `native/mingw` | `x86_64-pc-windows-gnu` | 目录预留，未随附（用 `DMCAN_SDK_DIR` 指定 MinGW 交付包即可） |
| `native/linux_x86_64`、`native/linux_aarch64` | `x86_64-unknown-linux-gnu`、`aarch64-unknown-linux-gnu` | **随包携带**（真机验证） |
| `native/macos_arm64`、`native/macos_x86_64` | `aarch64-apple-darwin`、`x86_64-apple-darwin` | **随包携带**（真机验证） |

各平台的运行前提：

| 平台 | 运行期依赖 |
| --- | --- |
| Windows | 自备 `libusb-1.0.dll`（放到 `PATH` 或可执行文件同目录） |
| Linux | 装系统包 `libusb-1.0`（如 `apt install libusb-1.0-0`）；设备权限按 C++ 侧 README（udev 规则） |
| macOS | 自备 `libusb-1.0.0.dylib`（如 `brew install libusb`），**必须与 `libdmcan.dylib` 同目录**——`libdmcan.dylib` 内部按 `@loader_path/libusb-1.0.0.dylib` 找它 |

指向自建/新版 SDK：

```bash
DMCAN_SDK_DIR=/path/to/dmcan_v2.1.0/linux_x86_64 cargo build
```

> **crates.io 包里不含 libusb 的任何形态**（`libusb-1.0.dll` / `libusb-1.0.0.dylib`）：它属于第三方
> （LGPL-2.1），由使用方按上表自行提供。缺依赖时 `build.rs` 会在构建日志里给出明确提示；
> 用 `DMCAN_SDK_DIR` 指向**完整交付包**（里面自带 libusb，macOS 包就是这样）时，它会被一起拷到运行目录。

## 与 C 侧 API 的对应

安全层的方法**一一对应** C 侧函数（rustdoc 里写了对应的 `dmcan_*` 名字），
没有合并、没有改名、没有"更聪明的默认值"：

| 能力 | Rust 侧 |
| --- | --- |
| 上下文 / 枚举 / 热插拔 | `Context::{new, find_devices, find_devices_with_type, device, register_hotplug, show_all_devices}` |
| 打开 / 固件信息 | `Device::{open, close, is_present, info, firmware_info}` + `FirmwareInfo::to_string_line`（对应 `dmcan_device_get_firmware_info` / `dmcan_firmware_info_to_string`） |
| 通道与波特率 | `Device::{enable_channel, disable_channel, channel_baudrate, set_channel_baudrate, channel_baudrate_details, set_channel_baudrate_details, channel_tdc, channel_ext_config, set_channel_ext_config, save_param}`；节点模式见 `ChannelMode`（正常 / 内部回环 / 外部回环 / 只听） |
| 硬件过滤表 / 看门狗 | `Device::{set_channel_filter, channel_filter, enable_watchdog}` |
| 收帧 / 事件 | `Device::subscribe`（取代旧式 `hook_*`）+ `dmcan::{Event, Events, FrameRef, RxFrame}` |
| 发送 | `Device::{send_can, send_can_periodic, stop_send, send_can_batch, send_can_bulk, send_profile, set_send_profile}` |
| USB2UART 透传 | `Device::{set_uart_auto_open, set_uart_config, uart_is_open, uart_port_name, uart_open, uart_close, uart_write, uart_stats}` |
| 诊断 | `Device::{module_features, bus_state, channel_stats, clear_bus_off, echo_stats, set_echo_timeout_ms, enable_can_res, can_res, set_channel_echo}` |
| 日志 | `dmcan::log::{set_level, level, flush, subscribe}`（终端级别 + 最多 4 个订阅） |
| 纯函数工具 | `dmcan::utils::{dlc_from_len, len_from_dlc}` |
| 裸绑定（逃逸通道） | `dmcan::ffi::*`（73 个导出符号的原始声明） |

## 使用注意（都是从 C 侧继承来的硬约定）

- **回调同步执行在 SDK 自己的线程上**：闭包要求 `Send`，里面不要落盘、不要阻塞、不要加长锁；
  panic 不会穿过 C ABI（被吞掉并计入 `Subscription::panic_count`）。
- **回调负载只在回调内有效**：帧 / UART 数据是借用视图，`to_owned()` 才拷贝。
- **运行期配置不落盘**：波特率、终端电阻、echo 开关等改动拔插后就没了，
  要持久化得显式 `Device::save_param()`；拔插后还要重新 `enable_channel()`。
- **下发与生效不同步**：改波特率 / 过滤表后，硬件在固件每秒任务里重载（实测 0.086 ~ 1.0 s），
  刚下发就回读可能还是旧值。
- **顺序：先配参数、再使能通道**。参数下发会让固件重载 MCAN，重载后通道回到**未使能**状态
  （实测如此），先使能再配等于白配，之后发的帧会被计成 `tx_dropped_ch_disabled` 丢掉。
- **注销订阅与 `close()` 的顺序随意**：两者都支持。曾经记过"`close()` 之后注销会让 C 侧访问违规"，
  2026-10-10 查清**那是本封装自己的 use-after-free**（回调 trampoline 里 `Arc::from_raw` 没先
  increment 计数，回调一结束就把状态释放了），已修；C 侧没有问题。
- **CAN FD 高数据段波特率需要两端端接**：4~5 Mbps 数据相位对端接/线长很敏感，
  台架上只有一侧能软件控端接时会出现"帧提交后一直不完成"（没有 ACK）。
  `Device::enable_can_res()` 可开软件端接，但**不是所有硬件都支持**（能力位 bit0 = `MODULE_FEATURE_CAN_RES_SW`）。
- **句柄表只增不减**：设备拔掉后句柄仍然有效（`is_present()` 为 `false`），
  `Device` 可以 `clone`，`Drop` 不会关设备。
- **线程模型**：C 侧只对"回调线程 / 重入"给了明确约定，没有覆盖全部接口的线程安全承诺，
  因此 `Context` / `Device` 默认**不实现 `Send`/`Sync`**，请在单线程里使用。
- **不包装 `dmadmin`**（IAP 升级 / 恢复出厂 / 读 UUID）：上游只以静态库交付，没有动态导出。

## 版本

crate 版本**跟随被包装的 C SDK 版本**：当前 `2.1.0`（`dmcan_v2.1.0`，`DMCAN_VERSION_INT = 0x0201_0000`）。
`Context::new()` 会用 `dmcan_check_version()` 校验"手里的库"与"本绑定"是否配套，不配套直接报错。

## 验证

```bash
cargo test                                  # 离线用例（ABI、位域、DLC 边界、错误码线程局部性等）
cargo test --test hardware_test -- --ignored --test-threads=1   # 真机用例（需要接模块）
```

用例与报告的命名口径见 `AGENTS.md`；最近一次真机结果见 `tests/RUST_WRAPPER_SMOKE_TEST_REPORT.md`。
