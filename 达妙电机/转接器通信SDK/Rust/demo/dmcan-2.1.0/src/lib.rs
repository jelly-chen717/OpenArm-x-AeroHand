//! # dmcan —— USB2CANFD SDK 的 Rust 封装层
//!
//! 薄薄一层：底层直接调用 C SDK（`dmcan`）导出的 C ABI，不重新实现任何协议、调度或缓存逻辑。
//! `unsafe` 全部收在 [`ffi`] 模块里，外面只做三件事：句柄生命周期（RAII）、错误码（`Result`）、
//! 回调（闭包转发）。
//!
//! ```no_run
//! use dmcan::{ChannelBaudInfo, Context, Events, Frame, CHANNEL_MASK_ALL};
//!
//! let ctx = Context::new()?;                        // dmcan_context_create + 版本配套校验
//! if ctx.find_devices() == 0 {
//!     return Ok(());
//! }
//! let dev = ctx.device(0)?;                         // dmcan_device_get
//! dev.open()?;                                      // dmcan_device_open
//! let fw = dev.firmware_info()?;                    // dmcan_device_get_firmware_info
//! println!("固件：{}", fw.to_string_line()?);       // 形如 USB2CANFD_D V2.1.0 [hash 时间]
//!
//! let (tx, rx) = std::sync::mpsc::channel();        // 用哪个通道由调用方决定，本 crate 不带依赖
//! let _sub = dev.subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
//!     if let dmcan::Event::RxFrame(frame) = event {
//!         let _ = tx.send(frame.to_owned());        // 帧只在回调内有效，要留存必须拷贝
//!     }
//! })?;
//!
//! dev.enable_channel(0)?;
//! dev.set_channel_baudrate(0, &ChannelBaudInfo {
//!     canfd: true,
//!     retrans: true,
//!     can_baudrate: 1_000_000,
//!     canfd_baudrate: 4_000_000,
//!     ..Default::default()
//! })?;
//! dev.send_can(0, &Frame::fd(0x123, &[1, 2, 3], true)?)?;
//! # Ok::<(), dmcan::Error>(())
//! ```
//!
//! ## 与 C 侧约定的对应关系
//!
//! - **错误码是线程局部的**（`dmcan_get_last_error`）：包装层在每次调用失败的**同一线程**、
//!   紧接着读取，对外只给 [`Result`]，调用方不需要碰错误码；
//! - **回调在 SDK 自己的线程上同步执行**：闭包要求 `Send`，回调里 panic 不会穿过 C ABI
//!   （被吞掉并计入 `Subscription::panic_count`）；
//! - **回调负载只在回调内有效**：帧 / UART 数据给的是借用视图，要留存请显式 `to_owned()`；
//! - **句柄表只增不减**：设备拔掉后句柄仍然有效，`Device` 也不代表"独占"，
//!   可以 `clone` 出多个副本；
//! - **运行期配置不落盘**：波特率、终端电阻、echo 开关等改动拔插后就回到上次保存的那一套，
//!   要持久化得显式 `Device::save_param()`。
//!
//! ## 线程模型
//!
//! C 侧头文件只对"回调线程 / 重入"给了明确约定，没有覆盖全部接口的线程安全承诺，
//! 因此 [`Context`] / [`Device`] / 订阅句柄**默认不实现 `Send` / `Sync`**，请在单个线程里使用
//! （回调闭包例外：它必须 `Send`，因为它会被 SDK 的通知线程调用）。
//!
//! ## 不包装的部分
//!
//! `dmadmin`（IAP 升级 / 恢复出厂 / 读 UUID）在 C 侧只以**静态库**形态交付，没有动态导出，
//! 因此本 crate 不提供；需要时请用 C++ 侧的管理接口。旧式 hook 接口
//! （`dmcan_device_hook_*_callback`）与旧式日志回调也只是保留在 [`ffi`] 里，建议用
//! [`Device::subscribe`] 与 [`log::subscribe`]。

pub mod ffi;
pub mod types;

mod context;
mod device;
mod error;
mod subscribe;

pub use context::{log, utils, Context};
pub use device::Device;
pub use error::{Error, Result};
pub use subscribe::{HotplugGuard, LogSubscription, Subscription};
pub use types::{
    BulkSendConfig, BusState, ChannelBaudConf, ChannelBaudInfo, ChannelExtConfig, ChannelMode,
    ChannelStats, DeviceInfo, DeviceType, EchoStats, Event, Events, FilterAction, FilterIdType,
    FilterMode, FilterRule, FilterRuleType, FilterTable, FilterUsage, FirmwareInfo, Frame,
    FrameRef, HotplugEvent, HotplugKind, LogLevel, LogRecord, RxFrame, SendProfile, TdcStatus,
    UartConfig, UartStats, Version, CHANNEL_MASK_ALL, DROP_FLAG_RX_ECHO, DROP_FLAG_TX,
    MODULE_FEATURE_CAN_RES_SW,
};

/// 本 crate 绑定所对应的 C SDK 版本整型（`dmcan_version.h` 的 `DMCAN_VERSION_INT`）。
///
/// `2.1.0` → `0x0201_0000`。`Context::new()` 会拿它去调 `dmcan_check_version()`：
/// 手里这个库不是这一版就返回 [`Error::VersionMismatch`]。
pub const SDK_VERSION_INT: u32 = ffi::DMCAN_VERSION_INT;
