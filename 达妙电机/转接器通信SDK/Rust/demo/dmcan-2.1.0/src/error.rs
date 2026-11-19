//! 错误类型：把 C 侧 `dmcan_status_t` 与包装层自己发现的错误统一成一个 enum。

use core::fmt;

use crate::ffi;

/// dmcan 调用失败的原因。
///
/// 前 8 个变体与 C 侧 `dmcan_status_t` 一一对应；后两个是包装层自己产生的
/// （`VersionMismatch` 来自库与绑定不配套，`Unknown` 兜住将来新增的 C 侧错误码）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// 入参非法（空指针、通道号越界等）。
    InvalidArg,
    /// 设备尚未 `open()`（或已 `close()`）。
    NotOpen,
    /// 设备索引越界 / 当前没有设备。
    DeviceNotFound,
    /// 内存不足。
    NoMemory,
    /// 与设备交互失败（USB 超时 / 固件拒绝）。
    Io,
    /// 设备或固件不支持该能力。
    Unsupported,
    /// 内部状态异常（不该发生）。
    Internal,
    /// 在 SDK 自己的回调线程上调用了会阻塞/自锁的接口，调用被拒绝（设备状态未变）。
    Reentrant,
    /// 库与绑定不配套：`dmcan_check_version()` 认为手里这个库不是 2.0.1 的 ABI。
    VersionMismatch {
        /// 绑定期望的版本整型。
        expected: u32,
        /// 库里记录的版本整型，读不到时为 0。
        actual: u32,
    },
    /// C 侧返回了本绑定未认识的错误码。
    Unknown(i32),
}

/// 本 crate 的 `Result` 别名。
pub type Result<T> = core::result::Result<T, Error>;

impl Error {
    /// 把 C 侧错误码映射成 Rust 错误；`DMCAN_OK` 返回 `None`。
    pub(crate) fn from_status(status: i32) -> Option<Error> {
        match status {
            ffi::DMCAN_OK => None,
            ffi::DMCAN_ERR_INVALID_ARG => Some(Error::InvalidArg),
            ffi::DMCAN_ERR_NOT_OPEN => Some(Error::NotOpen),
            ffi::DMCAN_ERR_DEVICE_NOT_FOUND => Some(Error::DeviceNotFound),
            ffi::DMCAN_ERR_NO_MEMORY => Some(Error::NoMemory),
            ffi::DMCAN_ERR_IO => Some(Error::Io),
            ffi::DMCAN_ERR_UNSUPPORTED => Some(Error::Unsupported),
            ffi::DMCAN_ERR_INTERNAL => Some(Error::Internal),
            ffi::DMCAN_ERR_REENTRANT => Some(Error::Reentrant),
            other => Some(Error::Unknown(other)),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Error::InvalidArg => "入参非法",
            Error::NotOpen => "设备尚未打开",
            Error::DeviceNotFound => "设备不存在",
            Error::NoMemory => "内存不足",
            Error::Io => "与设备交互失败",
            Error::Unsupported => "设备或固件不支持该能力",
            Error::Internal => "SDK 内部状态异常",
            Error::Reentrant => "不允许在 SDK 回调线程上调用该接口",
            Error::VersionMismatch { .. } => "库与绑定不配套",
            Error::Unknown(_) => "未知错误码",
        };
        match self {
            Error::VersionMismatch { expected, actual } => {
                write!(f, "{text}（期望 0x{expected:08X}，库里是 0x{actual:08X}）")
            }
            Error::Unknown(code) => write!(f, "{text}（{code}）"),
            _ => f.write_str(text),
        }
    }
}

impl std::error::Error for Error {}

/// 失败后立刻读取**本线程**的错误码。
///
/// C 侧的错误码是线程局部的、每次 API 调用都会重新置位，所以只能在调用失败的同一线程、
/// 紧接着读取——调用方（本 crate 的各包装函数）必须在 FFI 调用之后马上调它。
pub(crate) fn last_error_after_failure() -> Error {
    // 返回 false 却拿到 OK：C 侧约定里不该出现，按内部异常处理，避免谎报成功。
    last_status().unwrap_or(Error::Internal)
}

/// 读取本线程的错误码；`DMCAN_OK` 返回 `None`。
///
/// 用于区分 C 侧两种 `false`：一种是失败（错误码非 OK），另一种是"没拿到结果"
/// （例如"还没收到过周期上报"），后者不该被包装成错误。
pub(crate) fn last_status() -> Option<Error> {
    // SAFETY: 只是读取本线程的状态，无参数、无指针。
    let status = unsafe { ffi::dmcan_get_last_error() };
    Error::from_status(status)
}

/// 把 C 侧 `bool` 返回值转成 `Result<()>`（失败时立刻取本线程错误码）。
pub(crate) fn status_ok(ok: bool) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(last_error_after_failure())
    }
}
