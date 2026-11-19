//! SDK 上下文与全局接口：版本、日志、设备枚举、热插拔。
//!
//! `Context` 持有 C 侧上下文的所有权（内部是 `Arc`，`Device` / 订阅也会各持一份引用），
//! 因此"先丢 `Context`、后面继续用 `Device`"是安全的：C 侧上下文要到最后一个引用消失才销毁。

use core::ptr;
use std::sync::Arc;

use crate::device::Device;
use crate::error::{last_error_after_failure, Error, Result};
use crate::ffi;
use crate::subscribe::{self, HotplugGuard, LogSubscription};
use crate::types::{DeviceType, HotplugEvent, LogLevel, LogRecord, Version};

/// 上下文内部状态：最后一个引用被释放时销毁 C 侧上下文。
pub(crate) struct ContextInner {
    raw: *mut ffi::dmcan_context,
}

impl ContextInner {
    /// C 侧上下文指针。
    pub(crate) fn raw(&self) -> *mut ffi::dmcan_context {
        self.raw
    }
}

impl Drop for ContextInner {
    fn drop(&mut self) {
        // SAFETY: 指针来自成功的 dmcan_context_create；此刻所有 Device / 订阅都已释放。
        // 注意 C 侧的 destroy 会 join 内部线程。
        unsafe { ffi::dmcan_context_destroy(self.raw) };
    }
}

/// SDK 上下文。
///
/// 用法：`Context::new()` → `find_devices()` → `device(index)` → `open()` → 配置/收发。
///
/// **线程模型**：C 侧头文件只对"回调线程 / 重入"给了明确约定，没有覆盖全部接口的线程安全承诺，
/// 因此本类型**默认不实现 `Send` / `Sync`**（拿到的指针天然如此），请在单个线程里使用；
/// 回调闭包仍然要求 `Send`，因为它会被 SDK 自己的通知线程调用。
pub struct Context {
    inner: Arc<ContextInner>,
}

impl Clone for Context {
    fn clone(&self) -> Self {
        Context {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl Context {
    /// 创建上下文（`dmcan_context_create`），并立刻校验"库与绑定是否配套"。
    ///
    /// 校验用的是 C 侧 `dmcan_check_version(DMCAN_VERSION_INT)`：不配套时返回
    /// [`Error::VersionMismatch`]（ABI 变更后会表现为结构体错位，所以这里必须挡住）。
    pub fn new() -> Result<Context> {
        let mut raw: *mut ffi::dmcan_context = ptr::null_mut();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        unsafe { ffi::dmcan_context_create(&mut raw) };
        if raw.is_null() {
            // C 侧 `dmcan_context_create` 无返回值，拿不到句柄只能按内存不足处理。
            return Err(Error::NoMemory);
        }
        let inner = Arc::new(ContextInner { raw });
        // SAFETY: 只是查库里的版本常量，不需要句柄。
        let compatible = unsafe { ffi::dmcan_check_version(ffi::DMCAN_VERSION_INT) };
        if !compatible {
            let mut packed = 0u32;
            // SAFETY: 句柄刚创建、仍有效。
            unsafe { ffi::dmcan_get_sdk_version(raw, &mut packed) };
            drop(inner); // 立刻销毁上下文，不把不配套的库留在手里
            return Err(Error::VersionMismatch {
                expected: ffi::DMCAN_VERSION_INT,
                actual: packed,
            });
        }
        Ok(Context { inner })
    }

    /// 裸上下文指针（逃逸通道）。
    ///
    /// # Safety
    ///
    /// 调用方自己保证不在 C 侧销毁上下文的时机之外使用它。
    pub unsafe fn as_raw(&self) -> *mut ffi::dmcan_context {
        self.inner.raw()
    }

    /// 读取 SDK 版本（`dmcan_get_sdk_version`）。
    ///
    /// 版本号**权威在库里**：返回的是库编译时写死的值。
    pub fn sdk_version(&self) -> Version {
        let mut packed = 0u32;
        // SAFETY: 上下文有效；输出参数指向本栈帧的局部变量。
        unsafe { ffi::dmcan_get_sdk_version(self.inner.raw(), &mut packed) };
        Version {
            major: (packed >> 24) as u8,
            minor: (packed >> 16) as u8,
            build: (packed >> 8) as u8,
        }
    }

    /// 打印 SDK 版本到终端（`dmcan_print_version`，会往 stdout 写字）。
    pub fn print_version(&self) {
        // SAFETY: 上下文有效。
        unsafe { ffi::dmcan_print_version(self.inner.raw()) };
    }

    /// 枚举设备并刷新句柄表（`dmcan_find_devices`）。
    ///
    /// 返回**句柄表里的设备总数（含已拔掉的）**；要判断某台还在不在，用
    /// [`Device::is_present`]。句柄表只增不减，所以重新枚举不会让手里的句柄失效。
    pub fn find_devices(&self) -> usize {
        // SAFETY: 上下文有效。
        let count = unsafe { ffi::dmcan_find_devices(self.inner.raw()) };
        count.max(0) as usize
    }

    /// 按型号枚举设备（`dmcan_find_devices_with_type`），返回表内该型号的设备总数。
    pub fn find_devices_with_type(&self, device_type: DeviceType) -> usize {
        // SAFETY: 上下文有效。
        let count =
            unsafe { ffi::dmcan_find_devices_with_type(self.inner.raw(), device_type.to_raw()) };
        count.max(0) as usize
    }

    /// 打印句柄表里全部设备的信息到终端（`dmcan_show_all_devices`，诊断用）。
    pub fn show_all_devices(&self) {
        // SAFETY: 上下文有效。
        unsafe { ffi::dmcan_show_all_devices(self.inner.raw()) };
    }

    /// 按句柄表下标取设备（`dmcan_device_get`，下标含义同上：含已拔掉的设备）。
    pub fn device(&self, index: usize) -> Result<Device> {
        let mut handle: *mut ffi::dmcan_device_handle = ptr::null_mut();
        // SAFETY: 上下文有效；输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get(self.inner.raw(), &mut handle, index as i32) };
        if !ok || handle.is_null() {
            return Err(last_error_after_failure());
        }
        Ok(Device::new(handle, Arc::clone(&self.inner)))
    }

    /// 注册热插拔回调（`dmcan_context_register_hotplug`；同一上下文只保留一个，重复注册以最后一次为准）。
    ///
    /// 三条约定：
    ///
    /// - 回调在 libusb 事件线程（或 SDK 的轮询线程）上同步执行，必须快速返回；
    /// - 事件**只报描述表里认识的设备**，无关 USB 设备不会打扰；
    /// - **不要在回调里注销热插拔**（会死锁），要停就把 `HotplugGuard` 丢掉。
    pub fn register_hotplug<F>(&self, callback: F) -> Result<HotplugGuard>
    where
        F: FnMut(HotplugEvent) + Send + 'static,
    {
        subscribe::register_hotplug(self.inner.raw(), callback)
    }

    /// 本机 libusb 的**原生**热插拔能力是否可用（`dmcan_context_hotplug_supported`，诊断用）。
    ///
    /// 返回 `false` 不影响注册：SDK 内部用低频枚举比对兜底。
    pub fn hotplug_supported(&self) -> bool {
        // SAFETY: 上下文有效。
        unsafe { ffi::dmcan_context_hotplug_supported(self.inner.raw()) }
    }
}

/// 终端日志控制与日志订阅（进程级，不需要上下文）。
pub mod log {
    use super::*;

    /// 设置终端日志级别（`dmcan_set_log_level`）：低于该级别的日志不再输出。
    ///
    /// 默认 `Info`；也可用环境变量 `DMCAN_LOG_LEVEL` 设初始值。
    /// 注意 Release 构建默认把 trace/debug 编译裁剪掉，设成 `Trace` 也拿不到那两级。
    pub fn set_level(level: LogLevel) {
        // SAFETY: 只设置进程级状态。
        unsafe { ffi::dmcan_set_log_level(level.as_i32()) };
    }

    /// 读取当前终端日志级别（`dmcan_get_log_level`）。
    pub fn level() -> LogLevel {
        // SAFETY: 只读进程级状态。
        LogLevel::from_i32(unsafe { ffi::dmcan_get_log_level() })
    }

    /// 立即刷新日志缓冲（`dmcan_flush_log`；warn 及以上本来就会立即 flush）。
    pub fn flush() {
        // SAFETY: 只刷缓冲。
        unsafe { ffi::dmcan_flush_log() };
    }

    /// 注册日志订阅（`dmcan_add_log_callback`，最多 4 个）。
    ///
    /// `min_level` 是**该回调自己的阈值**，与终端级别互相独立：终端可以用 `set_level(Off)`
    /// 静音，回调仍按自己的阈值收日志。
    ///
    /// 回调**同步执行在产生日志的那条线程上**，不要在里面做重活（要落盘就自己攒队列由
    /// 自己的线程写），也不要在回调里再调 SDK 的日志接口。`LogRecord::line` 只在回调内有效。
    pub fn subscribe<F>(min_level: LogLevel, callback: F) -> Result<LogSubscription>
    where
        F: for<'a> FnMut(LogRecord<'a>) + Send + 'static,
    {
        subscribe::subscribe_log(callback, min_level)
    }
}

/// 纯函数工具（不碰设备、不需要上下文）。
pub mod utils {
    use crate::error::{Error, Result};
    use crate::ffi;

    /// 数据长度 → DLC，**向上取整**到最小能装下它的那一档（`9~12 → 9`、`13~16 → 10`…）。
    ///
    /// 线格式里能表达的长度只有经典 `0~8`、CAN FD 再加 `12/16/20/24/32/48/64`，
    /// 所以"9 字节"只能按 DLC 9 发送、线上长度是 12 字节（尾部补 0）。
    /// `len > 64` 返回 [`Error::InvalidArg`]。
    pub fn dlc_from_len(len: u8) -> Result<u8> {
        // SAFETY: 纯函数，无指针参数。
        let dlc = unsafe { ffi::dmcan_utils_get_dlc_from_len(i32::from(len)) };
        u8::try_from(dlc).map_err(|_| Error::InvalidArg)
    }

    /// DLC → 数据长度，按表原样返回（`9 → 12`、`15 → 64`）；`dlc > 15` 返回 [`Error::InvalidArg`]。
    pub fn len_from_dlc(dlc: u8) -> Result<u8> {
        // SAFETY: 纯函数，无指针参数。
        let len = unsafe { ffi::dmcan_utils_get_len_from_dlc(i32::from(dlc)) };
        u8::try_from(len).map_err(|_| Error::InvalidArg)
    }
}
