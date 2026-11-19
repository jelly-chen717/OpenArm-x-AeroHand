//! 回调转发层：把 C 侧的 C 函数指针回调转成 Rust 闭包，并管理闭包与订阅句柄的生命周期。
//!
//! 三条硬约束（都来自 C 侧注释，改这里必须守住）：
//!
//! 1. **回调在 SDK 自己的线程上同步执行**（每设备一条通知线程；日志回调在产生日志的那条线程上）；
//!    所以闭包必须 `Send`，且要做 `catch_unwind` 兜底——panic 穿过 `extern "C"` 是 UB；
//! 2. **回调负载只在回调期间有效**：帧 / UART 数据给的是借用视图，要留存由用户显式 `to_owned()`；
//! 3. **注销之后 C 侧不会再投递**（正在执行的那一条会先返回），所以可以先注销、再释放闭包。

use core::cell::UnsafeCell;
use core::ffi::c_void;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};
use std::ffi::CStr;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;

use crate::error::{last_error_after_failure, Result};
use crate::ffi;
use crate::types::{
    ChannelError, Event, Events, FrameRef, HotplugEvent, HotplugKind, LogLevel, LogRecord,
};

/// 设备事件闭包（`F: for<'a> FnMut(Event<'a>) + Send`）。
pub(crate) type BoxedEventCallback = Box<dyn for<'a> FnMut(Event<'a>) + Send>;

/// 日志闭包。
type BoxedLogCallback = Box<dyn for<'a> FnMut(LogRecord<'a>) + Send>;

/// 热插拔闭包。
type BoxedHotplugCallback = Box<dyn FnMut(HotplugEvent) + Send>;

/// 事件闭包的存放点。
///
/// 闭包要被 SDK 线程调用、又要被持有订阅的线程注销，因此用 `UnsafeCell` 承载"内部可变"，
/// 并手工声明 `Send + Sync`：SDK 保证**同一设备的事件回调串行执行**，
/// 闭包本身也要求 `Send`，所以不存在并发访问。
pub(crate) struct EventState {
    callback: UnsafeCell<BoxedEventCallback>,
    panics: AtomicUsize,
}

// SAFETY: 见上；被 SDK 线程调用时闭包自身是 Send，且同一设备不会并发进入。
unsafe impl Send for EventState {}
unsafe impl Sync for EventState {}

struct LogState {
    callback: UnsafeCell<BoxedLogCallback>,
    panics: AtomicUsize,
}

// SAFETY: 同 EventState；日志回调可能在产生日志的任意线程上并发进入，
// 因此这里的闭包由 `Mutex` 之外的"SDK 日志锁之外调用"约束兜底：C 侧注释说明
// 回调之间可能穿插，但仍然保证不会重入同一个订阅，故用 UnsafeCell + Send 闭包成立。
unsafe impl Send for LogState {}
unsafe impl Sync for LogState {}

struct HotplugState {
    callback: UnsafeCell<BoxedHotplugCallback>,
    panics: AtomicUsize,
}

// SAFETY: 热插拔回调在 libusb 事件线程（或轮询线程）上串行执行。
unsafe impl Send for HotplugState {}
unsafe impl Sync for HotplugState {}

/// 设备事件订阅（`Device::subscribe()` 的返回值）。
///
/// `Drop` 会自动调 `dmcan_device_unsubscribe()`，因此**把它丢掉就等于注销**；
/// 想立刻停止收事件就显式 `drop(guard)`。C 侧允许在回调里注销订阅，Rust 侧同样允许。
pub struct Subscription {
    device: *mut ffi::dmcan_device_handle,
    subscription: *mut ffi::dmcan_subscription,
    state: *const EventState,
    active: bool,
}

impl Subscription {
    /// 在本次回调里发生 panic 的次数（包装层吞掉了 panic，不让它穿过 C ABI）。
    ///
    /// 正常应当是 0；非 0 说明用户闭包里有 bug，事件仍会继续投递。
    pub fn panic_count(&self) -> usize {
        // SAFETY: `state` 由本结构体持有一个强引用，直到 Drop 才释放。
        unsafe { (*self.state).panics.load(Ordering::Relaxed) }
    }

    /// 立刻注销；返回 C 侧是否受理（重复注销返回 `false`）。`Drop` 也会做同一件事。
    pub fn unsubscribe(&mut self) -> bool {
        if !self.active {
            return false;
        }
        self.active = false;
        // SAFETY: 句柄来自成功的 dmcan_device_subscribe，且此时还未注销。
        unsafe { ffi::dmcan_device_unsubscribe(self.device, self.subscription) }
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        self.unsubscribe();
        // SAFETY: 与 `BoxedEventCallback` 的强引用配对释放（注销之后 C 侧不会再回调）。
        unsafe { drop(Arc::from_raw(self.state)) };
    }
}

/// 日志订阅（`log::subscribe()` 的返回值）。`Drop` 会自动注销。
pub struct LogSubscription {
    subscription: *mut ffi::dmcan_log_subscription,
    state: *const LogState,
    active: bool,
}

impl LogSubscription {
    /// 在本次回调里发生 panic 的次数。
    pub fn panic_count(&self) -> usize {
        // SAFETY: 同上，强引用由本结构体持有。
        unsafe { (*self.state).panics.load(Ordering::Relaxed) }
    }

    /// 立刻注销；返回 C 侧是否受理。
    pub fn unsubscribe(&mut self) -> bool {
        if !self.active {
            return false;
        }
        self.active = false;
        // SAFETY: 句柄来自成功的 dmcan_add_log_callback。
        unsafe { ffi::dmcan_remove_log_callback(self.subscription) }
    }
}

impl Drop for LogSubscription {
    fn drop(&mut self) {
        self.unsubscribe();
        // SAFETY: 与 `Arc::into_raw` 配对的强引用释放。
        unsafe { drop(Arc::from_raw(self.state)) };
    }
}

/// 热插拔监听（`Context::register_hotplug()` 的返回值）。`Drop` 会自动注销。
pub struct HotplugGuard {
    context: *mut ffi::dmcan_context,
    state: *const HotplugState,
    active: bool,
}

impl HotplugGuard {
    /// 在本次回调里发生 panic 的次数。
    pub fn panic_count(&self) -> usize {
        // SAFETY: 同上，强引用由本结构体持有。
        unsafe { (*self.state).panics.load(Ordering::Relaxed) }
    }
}

impl Drop for HotplugGuard {
    fn drop(&mut self) {
        if self.active {
            self.active = false;
            // SAFETY: 上下文仍存活（由调用方持有的 Context 保证），且本结构体只注销一次。
            unsafe { ffi::dmcan_context_unregister_hotplug(self.context) };
        }
        // SAFETY: 与 `Arc::into_raw` 配对的强引用释放。
        unsafe { drop(Arc::from_raw(self.state)) };
    }
}

/// 订阅设备事件。
pub(crate) fn subscribe<F>(
    device: *mut ffi::dmcan_device_handle,
    events: Events,
    channel_mask: u32,
    callback: F,
) -> Result<Subscription>
where
    F: for<'a> FnMut(Event<'a>) + Send + 'static,
{
    let state = Arc::into_raw(Arc::new(EventState {
        callback: UnsafeCell::new(Box::new(callback)),
        panics: AtomicUsize::new(0),
    }));
    let config = ffi::dmcan_subscribe_config_t {
        events: events.bits(),
        channel_mask,
        callback: Some(event_trampoline),
        user_data: state as *mut c_void,
    };
    let mut subscription: *mut ffi::dmcan_subscription = ptr::null_mut();
    // SAFETY: `device` 由调用方保证有效；`config` 在本次调用期间存活。
    let ok = unsafe { ffi::dmcan_device_subscribe(device, &config, &mut subscription) };
    if !ok {
        // SAFETY: 订阅失败，C 侧不会保存 `user_data`，这里把强引用收回并释放。
        unsafe { drop(Arc::from_raw(state)) };
        return Err(last_error_after_failure());
    }
    Ok(Subscription {
        device,
        subscription,
        state,
        active: true,
    })
}

/// 注册日志订阅（最多 4 个）。
pub(crate) fn subscribe_log<F>(callback: F, min_level: LogLevel) -> Result<LogSubscription>
where
    F: for<'a> FnMut(LogRecord<'a>) + Send + 'static,
{
    let state = Arc::into_raw(Arc::new(LogState {
        callback: UnsafeCell::new(Box::new(callback)),
        panics: AtomicUsize::new(0),
    }));
    let mut subscription: *mut ffi::dmcan_log_subscription = ptr::null_mut();
    // SAFETY: 只是把回调与 `user_data` 交给 C 侧登记。
    let ok = unsafe {
        ffi::dmcan_add_log_callback(
            Some(log_trampoline),
            min_level.as_i32(),
            state as *mut c_void,
            &mut subscription,
        )
    };
    if !ok {
        // SAFETY: 注册失败，C 侧不会保存 `user_data`。
        unsafe { drop(Arc::from_raw(state)) };
        return Err(last_error_after_failure());
    }
    Ok(LogSubscription {
        subscription,
        state,
        active: true,
    })
}

/// 注册热插拔回调（同一上下文只保留一个）。
pub(crate) fn register_hotplug<F>(
    context: *mut ffi::dmcan_context,
    callback: F,
) -> Result<HotplugGuard>
where
    F: FnMut(HotplugEvent) + Send + 'static,
{
    let state = Arc::into_raw(Arc::new(HotplugState {
        callback: UnsafeCell::new(Box::new(callback)),
        panics: AtomicUsize::new(0),
    }));
    // SAFETY: 只是把回调与 `user_data` 交给 C 侧登记。
    let ok = unsafe {
        ffi::dmcan_context_register_hotplug(context, Some(hotplug_trampoline), state as *mut c_void)
    };
    if !ok {
        // SAFETY: 注册失败，C 侧不会保存 `user_data`。
        unsafe { drop(Arc::from_raw(state)) };
        return Err(last_error_after_failure());
    }
    Ok(HotplugGuard {
        context,
        state,
        active: true,
    })
}

/// 设备事件回调入口（运行在 SDK 的通知线程上）。
///
/// # Safety
///
/// 由 C SDK 调用：`user_data` 必须是 `subscribe()` 里用 `Arc::into_raw` 交给它的指针。
unsafe extern "C" fn event_trampoline(
    _device: *mut ffi::dmcan_device_handle,
    event: *const ffi::dmcan_event_t,
    user_data: *mut c_void,
) {
    if user_data.is_null() || event.is_null() {
        return;
    }
    // **必须先 increment 再 from_raw**：`Arc::from_raw` 不增加计数，它接管的正是"交给 C 侧的那个
    // 引用"。如果直接 from_raw，回调结束把它 drop 掉就把整个状态释放了 —— 第二次回调会踩到
    // 已释放内存（实测 `0xc0000005`，vtable 指向 0xFEEEFEEE）。先 +1 拿一个真正属于自己的引用，
    // 才能保证"即使用户在回调里把 Subscription 丢掉，本次回调期间闭包依然存活"。
    unsafe { Arc::increment_strong_count(user_data as *const EventState) };
    let state = Arc::from_raw(user_data as *const EventState);
    let event = ptr::read_unaligned(event);
    if let Some(payload) = translate_event(&event) {
        // SAFETY: 同一设备的回调串行执行（C 侧保证），因此这里是独占访问。
        let callback = &mut *state.callback.get();
        if catch_unwind(AssertUnwindSafe(|| callback(payload))).is_err() {
            state.panics.fetch_add(1, Ordering::Relaxed);
        }
    }
    // `state` 在这里被释放：只是把上面那个临时强引用还回去。
}

/// 日志回调入口。
///
/// # Safety
///
/// 由 C SDK 调用：`user_data` 必须是 `subscribe_log()` 里交给它的指针，`line` 为 NUL 结尾字符串。
unsafe extern "C" fn log_trampoline(
    level: i32,
    seq: u64,
    line: *const core::ffi::c_char,
    user_data: *mut c_void,
) {
    if user_data.is_null() || line.is_null() {
        return;
    }
    // 同 `event_trampoline`：先 increment 再 from_raw，避免回调结束后状态被提前释放。
    unsafe { Arc::increment_strong_count(user_data as *const LogState) };
    let state = Arc::from_raw(user_data as *const LogState);
    let text = CStr::from_ptr(line).to_string_lossy();
    let record = LogRecord {
        level: LogLevel::from_i32(level),
        seq,
        line: text.as_ref(),
    };
    let callback = &mut *state.callback.get();
    if catch_unwind(AssertUnwindSafe(|| callback(record))).is_err() {
        state.panics.fetch_add(1, Ordering::Relaxed);
    }
}

/// 热插拔回调入口。
///
/// # Safety
///
/// 由 C SDK 调用：`user_data` 必须是 `register_hotplug()` 里交给它的指针。
unsafe extern "C" fn hotplug_trampoline(
    _context: *mut ffi::dmcan_context,
    event: i32,
    device: *const ffi::dmcan_hotplug_device_t,
    user_data: *mut c_void,
) {
    if user_data.is_null() || device.is_null() {
        return;
    }
    // 同 `event_trampoline`：先 increment 再 from_raw。
    unsafe { Arc::increment_strong_count(user_data as *const HotplugState) };
    let state = Arc::from_raw(user_data as *const HotplugState);
    let kind = if event == ffi::DMCAN_HOTPLUG_DETACHED {
        HotplugKind::Detached
    } else {
        HotplugKind::Attached
    };
    let payload = HotplugEvent::from_raw(kind, &*device);
    let callback = &mut *state.callback.get();
    if catch_unwind(AssertUnwindSafe(|| callback(payload))).is_err() {
        state.panics.fetch_add(1, Ordering::Relaxed);
    }
}

/// 把 C 侧事件翻译成 Rust 事件（不认识的类型返回 `None`）。
fn translate_event(event: &ffi::dmcan_event_t) -> Option<Event<'_>> {
    match event.event_type {
        ffi::DMCAN_EVENT_RX_FRAME if !event.frame.is_null() => {
            // SAFETY: C 侧保证帧指针在本次回调期间有效。
            Some(Event::RxFrame(unsafe { FrameRef::from_raw(event.frame) }))
        }
        ffi::DMCAN_EVENT_TX_COMPLETED if !event.frame.is_null() => {
            Some(Event::TxCompleted(unsafe {
                FrameRef::from_raw(event.frame)
            }))
        }
        ffi::DMCAN_EVENT_TX_ON_BUS if !event.frame.is_null() => {
            Some(Event::TxOnBus(unsafe { FrameRef::from_raw(event.frame) }))
        }
        ffi::DMCAN_EVENT_CHANNEL_ERROR if !event.error.is_null() => {
            Some(Event::ChannelError(unsafe {
                ChannelError::from_raw(event.error)
            }))
        }
        ffi::DMCAN_EVENT_UART_RX if !event.uart.is_null() => {
            // SAFETY: UART 负载在本次回调期间有效。
            let chunk = unsafe { ptr::read_unaligned(event.uart) };
            let data: &[u8] = if chunk.data.is_null() || chunk.length == 0 {
                &[]
            } else {
                // SAFETY: C 侧保证 `data` 至少 `length` 字节且本次回调期间有效。
                unsafe { core::slice::from_raw_parts(chunk.data, chunk.length as usize) }
            };
            Some(Event::UartRx(data))
        }
        _ => None,
    }
}
