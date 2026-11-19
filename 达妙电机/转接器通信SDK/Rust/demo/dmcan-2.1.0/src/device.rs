//! 设备句柄的安全包装（`Context::device()` 的返回值）。
//!
//! 方法名与 C 侧一一对应（rustdoc 里写了对应的 `dmcan_*` 函数），失败语义按 C 头注释逐条映射：
//!
//! - `false` + 错误码非 OK → `Err`；
//! - `false` 但语义是"没拿到结果"（如还没收到过周期上报）→ `Ok(None)`；
//! - 输出参数被 C 侧清零的行为原样保留，不替调用方兜底。

use core::ffi::c_char;
use core::ptr;
use std::ffi::CStr;
use std::sync::Arc;

use crate::context::ContextInner;
use crate::error::{last_status, status_ok, Error, Result};
use crate::ffi;
use crate::subscribe::{self, Subscription};
use crate::types::{
    BulkSendConfig, BusState, ChannelBaudConf, ChannelBaudInfo, ChannelExtConfig, ChannelStats,
    DeviceInfo, DeviceType, EchoStats, Events, FilterMode, FilterRule, FilterTable, FilterUsage,
    FirmwareInfo, Frame, RxFrame, SendProfile, TdcStatus, UartConfig, UartStats,
};

/// 设备句柄的封装。
///
/// 句柄本身由 SDK 的句柄表持有（**只增不减**），本类型只是它的借用视图：
///
/// - `Device` 可以 `clone`，多个副本指向同一台设备；
/// - `Device` **不拥有、也不释放**句柄，`Drop` 不会关闭设备，要关请显式 `close()`；
/// - 内部持有一份上下文引用，因此"先丢 `Context`、后面继续用 `Device`"是安全的；
/// - 设备被拔掉后句柄依然有效，`is_present()` 会返回 `false`。
pub struct Device {
    handle: *mut ffi::dmcan_device_handle,
    _context: Arc<ContextInner>,
}

impl Clone for Device {
    fn clone(&self) -> Self {
        Device {
            handle: self.handle,
            _context: Arc::clone(&self._context),
        }
    }
}

impl Device {
    pub(crate) fn new(handle: *mut ffi::dmcan_device_handle, context: Arc<ContextInner>) -> Self {
        Device {
            handle,
            _context: context,
        }
    }

    /// 裸句柄（逃逸通道：需要调用本 crate 未包装的 C 接口时用）。
    ///
    /// # Safety
    ///
    /// 调用方自己保证不把它交给会释放句柄的接口，也不在 SDK 回调线程上做自锁调用。
    pub unsafe fn as_raw(&self) -> *mut ffi::dmcan_device_handle {
        self.handle
    }

    // ------------------------------------------------------------ 打开 / 关闭 / 信息

    /// 打开设备（`dmcan_device_open`）。
    ///
    /// 打开时会按 VID/PID + 序列号自动打开 USB2UART 串口（可用 `set_uart_auto_open(false)` 关掉），
    /// 并读一次模块能力位缓存起来。
    pub fn open(&self) -> Result<()> {
        // SAFETY: 句柄由上下文句柄表持有，调用期间有效。
        status_ok(unsafe { ffi::dmcan_device_open(self.handle) })
    }

    /// 关闭设备（`dmcan_device_close`，可重复调用）。
    ///
    /// **不要在 SDK 的回调里调用**：C 侧会直接拒绝并返回 `DMCAN_ERR_REENTRANT`，
    /// 本方法把它原样映射成 `Error::Reentrant`。
    pub fn close(&self) {
        // SAFETY: 同上；C 侧允许重复调用。
        unsafe { ffi::dmcan_device_close(self.handle) };
    }

    /// 该设备是否还在总线上（`dmcan_device_is_present`）。
    ///
    /// 设备拔掉后句柄仍然有效，可以直接查询。
    pub fn is_present(&self) -> Result<bool> {
        let mut present = false;
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_is_present(self.handle, &mut present) };
        status_ok(ok).map(|()| present)
    }

    /// 读取枚举信息：型号 / PID / VID / 序列号（`dmcan_device_get_info`）。
    ///
    /// 只读会话里缓存的枚举信息，不需要设备处于 open 状态，也不与设备通信。
    pub fn info(&self) -> Result<DeviceInfo> {
        let mut device_type = 0;
        let mut pid = 0;
        let mut vid = 0;
        let mut serial: *mut u8 = ptr::null_mut();
        // SAFETY: 四个输出参数都指向本栈帧的局部变量。
        unsafe {
            ffi::dmcan_device_get_info(
                self.handle,
                &mut device_type,
                &mut pid,
                &mut vid,
                &mut serial,
            )
        };
        let serial = if serial.is_null() {
            None
        } else {
            // SAFETY: C 侧保证这是 NUL 结尾、随句柄生命周期有效的库内缓冲；
            // 这里立刻拷成 String，不把库内指针带出去。
            Some(
                unsafe { CStr::from_ptr(serial as *const c_char) }
                    .to_string_lossy()
                    .into_owned(),
            )
        };
        Ok(DeviceInfo {
            device: DeviceType::from_raw(device_type),
            pid,
            vid,
            serial,
        })
    }

    /// 读设备固件信息（`dmcan_device_get_firmware_info`）：型号 + 发布版本号 + 构建标识，
    /// 一条 `CMD_GET_VERSION` 拿全。
    ///
    /// 两个 `*_valid` 分开表达"这次读到了没有"：
    ///
    /// - `version_valid == false`：没拿到 app 的发布版本号（`version` 恒 `0.0.0`）。
    ///   **设备跑在 boot 时就是这种情况**——判"板子还停在 boot"看这个字段，不要看 `Result`；
    /// - `build_id == None`：固件没在版本应答里带构建标识（老固件），不算失败。
    ///
    /// 两样都没读到时 C 侧返回 `false` 并置 `DMCAN_ERR_IO`，这里映射成 `Err(Io)`；只读到一样算成功。
    /// 判"升级是否生效"**只能看 `build_id`**（发布版本号常常不变）。
    pub fn firmware_info(&self) -> Result<FirmwareInfo> {
        let mut raw = unsafe { core::mem::zeroed::<ffi::dmcan_firmware_info_t>() };
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_firmware_info(self.handle, &mut raw) };
        status_ok(ok)?;
        // 按值读取，避免对 packed 字段取引用。
        Ok(FirmwareInfo::from_raw(unsafe { ptr::read_unaligned(&raw) }))
    }

    // ------------------------------------------------------------ 通道开关与波特率

    /// 使能通道（`dmcan_device_enable_channel`）。
    pub fn enable_channel(&self, channel: u8) -> Result<()> {
        // SAFETY: 句柄有效；通道号由 C 侧校验。
        status_ok(unsafe { ffi::dmcan_device_enable_channel(self.handle, channel) })
    }

    /// 关闭通道（`dmcan_device_disable_channel`）。
    pub fn disable_channel(&self, channel: u8) -> Result<()> {
        // SAFETY: 同上。
        status_ok(unsafe { ffi::dmcan_device_disable_channel(self.handle, channel) })
    }

    /// 读通道波特率（`dmcan_device_get_channel_baudrate`）。
    ///
    /// 返回的结构里**没有 TDC 字段**：SDK 会在 CAN FD 且数据段波特率 > 1 Mbps 时自动打开 TDC。
    /// 要看当前生效的 TDC 用 [`Device::channel_tdc`]。
    pub fn channel_baudrate(&self, channel: u8) -> Result<ChannelBaudInfo> {
        let mut raw = ffi::dmcan_channel_can_info_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_channel_baudrate(self.handle, channel, &mut raw) };
        status_ok(ok)?;
        Ok(ChannelBaudInfo::from_raw(unsafe {
            ptr::read_unaligned(&raw)
        }))
    }

    /// 读通道完整下发结构（`dmcan_device_get_channel_baudrate_details`，含 TDC 字段）。
    pub fn channel_baudrate_details(&self, channel: u8) -> Result<ChannelBaudConf> {
        let mut raw = ffi::dmcan_ch_baud_conf_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe {
            ffi::dmcan_device_get_channel_baudrate_details(self.handle, channel, &mut raw)
        };
        status_ok(ok)?;
        Ok(ChannelBaudConf::from_raw(unsafe {
            ptr::read_unaligned(&raw)
        }))
    }

    /// 写通道波特率（`dmcan_device_set_channel_baudrate`）。**TDC 开关由 SDK 自动决定**。
    ///
    /// 下发与生效不同步：`CMD_SETUP_BAUD` 只写 RAM 并置重载标志，硬件重载发生在固件每秒任务里
    /// （实测 0.086 ~ 1.0 s），刚下发就回读可能还是旧值。
    /// 运行期改动**不落盘**，要断电保留请再调 [`Device::save_param`]。
    pub fn set_channel_baudrate(&self, channel: u8, config: &ChannelBaudInfo) -> Result<()> {
        let raw = config.to_raw();
        // SAFETY: `raw` 在本次调用期间存活。
        status_ok(unsafe { ffi::dmcan_device_set_channel_baudrate(self.handle, channel, &raw) })
    }

    /// 写通道完整下发结构（`dmcan_device_set_channel_baudrate_details`）。
    pub fn set_channel_baudrate_details(
        &self,
        channel: u8,
        config: &ChannelBaudConf,
    ) -> Result<()> {
        let raw = config.to_raw();
        // SAFETY: `raw` 在本次调用期间存活。
        status_ok(unsafe {
            ffi::dmcan_device_set_channel_baudrate_details(self.handle, channel, &raw)
        })
    }

    /// 读通道 TDC 的硬件实际生效值（`dmcan_device_get_channel_tdc`）。
    ///
    /// `TdcStatus::valid == false` 表示"没读到"（老固件应答不足 25B，或通道没启用 TDC），
    /// 而不是"生效值是 0"。
    pub fn channel_tdc(&self, channel: u8) -> Result<TdcStatus> {
        let mut raw = ffi::dmcan_tdc_status_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_channel_tdc(self.handle, channel, &mut raw) };
        status_ok(ok)?;
        let raw = unsafe { ptr::read_unaligned(&raw) };
        Ok(TdcStatus {
            valid: raw.valid,
            tdco: raw.tdco,
            tdcf: raw.tdcf,
            tdcv: raw.tdcv,
        })
    }

    /// 把当前运行期参数保存进设备闪存（`dmcan_device_save_param`）。
    ///
    /// 返回 `Ok` 只代表命令被受理，落盘由固件后台任务完成——要验证持久化请等一会儿再断电。
    pub fn save_param(&self) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_save_param(self.handle) })
    }

    /// 读通道扩展配置（`dmcan_device_get_channel_ext_config`）。
    pub fn channel_ext_config(&self, channel: u8) -> Result<ChannelExtConfig> {
        let mut raw = ffi::dmcan_channel_ext_config_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok =
            unsafe { ffi::dmcan_device_get_channel_ext_config(self.handle, channel, &mut raw) };
        status_ok(ok)?;
        Ok(ChannelExtConfig::from_raw(unsafe {
            ptr::read_unaligned(&raw)
        }))
    }

    /// 写通道扩展配置（`dmcan_device_set_channel_ext_config`）。
    ///
    /// 内部实现是"读回当前配置 → 只改这两个字段 → 写回"，不会影响该通道的波特率等其他参数。
    pub fn set_channel_ext_config(&self, channel: u8, config: &ChannelExtConfig) -> Result<()> {
        let raw = config.to_raw();
        // SAFETY: `raw` 在本次调用期间存活。
        status_ok(unsafe { ffi::dmcan_device_set_channel_ext_config(self.handle, channel, &raw) })
    }

    // ------------------------------------------------------------ 硬件过滤表

    /// 下发硬件过滤表（`dmcan_device_set_channel_filter`）。
    ///
    /// 两条硬约束：**改表前必须先 `disable_channel()`**（使能状态下固件回 `NACK_CALL_ERR`，
    /// 本方法返回 `Err`）；改完要重新 `enable_channel()`。
    /// 返回的用量是**受理结果**，硬件重载在固件每秒任务里做（≤1 s）。
    /// 表不随普通下发落盘，要断电保留得再调 [`Device::save_param`]。
    pub fn set_channel_filter(
        &self,
        channel: u8,
        mode: FilterMode,
        rules: &[FilterRule],
    ) -> Result<FilterUsage> {
        if rules.len() > ffi::DMCAN_FILTER_RULE_MAX {
            return Err(Error::InvalidArg);
        }
        let raw_rules: Vec<ffi::dmcan_filter_rule_t> =
            rules.iter().map(|rule| rule.to_raw()).collect();
        let rules_ptr = if raw_rules.is_empty() {
            ptr::null()
        } else {
            raw_rules.as_ptr()
        };
        let mut usage = ffi::dmcan_filter_usage_t::default();
        // SAFETY: 规则数组与输出参数都在本次调用期间存活；条数已按上限校验。
        let ok = unsafe {
            ffi::dmcan_device_set_channel_filter(
                self.handle,
                channel,
                mode.to_raw(),
                rules_ptr,
                raw_rules.len() as u16,
                &mut usage,
            )
        };
        status_ok(ok)?;
        let usage = unsafe { ptr::read_unaligned(&usage) };
        Ok(FilterUsage {
            std_used: usage.std_used,
            ext_used: usage.ext_used,
        })
    }

    /// 回读硬件过滤表（`dmcan_device_get_channel_filter`，整表一次带回）。
    ///
    /// 空表是合法结果（`rules` 为空）：白名单模式下的空表会拒收所有帧。
    pub fn channel_filter(&self, channel: u8) -> Result<FilterTable> {
        // 389B 的结构体放在堆上，避免栈上大对象。
        let mut raw = Box::new(unsafe { core::mem::zeroed::<ffi::dmcan_filter_table_t>() });
        // SAFETY: 输出参数指向堆上的缓冲，本次调用期间有效。
        let ok = unsafe { ffi::dmcan_device_get_channel_filter(self.handle, channel, &mut *raw) };
        status_ok(ok)?;
        let raw = unsafe { ptr::read_unaligned(raw.as_ref()) };
        let count = usize::from(raw.count).min(ffi::DMCAN_FILTER_RULE_MAX);
        let rules = raw.rules[..count]
            .iter()
            .map(|rule| FilterRule::from_raw(unsafe { ptr::read_unaligned(rule) }))
            .collect();
        Ok(FilterTable {
            mode: FilterMode::from_raw(raw.mode),
            std_max: raw.std_max,
            ext_max: raw.ext_max,
            rules,
        })
    }

    /// 使能/关闭通道看门狗（`dmcan_device_enable_watchdog`）。
    pub fn enable_watchdog(&self, ms: u16, state: bool) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_enable_watchdog(self.handle, ms, state) })
    }

    // ------------------------------------------------------------ 订阅

    /// 订阅设备事件（`dmcan_device_subscribe`，每设备最多 4 个订阅者）。
    ///
    /// 三条使用约定（C 侧强制）：
    ///
    /// - 回调**同步执行在 SDK 的通知线程上**，必须快速返回：不要落盘、不要阻塞、不要加长锁；
    /// - 回调参数里的帧 / UART 数据**只在回调内有效**，要留存请 `to_owned()`；
    /// - **可以在 `open()` 之前订阅**，而且 `close()` / 热插拔拔插都不会丢订阅。
    ///
    /// 闭包必须 `Send`（它会被 SDK 线程调用）；回调里 panic 不会穿过 C ABI
    /// （被包装层吞掉并计入 [`Subscription::panic_count`]）。
    pub fn subscribe<F>(
        &self,
        events: Events,
        channel_mask: u32,
        callback: F,
    ) -> Result<Subscription>
    where
        F: for<'a> FnMut(crate::types::Event<'a>) + Send + 'static,
    {
        subscribe::subscribe(self.handle, events, channel_mask, callback)
    }

    // ------------------------------------------------------------ 发送

    /// 单次发送一帧（`dmcan_device_send_can`）。
    ///
    /// `dlen` 会自动向上取整到最近的 DLC：线格式只能表达经典 `0~8`、
    /// CAN FD 再加 `12/16/20/24/32/48/64`，所以 9 字节会按 DLC 9 发送、线上长度是 12 字节。
    pub fn send_can(&self, channel: u8, frame: &Frame) -> Result<()> {
        let (can_id, canfd, ext, rtr, brs, dlen, payload) = frame.send_args();
        // SAFETY: `payload` 指向 `frame`，在本次调用期间存活。
        status_ok(unsafe {
            ffi::dmcan_device_send_can(
                self.handle,
                channel,
                can_id,
                canfd,
                ext,
                rtr,
                brs,
                dlen,
                payload,
            )
        })
    }

    /// 周期发送（`dmcan_device_send_can_details`；`interval_ms = 0` 等价单次发送）。
    ///
    /// `send_times = 0` 表示一直发，用 [`Device::stop_send`] 停止。
    /// 调度完全在 SDK 侧，不会向固件下发停止命令。
    pub fn send_can_periodic(
        &self,
        channel: u8,
        interval_ms: u32,
        send_times: u32,
        frame: &Frame,
    ) -> Result<()> {
        let (can_id, canfd, ext, rtr, brs, dlen, payload) = frame.send_args();
        // SAFETY: `payload` 指向 `frame`，在本次调用期间存活。
        status_ok(unsafe {
            ffi::dmcan_device_send_can_details(
                self.handle,
                channel,
                interval_ms,
                send_times,
                can_id,
                canfd,
                ext,
                rtr,
                brs,
                dlen,
                payload,
            )
        })
    }

    /// 停止指定通道的周期发送任务（`dmcan_device_stop_send`）。
    pub fn stop_send(&self, channel: u8) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_stop_send(self.handle, channel) })
    }

    /// 一次交一批帧（`dmcan_device_send_can_batch`）。
    ///
    /// 参数校验是全部先做的：某一帧参数非法时整批拒收，不会发出前面那些帧。
    /// 空数组算参数非法（C 侧约定）。
    pub fn send_can_batch(&self, channel: u8, frames: &[Frame]) -> Result<()> {
        if frames.is_empty() {
            return Err(Error::InvalidArg);
        }
        let raw: Vec<ffi::dmcan_frame_t> = frames.iter().map(|frame| frame.to_raw()).collect();
        // SAFETY: 数组在本次调用期间存活。
        status_ok(unsafe {
            ffi::dmcan_device_send_can_batch(self.handle, channel, raw.as_ptr(), raw.len() as u32)
        })
    }

    /// 批量/递增发送（`dmcan_device_send_can_bulk`）。
    ///
    /// `id_inc` 时的发送区间是 `[can_id, stop_id)`；`data_inc` 时 payload 按 `dlen` 字节的
    /// 小端无符号整数逐帧 +1；`send_times` 与 `stop_id` 谁先到谁停。
    pub fn send_can_bulk(&self, channel: u8, config: &BulkSendConfig<'_>) -> Result<()> {
        let raw = config.to_raw();
        // SAFETY: `raw.payload` 指向 `config.frame`，在本次调用期间存活。
        status_ok(unsafe { ffi::dmcan_device_send_can_bulk(self.handle, channel, &raw) })
    }

    /// 读取发送聚合档位（`dmcan_device_get_send_profile`）。
    pub fn send_profile(&self) -> Result<SendProfile> {
        let mut raw = 0;
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_send_profile(self.handle, &mut raw) };
        status_ok(ok).map(|()| SendProfile::from_raw(raw))
    }

    /// 设置发送聚合档位（`dmcan_device_set_send_profile`）。
    ///
    /// 该档位是**设备级**的，设置后立即生效；换档不改变 credit/队列语义。
    pub fn set_send_profile(&self, profile: SendProfile) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_set_send_profile(self.handle, profile.to_raw()) })
    }

    // ------------------------------------------------------------ USB2UART 透传

    /// 开关"打开设备时自动打开 UART"（`dmcan_device_set_uart_auto_open`，默认开）。
    ///
    /// 立即生效于**下一次** `open()`；不影响已经打开的端口。
    pub fn set_uart_auto_open(&self, enable: bool) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_set_uart_auto_open(self.handle, enable) })
    }

    /// 设置 UART 线参数（`dmcan_device_set_uart_config`），存下来给"自动打开"和 `uart_open(None)` 用。
    ///
    /// `baud_rate = 0` 表示不改线参数（沿用设备当前配置）。
    pub fn set_uart_config(&self, config: &UartConfig) -> Result<()> {
        let raw = config.to_raw();
        // SAFETY: `raw` 在本次调用期间存活。
        status_ok(unsafe { ffi::dmcan_device_set_uart_config(self.handle, &raw) })
    }

    /// 查询 UART 端口是否已打开（`dmcan_device_uart_is_open`）。
    pub fn uart_is_open(&self) -> Result<bool> {
        let mut is_open = false;
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_uart_is_open(self.handle, &mut is_open) };
        status_ok(ok).map(|()| is_open)
    }

    /// 取当前 UART 端口名（`dmcan_device_uart_get_port_name`，如 `COM70` / `/dev/ttyACM0`）。
    ///
    /// 端口没打开时 C 侧返回 `false` 且**不置错误码**，这里映射成 `Ok(None)`。
    pub fn uart_port_name(&self) -> Result<Option<String>> {
        self.call_optional_string(64, |buffer, size| unsafe {
            // SAFETY: 缓冲区是本函数持有的可写缓冲。
            ffi::dmcan_device_uart_get_port_name(self.handle, buffer, size)
        })
    }

    /// 手动打开 UART 端口（`dmcan_device_uart_open`；`config = None` 用存下来的那份线参数）。
    pub fn uart_open(&self, config: Option<&UartConfig>) -> Result<()> {
        let raw = config.map(|config| config.to_raw());
        let ptr = match raw.as_ref() {
            Some(raw) => raw as *const ffi::dmcan_uart_config_t,
            None => ptr::null(),
        };
        // SAFETY: `raw`（若存在）在本次调用期间存活。
        status_ok(unsafe { ffi::dmcan_device_uart_open(self.handle, ptr) })
    }

    /// 关闭 UART 端口（`dmcan_device_uart_close`，可重复调用）。
    ///
    /// 停读线程最多等一个读超时（约 100 ms）才返回。
    pub fn uart_close(&self) {
        // SAFETY: 句柄有效；C 侧允许重复调用。
        unsafe { ffi::dmcan_device_uart_close(self.handle) };
    }

    /// 往 UART 写一段数据（`dmcan_device_uart_write`，同步；写不完算失败）。
    ///
    /// 收数据走订阅事件 `Events::UART_RX`。
    pub fn uart_write(&self, data: &[u8]) -> Result<()> {
        // SAFETY: 数据切片在本次调用期间存活。
        status_ok(unsafe {
            ffi::dmcan_device_uart_write(self.handle, data.as_ptr(), data.len() as u32)
        })
    }

    /// 读 UART 统计（`dmcan_device_get_uart_stats`）。
    pub fn uart_stats(&self) -> Result<UartStats> {
        let mut raw = ffi::dmcan_uart_stats_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_uart_stats(self.handle, &mut raw) };
        status_ok(ok)?;
        Ok(UartStats {
            rx_bytes: raw.rx_bytes,
            tx_bytes: raw.tx_bytes,
            rx_dropped: raw.rx_dropped,
            errors: raw.errors,
            is_open: raw.is_open != 0,
        })
    }

    // ------------------------------------------------------------ 诊断

    /// 读取模块能力位（`dmcan_device_get_module_features`）。
    ///
    /// 数值在 `open()` 时读一次并缓存，本方法不重新查询设备。
    /// **设备没给出应答时返回 [`Error::Unsupported`]**（C 侧就是这么置的错误码），
    /// 这时调用方应按"无任何能力"处理。
    pub fn module_features(&self) -> Result<u16> {
        let mut features = 0u16;
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_module_features(self.handle, &mut features) };
        if ok {
            return Ok(features);
        }
        // 设备没应答时 C 侧置 UNSUPPORTED；万一是别的错误码（本地参数错）也原样往上抛。
        match last_status() {
            Some(error) => Err(error),
            None => Err(Error::Unsupported),
        }
    }

    /// 开关通道的软件终端电阻（`dmcan_device_enable_can_res`）。
    ///
    /// 返回 `Ok(())` 表示"实际状态已等于请求状态"。三条约束：状态不写 flash（拔插后要重新下发）；
    /// 写要求通道已使能（没使能时固件静默忽略，本方法返回 `Err`）；
    /// 老硬件没有该能力（能力位 bit0 为 0）时直接拒绝。
    pub fn enable_can_res(&self, channel: u8, state: bool) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_enable_can_res(self.handle, channel, state) })
    }

    /// 读软件终端电阻的当前状态（`dmcan_device_get_can_res`）。
    pub fn can_res(&self, channel: u8) -> Result<bool> {
        let mut enabled = false;
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_can_res(self.handle, channel, &mut enabled) };
        status_ok(ok).map(|()| enabled)
    }

    /// 开关通道的总线 echo（`dmcan_device_set_channel_echo`，真 echo）。
    ///
    /// 打开后固件把真正发上总线的帧按 `dir=SEND, ack=1` 回传，走 `Events::TX_ON_BUS`。
    pub fn set_channel_echo(&self, channel: u8, enable: bool) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_set_channel_echo(self.handle, channel, enable) })
    }

    /// 取 echo 诊断计数（`dmcan_device_get_echo_stats`，只给计数，不产生逐帧事件）。
    pub fn echo_stats(&self) -> Result<EchoStats> {
        let mut raw = ffi::dmcan_echo_stats_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_echo_stats(self.handle, &mut raw) };
        status_ok(ok)?;
        let raw = unsafe { ptr::read_unaligned(&raw) };
        Ok(EchoStats {
            timeout_count: raw.timeout_count,
            late_discarded: raw.late_discarded,
            pending: raw.pending,
            timeout_ms: raw.timeout_ms,
        })
    }

    /// 设置 echo 超时阈值（`dmcan_device_set_echo_timeout_ms`，内部 clamp 到 [10, 60000]）。
    pub fn set_echo_timeout_ms(&self, timeout_ms: u32) {
        // SAFETY: 句柄有效；无返回值。
        unsafe { ffi::dmcan_device_set_echo_timeout_ms(self.handle, timeout_ms) };
    }

    /// 按需读一次指定通道的总线状态（`dmcan_device_get_bus_state`）。
    ///
    /// 与错误记录回调的区别：本方法是主动查询，任何时候都能读，
    /// 适合"等待恢复""确认已退出 bus-off"这类判定。
    pub fn bus_state(&self, channel: u8) -> Result<BusState> {
        let mut raw = ffi::dmcan_bus_state_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_bus_state(self.handle, channel, &mut raw) };
        status_ok(ok)?;
        let raw = unsafe { ptr::read_unaligned(&raw) };
        Ok(BusState {
            bus_state: raw.bus_state,
            tec: raw.tec,
            rec: raw.rec,
            credits: raw.credits,
            in_flight: raw.in_flight,
        })
    }

    /// 读最近一次**周期上报**的通道统计快照（`dmcan_device_get_channel_stats`）。
    ///
    /// **`Ok(None)` 表示"还没收到过该通道的上报"**，与"读到全 0"（确实没丢过）必须分开处理。
    /// 判"现在有没有丢包"要连续读两次看增量，单次绝对值可能是很久以前的累计。
    pub fn channel_stats(&self, channel: u8) -> Result<Option<ChannelStats>> {
        let mut raw = ffi::dmcan_channel_stats_t::default();
        // SAFETY: 输出参数指向本栈帧的局部变量。
        let ok = unsafe { ffi::dmcan_device_get_channel_stats(self.handle, channel, &mut raw) };
        if !ok {
            return match last_status() {
                Some(error) => Err(error),
                None => Ok(None),
            };
        }
        Ok(Some(ChannelStats::from_raw(unsafe {
            ptr::read_unaligned(&raw)
        })))
    }

    /// 主动清一次通道的 bus-off（`dmcan_device_clear_bus_off`）。
    ///
    /// 不在 bus-off 状态时同样回成功（幂等），所以返回 `Ok` 只说明命令被接受；
    /// 要判定恢复请接着读 [`Device::bus_state`]。
    pub fn clear_bus_off(&self, channel: u8) -> Result<()> {
        // SAFETY: 句柄有效。
        status_ok(unsafe { ffi::dmcan_device_clear_bus_off(self.handle, channel) })
    }

    // ------------------------------------------------------------ 内部工具

    /// 调一个"填字符串缓冲"的 C 接口，把 `false` + 错误码为 `DMCAN_OK` 解释成"没有这个值"（`Ok(None)`）。
    ///
    /// C 侧有几处刻意用"错误码保持 OK + 返回 false"表达"现在没有值"，
    /// 例如"设备跑 app，没有 boot 版本串""UART 端口没打开"。
    fn call_optional_string<F>(&self, capacity: usize, call: F) -> Result<Option<String>>
    where
        F: FnOnce(*mut c_char, u32) -> bool,
    {
        let mut buffer = vec![0u8; capacity];
        let ok = call(buffer.as_mut_ptr() as *mut c_char, capacity as u32);
        if !ok {
            return match last_status() {
                Some(error) => Err(error),
                None => Ok(None),
            };
        }
        // SAFETY: C 侧保证内容已截断并以 '\0' 结尾。
        Ok(Some(
            unsafe { CStr::from_ptr(buffer.as_ptr() as *const c_char) }
                .to_string_lossy()
                .into_owned(),
        ))
    }
}

/// 便利转换：从回调里的 [`crate::types::FrameRef`] 直接拿到自有的 `RxFrame`。
impl From<crate::types::FrameRef<'_>> for RxFrame {
    fn from(frame: crate::types::FrameRef<'_>) -> Self {
        frame.to_owned()
    }
}
