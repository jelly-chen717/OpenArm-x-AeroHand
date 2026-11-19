//! 对 C 侧数据类型的 Rust 表达：帧、版本、统计、过滤规则、订阅事件等。
//!
//! 两条约定：
//!
//! - 与 C 结构体一一对应的类型提供 `from_raw` / `to_raw`（crate 内部用），转换一律按值拷贝，
//!   不制造指向 `#[repr(packed)]` 字段的引用；
//! - 回调里给出的事件负载是**借用视图**（`FrameRef`），只在回调期间有效；
//!   要留存请显式调 `to_owned()`。

use core::marker::PhantomData;
use core::ptr;
use std::ffi::CStr;

use crate::error::{Error, Result};
use crate::ffi;

/// "全部通道"掩码，可直接传给 `subscribe()` 的 `channel_mask`。
///
/// 掩码语义是"bit N = 通道 N"，**0 表示什么都没选**（参数错）。
pub const CHANNEL_MASK_ALL: u32 = ffi::DMCAN_CHANNEL_MASK_ALL;

/// 模块能力位 bit0：支持软件开关终端电阻。
pub const MODULE_FEATURE_CAN_RES_SW: u16 = ffi::DMCAN_MODULE_FEATURE_CAN_RES_SW;

/// 固件/设备版本号（`major.minor.build`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Version {
    /// 主版本。
    pub major: u8,
    /// 次版本。
    pub minor: u8,
    /// 构建号。
    pub build: u8,
}

impl Version {
    pub(crate) fn from_raw(raw: ffi::version_t) -> Self {
        Version {
            major: raw.major,
            minor: raw.minor,
            build: raw.build,
        }
    }
}

impl core::fmt::Display for Version {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "v{}.{}.{}", self.major, self.minor, self.build)
    }
}

/// 设备型号。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DeviceType {
    /// USB2CANFD_S（单路，1 通道）。
    Usb2CanfdS,
    /// USB2CANFD_D（双路，2 通道）。
    Usb2CanfdD,
    /// LinkX4C（四路，4 通道）。
    LinkX4C,
    /// 未知型号。
    Unknown,
}

impl DeviceType {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match raw {
            ffi::DMCAN_DEVICE_USB2CANFD_S => DeviceType::Usb2CanfdS,
            ffi::DMCAN_DEVICE_USB2CANFD_D => DeviceType::Usb2CanfdD,
            ffi::DMCAN_DEVICE_LINKX4C => DeviceType::LinkX4C,
            _ => DeviceType::Unknown,
        }
    }

    pub(crate) fn to_raw(self) -> i32 {
        match self {
            DeviceType::Usb2CanfdS => ffi::DMCAN_DEVICE_USB2CANFD_S,
            DeviceType::Usb2CanfdD => ffi::DMCAN_DEVICE_USB2CANFD_D,
            DeviceType::LinkX4C => ffi::DMCAN_DEVICE_LINKX4C,
            DeviceType::Unknown => ffi::DMCAN_DEVICE_UNKNOWN,
        }
    }
}

/// 设备固件信息（`dmcan_device_get_firmware_info`）：型号 + 发布版本号 + 构建标识。
///
/// 两个部分各自带"这次读到了没有"，**不要把没读到的零值当成读数**：
///
/// - `version_valid == false`：没拿到 app 的发布版本号（此时 `version` 恒 `0.0.0`）。
///   **设备跑在 boot 时就是这种情况**，所以判"板子还停在 boot"看这个字段；
/// - `build_id == None`：固件没在版本应答里带构建标识（老固件），不算失败。
///
/// **判"升级是否生效"只能看 `build_id`**：发布版本号在连续几版之间常常不变。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FirmwareInfo {
    /// 设备型号（取自枚举表，设备在位就有效）。
    pub device: DeviceType,
    /// 发布版本号；`version_valid` 为假时恒 `0.0.0`。
    pub version: Version,
    /// 版本号这次读到了没有。
    pub version_valid: bool,
    /// 构建标识（短 hash + 编译时间）；固件没提供时为 `None`
    /// （对应 C 侧 `build_id_valid == false`）。
    pub build_id: Option<String>,
}

impl FirmwareInfo {
    pub(crate) fn from_raw(raw: ffi::dmcan_firmware_info_t) -> Self {
        let build_id = if raw.build_id_valid {
            // SAFETY: C 侧保证是 NUL 结尾的 ASCII；这里立刻拷成 String。
            let text = unsafe { CStr::from_ptr(raw.build_id.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            if text.is_empty() {
                None
            } else {
                Some(text)
            }
        } else {
            None
        };
        FirmwareInfo {
            device: DeviceType::from_raw(raw.device),
            version: Version::from_raw(raw.version),
            version_valid: raw.version_valid,
            build_id,
        }
    }

    /// 格式化成"人读的一行"（对应 `dmcan_firmware_info_to_string()`，**不访问设备**）。
    ///
    /// 文本由 C 侧生成（2.1.0 起是英文），形如
    /// `USB2CANFD_D V2.0.1 [4bb41da 2026-09-24 19:25:07]`；没有构建标识时只输出型号与版本号；
    /// 只有标识、没读到版本号时输出 `... version not read (device may be stuck in boot) [build id]`。
    pub fn to_string_line(&self) -> Result<String> {
        // 把 Rust 侧的字段重新编码回 C 结构体（C 侧只读它，不改）。
        let mut build_id = [0 as core::ffi::c_char; 40];
        if let Some(id) = &self.build_id {
            let bytes = id.as_bytes();
            let len = bytes.len().min(build_id.len() - 1);
            for (slot, byte) in build_id.iter_mut().zip(&bytes[..len]) {
                *slot = *byte as core::ffi::c_char;
            }
        }
        let raw = ffi::dmcan_firmware_info_t {
            device: self.device.to_raw(),
            version: ffi::version_t {
                major: self.version.major,
                minor: self.version.minor,
                build: self.version.build,
            },
            version_valid: self.version_valid,
            build_id,
            build_id_valid: self.build_id.is_some(),
        };
        let mut buffer = vec![0u8; 192];
        // SAFETY: `raw` 与 `buffer` 都在本次调用期间存活；C 侧只读入参、只写缓冲。
        let written = unsafe {
            ffi::dmcan_firmware_info_to_string(
                &raw,
                buffer.as_mut_ptr() as *mut core::ffi::c_char,
                buffer.len() as u32,
            )
        };
        if written == 0 {
            return Err(Error::InvalidArg);
        }
        // SAFETY: C 侧保证内容以 '\0' 结尾。
        Ok(
            unsafe { CStr::from_ptr(buffer.as_ptr() as *const core::ffi::c_char) }
                .to_string_lossy()
                .into_owned(),
        )
    }
}

/// 设备的枚举信息（不需要设备处于 open 状态就能读）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    /// 设备型号。
    pub device: DeviceType,
    /// USB PID。
    pub pid: i32,
    /// USB VID。
    pub vid: i32,
    /// 序列号；设备没给出时为 `None`。
    ///
    /// 注意：C 侧该字段指向库内部缓冲，本 crate 已经把它拷成 `String`，
    /// 可以随意保存、跨线程传递。
    pub serial: Option<String>,
}

/// 一帧待发送的报文。
///
/// 字段是公开的（"薄"），因此也可以用结构体字面量直接构造；但用构造函数能提前拦住
/// "经典帧塞 > 8 字节 / 任意帧塞 > 64 字节"这类 C 侧必拒的组合。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// 帧 ID。
    pub can_id: u32,
    /// 是否 CAN FD。
    pub canfd: bool,
    /// 是否扩展帧（29 位 ID）。
    pub ext: bool,
    /// 是否远程帧。
    pub rtr: bool,
    /// 是否启用 BRS（仅 CAN FD 有意义）。
    pub brs: bool,
    /// 数据长度（字节）；C 侧会自动向上取整到最近的 DLC。
    pub dlen: u8,
    /// 数据；`dlen` 之外的字节被忽略。
    pub payload: [u8; 64],
}

impl Frame {
    /// 经典 CAN 帧（数据 ≤ 8 字节）。
    pub fn classic(can_id: u32, data: &[u8]) -> Result<Self> {
        Self::build(can_id, false, false, false, data)
    }

    /// CAN FD 帧（数据 ≤ 64 字节）。
    pub fn fd(can_id: u32, data: &[u8], brs: bool) -> Result<Self> {
        Self::build(can_id, true, false, brs, data)
    }

    /// 扩展帧构造：`canfd` / `rtr` / `brs` 由调用方指定。
    pub fn extended(can_id: u32, canfd: bool, rtr: bool, brs: bool, data: &[u8]) -> Result<Self> {
        Self::build(can_id, canfd, rtr, brs, data)
    }

    fn build(can_id: u32, canfd: bool, rtr: bool, brs: bool, data: &[u8]) -> Result<Self> {
        if data.len() > 64 || (!canfd && data.len() > 8) {
            return Err(Error::InvalidArg);
        }
        let mut payload = [0u8; 64];
        payload[..data.len()].copy_from_slice(data);
        Ok(Frame {
            can_id,
            canfd,
            ext: false,
            rtr,
            brs,
            dlen: data.len() as u8,
            payload,
        })
    }

    /// 标成扩展帧。
    pub fn with_extended(mut self, ext: bool) -> Self {
        self.ext = ext;
        self
    }

    /// 有效数据切片。
    pub fn data(&self) -> &[u8] {
        &self.payload[..usize::from(self.dlen).min(64)]
    }

    pub(crate) fn to_raw(self) -> ffi::dmcan_frame_t {
        ffi::dmcan_frame_t {
            can_id: self.can_id,
            canfd: self.canfd,
            ext: self.ext,
            rtr: self.rtr,
            brs: self.brs,
            dlen: self.dlen,
            payload: self.payload,
        }
    }

    /// 把帧交给 C 侧发送接口用的参数（payload 指针指向 `self`，所以 `self` 必须活到调用结束）。
    pub(crate) fn send_args(&self) -> (u32, bool, bool, bool, bool, u8, *mut u8) {
        (
            self.can_id,
            self.canfd,
            self.ext,
            self.rtr,
            self.brs,
            self.dlen,
            self.payload.as_ptr() as *mut u8,
        )
    }
}

/// 收到的帧（自有数据；从回调里的 `FrameRef` 拷贝而来）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxFrame {
    /// 帧 ID。
    pub can_id: u32,
    /// 时间戳（µs）。设备时间戳与上位机时间戳时基不同，不能相减。
    pub timestamp_us: u64,
    /// 通道号。
    pub channel: u8,
    /// 是否扩展帧。
    pub ext: bool,
    /// 是否远程帧。
    pub rtr: bool,
    /// 是否 CAN FD。
    pub canfd: bool,
    /// 是否 BRS。
    pub brs: bool,
    /// 错误状态指示（ESI）。
    pub esi: bool,
    /// ack = 0 假 echo / 1 真 echo。
    pub ack: bool,
    /// 数据长度。
    pub dlen: u8,
    /// 数据；`dlen` 之外的字节无意义。
    pub payload: [u8; 64],
}

impl RxFrame {
    /// 有效数据切片。
    pub fn data(&self) -> &[u8] {
        &self.payload[..usize::from(self.dlen).min(64)]
    }
}

/// 帧的**借用视图**：指向 SDK 内部缓冲，只在本次回调内有效。
#[derive(Clone, Copy)]
pub struct FrameRef<'a> {
    raw: *const ffi::dmcan_rx_frame_t,
    _marker: PhantomData<&'a ffi::dmcan_rx_frame_t>,
}

impl core::fmt::Debug for FrameRef<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FrameRef")
            .field("can_id", &self.can_id())
            .field("channel", &self.channel())
            .field("dlen", &self.dlen())
            .field("canfd", &self.is_canfd())
            .field("extended", &self.is_extended())
            .field("ack", &self.ack())
            .finish()
    }
}

impl<'a> FrameRef<'a> {
    /// # Safety
    ///
    /// `raw` 必须指向 SDK 在**本次回调期间**有效的一帧，且调用方给出的 `'a` 不得超出该回调。
    pub(crate) unsafe fn from_raw(raw: *const ffi::dmcan_rx_frame_t) -> Self {
        FrameRef {
            raw,
            _marker: PhantomData,
        }
    }

    /// 按值读整个 C 结构体（避免对 `#[repr(packed)]` 字段取引用）。
    fn read(&self) -> ffi::dmcan_rx_frame_t {
        // SAFETY: `raw` 由订阅回调保证在本次调用期间有效，且结构体是 POD。
        unsafe { ptr::read_unaligned(self.raw) }
    }

    /// 帧 ID。
    pub fn can_id(&self) -> u32 {
        self.read().can_id
    }

    /// 时间戳（µs）。
    pub fn timestamp_us(&self) -> u64 {
        self.read().timestamp
    }

    /// 通道号。
    pub fn channel(&self) -> u8 {
        self.read().channel
    }

    /// 是否扩展帧。
    pub fn is_extended(&self) -> bool {
        self.read().ext != 0
    }

    /// 是否远程帧。
    pub fn is_remote(&self) -> bool {
        self.read().rtr != 0
    }

    /// 是否 CAN FD。
    pub fn is_canfd(&self) -> bool {
        self.read().canfd != 0
    }

    /// 是否 BRS。
    pub fn is_brs(&self) -> bool {
        self.read().brs != 0
    }

    /// 错误状态指示（ESI）。
    pub fn is_esi(&self) -> bool {
        self.read().esi != 0
    }

    /// ack = 0 假 echo / 1 真 echo。
    pub fn ack(&self) -> bool {
        self.read().ack != 0
    }

    /// 数据长度。
    pub fn dlen(&self) -> u8 {
        self.read().dlen
    }

    /// 数据切片（长度按 `dlen` 截断，至多 64 字节）。
    pub fn data(&self) -> &'a [u8] {
        let len = usize::from(self.read().dlen).min(64);
        // SAFETY: payload 在 `dmcan_rx_frame_t` 内的偏移固定（有 ABI 断言钉住），
        // 且指针仍指向本次回调有效的 SDK 缓冲；不构造指向 packed 字段的引用。
        let base = unsafe {
            (self.raw as *const u8).add(core::mem::offset_of!(ffi::dmcan_rx_frame_t, payload))
        };
        unsafe { core::slice::from_raw_parts(base, len) }
    }

    /// 拷贝成自有的 `RxFrame`（要留存到回调之外时必须这么做）。
    pub fn to_owned(&self) -> RxFrame {
        let raw = self.read();
        RxFrame {
            can_id: raw.can_id,
            timestamp_us: raw.timestamp,
            channel: raw.channel,
            ext: raw.ext != 0,
            rtr: raw.rtr != 0,
            canfd: raw.canfd != 0,
            brs: raw.brs != 0,
            esi: raw.esi != 0,
            ack: raw.ack != 0,
            dlen: raw.dlen,
            payload: raw.payload,
        }
    }
}

/// 通道错误记录（自有数据；错误事件稀有，直接拷贝）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ChannelError {
    /// 设备时间戳（µs）。
    pub timestamp_us: u64,
    /// 错误位图。
    pub error_code: u16,
    /// 0 = active / 1 = warning / 2 = passive / 3 = bus-off。
    pub bus_state: u8,
    /// 最近一次错误码（总线相位）。
    pub lec: u8,
    /// 数据相位错误码。
    pub dlec: u8,
    /// 发送错误计数。
    pub tec: u8,
    /// 接收错误计数。
    pub rec: u8,
    /// 错误原因位。
    pub flags: u8,
}

impl ChannelError {
    /// # Safety
    ///
    /// `raw` 必须指向本次回调期间有效的一条错误记录。
    pub(crate) unsafe fn from_raw(raw: *const ffi::dmcan_channel_error_t) -> Self {
        let raw = ptr::read_unaligned(raw);
        ChannelError {
            timestamp_us: raw.timestamp_us,
            error_code: raw.error_code,
            bus_state: raw.bus_state,
            lec: raw.lec,
            dlec: raw.dlec,
            tec: raw.tec,
            rec: raw.rec,
            flags: raw.flags,
        }
    }
}

/// 订阅事件（回调参数）。
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum Event<'a> {
    /// 总线收到的帧。
    RxFrame(FrameRef<'a>),
    /// 假 echo：这笔 USB 提交已完成（还没证明上了总线）。
    TxCompleted(FrameRef<'a>),
    /// 真 echo：固件回报该帧已在总线上被确认。
    TxOnBus(FrameRef<'a>),
    /// 通道错误记录。
    ChannelError(ChannelError),
    /// USB2UART 收到的数据（一次回调 = 操作系统这次给到的一段连续字节，边界不是协议帧）。
    UartRx(&'a [u8]),
}

/// 订阅事件位图（`subscribe()` 的第一个参数）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Events(u32);

impl Events {
    /// 总线收到的帧。
    pub const RX_FRAME: Events = Events(ffi::DMCAN_EVENT_RX_FRAME);
    /// 假 echo。
    pub const TX_COMPLETED: Events = Events(ffi::DMCAN_EVENT_TX_COMPLETED);
    /// 真 echo。
    pub const TX_ON_BUS: Events = Events(ffi::DMCAN_EVENT_TX_ON_BUS);
    /// 通道错误记录。
    pub const CHANNEL_ERROR: Events = Events(ffi::DMCAN_EVENT_CHANNEL_ERROR);
    /// USB2UART 收到数据。
    pub const UART_RX: Events = Events(ffi::DMCAN_EVENT_UART_RX);

    /// 位图原值。
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// 是否没有选择任何事件（C 侧判为参数错）。
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl core::ops::BitOr for Events {
    type Output = Events;

    fn bitor(self, rhs: Events) -> Events {
        Events(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for Events {
    fn bitor_assign(&mut self, rhs: Events) {
        self.0 |= rhs.0;
    }
}

/// 发送聚合档位（设备级，默认吞吐优先）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SendProfile {
    /// 吞吐优先（默认）。
    Throughput,
    /// 延迟优先：攒批更小、帧到得更快，设备侧压力上升。
    LowLatency,
}

impl SendProfile {
    pub(crate) fn from_raw(raw: i32) -> Self {
        match raw {
            ffi::DMCAN_SEND_PROFILE_LOW_LATENCY => SendProfile::LowLatency,
            _ => SendProfile::Throughput,
        }
    }

    pub(crate) fn to_raw(self) -> i32 {
        match self {
            SendProfile::Throughput => ffi::DMCAN_SEND_PROFILE_THROUGHPUT,
            SendProfile::LowLatency => ffi::DMCAN_SEND_PROFILE_LOW_LATENCY,
        }
    }
}

/// 通道波特率配置（简化形态，`set_channel_baudrate` 用）。
///
/// TDC 不在这里：CAN FD 且数据段波特率 > 1 Mbps 时由 SDK 自动打开，
/// 要看当前生效值请用 `Device::channel_tdc()`。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChannelBaudInfo {
    /// 是否启用 CAN FD。
    pub canfd: bool,
    /// 是否非 ISO（CAN FD 非 ISO 模式）。
    pub non_iso: bool,
    /// 是否自动重传。
    pub retrans: bool,
    /// 仲裁段波特率。
    pub can_baudrate: u32,
    /// 数据段波特率。
    pub canfd_baudrate: u32,
    /// 仲裁段采样点（0~1）。
    pub can_sp: f32,
    /// 数据段采样点（0~1）。
    pub canfd_sp: f32,
}

impl ChannelBaudInfo {
    pub(crate) fn to_raw(self) -> ffi::dmcan_channel_can_info_t {
        ffi::dmcan_channel_can_info_t {
            canfd: self.canfd,
            non_iso: self.non_iso,
            retrans: self.retrans,
            can_baudrate: self.can_baudrate,
            canfd_baudrate: self.canfd_baudrate,
            can_sp: self.can_sp,
            canfd_sp: self.canfd_sp,
        }
    }

    pub(crate) fn from_raw(raw: ffi::dmcan_channel_can_info_t) -> Self {
        ChannelBaudInfo {
            canfd: raw.canfd,
            non_iso: raw.non_iso,
            retrans: raw.retrans,
            can_baudrate: raw.can_baudrate,
            canfd_baudrate: raw.canfd_baudrate,
            can_sp: raw.can_sp,
            canfd_sp: raw.canfd_sp,
        }
    }
}

/// 通道的 MCAN 节点模式（`dmcan_ch_baud_conf_t::mode` 的取值）。
///
/// 与固件/HPM 驱动的 `mcan_node_mode_t` 同口径（SDK 只透传）。外部回环**不依赖外部 ACK**，
/// 单个模块就能自发自收（适合自测与性能基线）；但帧确实会上总线，别和正常对端混用。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelMode {
    /// 正常收发（默认）。
    Normal,
    /// 内部回环：TX 不外发，帧在芯片内回到接收路径。
    LoopbackInternal,
    /// 外部回环：帧照常驱动总线并回环到自己；不依赖外部 ACK。
    LoopbackExternal,
    /// 只听不发（不驱动总线、不回 ACK）。
    ListenOnly,
    /// 固件回了本绑定未定义的模式值（原值带出来便于诊断）。
    Undefined(u8),
}

impl ChannelMode {
    pub(crate) fn from_raw(raw: u8) -> Self {
        match raw {
            ffi::DMCAN_CHANNEL_MODE_NORMAL => ChannelMode::Normal,
            ffi::DMCAN_CHANNEL_MODE_LOOPBACK_INTERNAL => ChannelMode::LoopbackInternal,
            ffi::DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL => ChannelMode::LoopbackExternal,
            ffi::DMCAN_CHANNEL_MODE_LISTEN_ONLY => ChannelMode::ListenOnly,
            other => ChannelMode::Undefined(other & 0x0F),
        }
    }

    pub(crate) fn to_raw(self) -> u8 {
        match self {
            ChannelMode::Normal => ffi::DMCAN_CHANNEL_MODE_NORMAL,
            ChannelMode::LoopbackInternal => ffi::DMCAN_CHANNEL_MODE_LOOPBACK_INTERNAL,
            ChannelMode::LoopbackExternal => ffi::DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL,
            ChannelMode::ListenOnly => ffi::DMCAN_CHANNEL_MODE_LISTEN_ONLY,
            ChannelMode::Undefined(value) => value & 0x0F,
        }
    }
}

/// 通道完整下发结构（`set_channel_baudrate_details` 用），字段逐一对应固件布局。
///
/// 首字节在 C 侧是位域，Rust 用位访问器表达：`enable_canfd` / `enable_tdc` / `non_iso` /
/// `enable_retrans` 各占 1 位，`mode` 占高 4 位。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelBaudConf {
    /// 仲裁段 BRP。
    pub nominal_brp: u16,
    /// 仲裁段 TSEG1。
    pub nominal_seg1: u16,
    /// 仲裁段 TSEG2。
    pub nominal_seg2: u16,
    /// 仲裁段 SJW。
    pub nominal_sjw: u16,
    /// 数据段 BRP。
    pub data_brp: u16,
    /// 数据段 TSEG1。
    pub data_seg1: u16,
    /// 数据段 TSEG2。
    pub data_seg2: u16,
    /// 数据段 SJW。
    pub data_sjw: u16,
    /// bus-off 后自动恢复间隔（毫秒）；0 = 不自动恢复。
    pub busoff_autoreset_ms: u16,
    /// 二次采样点偏移（数据段 mtq）；0 = 自动。
    pub tdc_offset: u8,
    /// TDC 滤波窗口（数据段 mtq）；0 = 跟随 `tdc_offset`。
    pub tdc_filter: u8,

    /// C 侧位域字节原值；只通过下面那组访问器读写。
    flags: u8,
}

impl ChannelBaudConf {
    const BIT_CANFD: u8 = 1 << 0;
    const BIT_TDC: u8 = 1 << 1;
    const BIT_NON_ISO: u8 = 1 << 2;
    const BIT_RETRANS: u8 = 1 << 3;

    fn bit(&self, mask: u8) -> bool {
        self.flags & mask != 0
    }

    fn set_bit(&mut self, mask: u8, value: bool) {
        if value {
            self.flags |= mask;
        } else {
            self.flags &= !mask;
        }
    }

    /// 是否启用 CAN FD。
    pub fn enable_canfd(&self) -> bool {
        self.bit(Self::BIT_CANFD)
    }

    /// 设置是否启用 CAN FD。
    pub fn set_enable_canfd(&mut self, value: bool) {
        self.set_bit(Self::BIT_CANFD, value);
    }

    /// 是否启用 TDC。
    pub fn enable_tdc(&self) -> bool {
        self.bit(Self::BIT_TDC)
    }

    /// 设置是否启用 TDC。
    pub fn set_enable_tdc(&mut self, value: bool) {
        self.set_bit(Self::BIT_TDC, value);
    }

    /// 是否非 ISO（CAN FD 非 ISO 模式）。
    pub fn non_iso(&self) -> bool {
        self.bit(Self::BIT_NON_ISO)
    }

    /// 设置是否非 ISO。
    pub fn set_non_iso(&mut self, value: bool) {
        self.set_bit(Self::BIT_NON_ISO, value);
    }

    /// 是否自动重传。
    pub fn enable_retrans(&self) -> bool {
        self.bit(Self::BIT_RETRANS)
    }

    /// 设置是否自动重传。
    pub fn set_enable_retrans(&mut self, value: bool) {
        self.set_bit(Self::BIT_RETRANS, value);
    }

    /// 节点模式（高 4 位）。
    pub fn mode(&self) -> ChannelMode {
        ChannelMode::from_raw(self.flags >> 4)
    }

    /// 设置节点模式（只取低 4 位）。
    pub fn set_mode(&mut self, mode: ChannelMode) {
        self.flags = (self.flags & 0x0F) | (mode.to_raw() & 0x0F) << 4;
    }

    /// 位域字节原值（诊断与测试用）。
    pub fn raw_flags(&self) -> u8 {
        self.flags
    }

    pub(crate) fn to_raw(self) -> ffi::dmcan_ch_baud_conf_t {
        ffi::dmcan_ch_baud_conf_t {
            flags: self.flags,
            nominal_brp: self.nominal_brp,
            nominal_seg1: self.nominal_seg1,
            nominal_seg2: self.nominal_seg2,
            nominal_sjw: self.nominal_sjw,
            data_brp: self.data_brp,
            data_seg1: self.data_seg1,
            data_seg2: self.data_seg2,
            data_sjw: self.data_sjw,
            busoff_autoreset_ms: self.busoff_autoreset_ms,
            tdc_offset: self.tdc_offset,
            tdc_filter: self.tdc_filter,
            reserved: 0,
        }
    }

    pub(crate) fn from_raw(raw: ffi::dmcan_ch_baud_conf_t) -> Self {
        ChannelBaudConf {
            nominal_brp: raw.nominal_brp,
            nominal_seg1: raw.nominal_seg1,
            nominal_seg2: raw.nominal_seg2,
            nominal_sjw: raw.nominal_sjw,
            data_brp: raw.data_brp,
            data_seg1: raw.data_seg1,
            data_seg2: raw.data_seg2,
            data_sjw: raw.data_sjw,
            busoff_autoreset_ms: raw.busoff_autoreset_ms,
            tdc_offset: raw.tdc_offset,
            tdc_filter: raw.tdc_filter,
            flags: raw.flags,
        }
    }
}

/// 通道 TDC 的硬件实际生效值。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TdcStatus {
    /// 是否读到硬件实际值；false 表示"没读到"（老固件 / 未启用 TDC），而不是"生效值是 0"。
    pub valid: bool,
    /// 硬件生效的二次采样点偏移。
    pub tdco: u8,
    /// 硬件生效的滤波窗口。
    pub tdcf: u8,
    /// 硬件实测收发器延迟；只有成功发送过数据帧后才非 0。
    pub tdcv: u8,
}

/// 通道扩展配置：重传策略与 bus-off 自动恢复间隔。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelExtConfig {
    /// 1 = 自动重传（默认）；0 = 一次性发送。
    pub enable_retrans: bool,
    /// bus-off 后自动恢复间隔（毫秒）；0 = 不自动恢复。
    pub busoff_autoreset_ms: u16,
}

impl ChannelExtConfig {
    pub(crate) fn to_raw(self) -> ffi::dmcan_channel_ext_config_t {
        ffi::dmcan_channel_ext_config_t {
            enable_retrans: u8::from(self.enable_retrans),
            busoff_autoreset_ms: self.busoff_autoreset_ms,
        }
    }

    pub(crate) fn from_raw(raw: ffi::dmcan_channel_ext_config_t) -> Self {
        ChannelExtConfig {
            enable_retrans: raw.enable_retrans != 0,
            busoff_autoreset_ms: raw.busoff_autoreset_ms,
        }
    }
}

/// 过滤表的匹配模式。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterMode {
    /// 白名单：未命中任何规则 → 拒收（空表 = 拒收全部）。
    Whitelist,
    /// 黑名单：未命中 → 正常接收（出厂默认是黑名单 + 空表 = 全接受）。
    Blacklist,
}

impl FilterMode {
    pub(crate) fn from_raw(raw: u8) -> Self {
        match raw {
            ffi::DMCAN_FILTER_MODE_WHITELIST => FilterMode::Whitelist,
            _ => FilterMode::Blacklist,
        }
    }

    pub(crate) fn to_raw(self) -> u8 {
        match self {
            FilterMode::Whitelist => ffi::DMCAN_FILTER_MODE_WHITELIST,
            FilterMode::Blacklist => ffi::DMCAN_FILTER_MODE_BLACKLIST,
        }
    }
}

/// 过滤规则的 ID 类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterIdType {
    /// 标准帧（11 位 ID）。
    Standard,
    /// 扩展帧（29 位 ID）。
    Extended,
}

impl FilterIdType {
    pub(crate) fn to_raw(self) -> u8 {
        match self {
            FilterIdType::Standard => ffi::DMCAN_FILTER_ID_STD,
            FilterIdType::Extended => ffi::DMCAN_FILTER_ID_EXT,
        }
    }

    pub(crate) fn from_raw(raw: u8) -> Self {
        match raw {
            ffi::DMCAN_FILTER_ID_EXT => FilterIdType::Extended,
            _ => FilterIdType::Standard,
        }
    }
}

/// 过滤规则的匹配类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterRuleType {
    /// 范围：`id1 <= rx_id <= id2`。
    Range,
    /// 双 ID 精确：`rx_id == id1` 或 `rx_id == id2`。
    DualId,
    /// 掩码：`(rx_id & id2) == (id1 & id2)`。
    Mask,
}

impl FilterRuleType {
    pub(crate) fn to_raw(self) -> u8 {
        match self {
            FilterRuleType::Range => ffi::DMCAN_FILTER_TYPE_RANGE,
            FilterRuleType::DualId => ffi::DMCAN_FILTER_TYPE_DUAL_ID,
            FilterRuleType::Mask => ffi::DMCAN_FILTER_TYPE_MASK,
        }
    }

    pub(crate) fn from_raw(raw: u8) -> Self {
        match raw {
            ffi::DMCAN_FILTER_TYPE_DUAL_ID => FilterRuleType::DualId,
            ffi::DMCAN_FILTER_TYPE_MASK => FilterRuleType::Mask,
            _ => FilterRuleType::Range,
        }
    }
}

/// 过滤规则的命中动作。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterAction {
    /// 存 RX FIFO0（正常接收）。
    Fifo0,
    /// 拒收。
    Reject,
    /// 置优先级并存 RX FIFO0。
    Priority,
}

impl FilterAction {
    pub(crate) fn to_raw(self) -> u8 {
        match self {
            FilterAction::Fifo0 => ffi::DMCAN_FILTER_ACTION_FIFO0,
            FilterAction::Reject => ffi::DMCAN_FILTER_ACTION_REJECT,
            FilterAction::Priority => ffi::DMCAN_FILTER_ACTION_PRIORITY,
        }
    }

    pub(crate) fn from_raw(raw: u8) -> Self {
        match raw {
            ffi::DMCAN_FILTER_ACTION_REJECT => FilterAction::Reject,
            ffi::DMCAN_FILTER_ACTION_PRIORITY => FilterAction::Priority,
            _ => FilterAction::Fifo0,
        }
    }
}

/// 单条硬件过滤规则。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FilterRule {
    /// ID 类型（标准 / 扩展）。
    pub id_type: FilterIdType,
    /// 匹配类型。
    pub rule_type: FilterRuleType,
    /// 命中动作。
    pub action: FilterAction,
    /// 含义随 `rule_type` 变化；标准帧 ≤ `0x7FF`，扩展帧 ≤ `0x1FFF_FFFF`。
    pub id1: u32,
    /// 含义随 `rule_type` 变化。
    pub id2: u32,
}

impl FilterRule {
    pub(crate) fn to_raw(self) -> ffi::dmcan_filter_rule_t {
        ffi::dmcan_filter_rule_t {
            id_type: self.id_type.to_raw(),
            rule_type: self.rule_type.to_raw(),
            action: self.action.to_raw(),
            reserved: 0,
            id1: self.id1,
            id2: self.id2,
        }
    }

    pub(crate) fn from_raw(raw: ffi::dmcan_filter_rule_t) -> Self {
        FilterRule {
            id_type: FilterIdType::from_raw(raw.id_type),
            rule_type: FilterRuleType::from_raw(raw.rule_type),
            action: FilterAction::from_raw(raw.action),
            id1: raw.id1,
            id2: raw.id2,
        }
    }
}

/// 过滤表下发后固件回的用量（**受理结果**，不等于已生效）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FilterUsage {
    /// 表里标准 ID 规则的条数。
    pub std_used: u8,
    /// 表里扩展 ID 规则的条数。
    pub ext_used: u8,
}

/// 过滤表回读结果。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FilterTable {
    /// 当前模式。
    pub mode: FilterMode,
    /// 固件支持的标准 ID 规则上限。
    pub std_max: u8,
    /// 固件支持的扩展 ID 规则上限。
    pub ext_max: u8,
    /// 表里的实际规则（长度 = 固件回的 `count`）。
    pub rules: Vec<FilterRule>,
}

/// 通道总线状态快照（主动查询，不含历史）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusState {
    /// 0 = error active / 1 = error warning / 2 = error passive / 3 = bus-off。
    pub bus_state: u8,
    /// 发送错误计数。
    pub tec: u8,
    /// 接收错误计数。
    pub rec: u8,
    /// 当前可用发送额度。
    pub credits: u8,
    /// 在途未确认的帧数。
    pub in_flight: u8,
}

/// 通道统计快照（来自固件每秒一次的周期上报，最多旧 1 秒）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelStats {
    /// 受理进发送队列的帧数（含后续被丢弃/失败的）。
    pub tx_admitted: u32,
    /// 真正发送完成的帧数。
    pub tx_completed: u32,
    /// 因队列或额度不足丢弃的帧数。
    pub tx_dropped: u32,
    /// 发送尝试已结束但没上总线的帧数。
    pub tx_failed: u32,
    /// 收到的帧数。
    pub rx_received: u32,
    /// 接收侧丢弃的帧数。
    pub rx_dropped: u32,
    /// 软件发送队列当前深度。
    pub sw_tx_fifo_level: u16,
    /// 硬件发送 FIFO 当前深度。
    pub hw_tx_fifo_level: u16,
    /// 该通道当前是否使能。
    pub enabled: bool,
    /// 丢包细分有效位（见 `DROP_FLAG_*`）；位为 0 时对应字段恒 0，不代表"没丢过"。
    pub drop_flags: u8,
    /// 接收队列满丢弃的帧数（需 `DROP_FLAG_RX_ECHO` 置位才有效）。
    pub rx_dropped_queue: u32,
    /// 接收帧写不进 msgc 丢弃的帧数。
    pub rx_dropped_msgc: u32,
    /// echo 待回传队列满丢弃的条数。
    pub echo_dropped_queue: u32,
    /// echo 写不进 msgc 丢弃的条数。
    pub echo_dropped_msgc: u32,
    /// 已写入 msgc 的 echo 条数。
    pub echo_sent: u32,
    /// 到达时 credit 耗尽，本帧不受理（需 `DROP_FLAG_TX` 置位才有效）。
    pub tx_dropped_no_credit: u32,
    /// 到达时软件发送队列满，本帧不受理。
    pub tx_dropped_queue_full: u32,
    /// 提交 MCAN 发送 FIFO 失败。
    pub tx_dropped_submit_fail: u32,
    /// 通道已关闭。
    pub tx_dropped_ch_disabled: u32,
}

/// 丢包细分有效位 bit0：接收/echo 侧细分已收到。
pub const DROP_FLAG_RX_ECHO: u8 = ffi::DMCAN_DROP_FLAG_RX_ECHO;
/// 丢包细分有效位 bit1：发送侧细分已收到。
pub const DROP_FLAG_TX: u8 = ffi::DMCAN_DROP_FLAG_TX;

impl ChannelStats {
    pub(crate) fn from_raw(raw: ffi::dmcan_channel_stats_t) -> Self {
        ChannelStats {
            tx_admitted: raw.tx_admitted,
            tx_completed: raw.tx_completed,
            tx_dropped: raw.tx_dropped,
            tx_failed: raw.tx_failed,
            rx_received: raw.rx_received,
            rx_dropped: raw.rx_dropped,
            sw_tx_fifo_level: raw.sw_tx_fifo_level,
            hw_tx_fifo_level: raw.hw_tx_fifo_level,
            enabled: raw.enabled != 0,
            drop_flags: raw.drop_flags,
            rx_dropped_queue: raw.rx_dropped_queue,
            rx_dropped_msgc: raw.rx_dropped_msgc,
            echo_dropped_queue: raw.echo_dropped_queue,
            echo_dropped_msgc: raw.echo_dropped_msgc,
            echo_sent: raw.echo_sent,
            tx_dropped_no_credit: raw.tx_dropped_no_credit,
            tx_dropped_queue_full: raw.tx_dropped_queue_full,
            tx_dropped_submit_fail: raw.tx_dropped_submit_fail,
            tx_dropped_ch_disabled: raw.tx_dropped_ch_disabled,
        }
    }
}

/// echo 诊断计数（只给计数，不产生逐帧事件）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EchoStats {
    /// 超过阈值仍未收到总线确认的帧数（累计）。
    pub timeout_count: u64,
    /// 超时后才回来、按迟到丢弃的条数（累计）。
    pub late_discarded: u64,
    /// 当前仍在等总线确认的帧数。
    pub pending: u32,
    /// 当前超时阈值（毫秒）。
    pub timeout_ms: u32,
}

/// UART 线参数（对应 CDC 的 line coding）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UartConfig {
    /// 波特率；**0 表示不改线参数**（沿用设备当前配置）。
    pub baud_rate: u32,
    /// 数据位：5~8（默认 8）。
    pub data_bits: u8,
    /// 校验：0 = 无 / 1 = 奇 / 2 = 偶（默认 0）。
    pub parity: u8,
    /// 停止位：1 或 2（默认 1）。
    pub stop_bits: u8,
}

impl Default for UartConfig {
    fn default() -> Self {
        UartConfig {
            baud_rate: 115_200,
            data_bits: 8,
            parity: 0,
            stop_bits: 1,
        }
    }
}

impl UartConfig {
    pub(crate) fn to_raw(self) -> ffi::dmcan_uart_config_t {
        ffi::dmcan_uart_config_t {
            baud_rate: self.baud_rate,
            data_bits: self.data_bits,
            parity: self.parity,
            stop_bits: self.stop_bits,
            reserved: 0,
        }
    }
}

/// UART 统计。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UartStats {
    /// 已投递给用户的字节数。
    pub rx_bytes: u64,
    /// 已写入端口的字节数。
    pub tx_bytes: u64,
    /// 用户回调来不及/队列满而丢掉的字节数。
    pub rx_dropped: u64,
    /// 串口层错误次数（含端口掉线）。
    pub errors: u32,
    /// 端口当前是否打开。
    pub is_open: bool,
}

/// 批量/递增发送配置。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BulkSendConfig<'a> {
    /// 发送间隔（毫秒）；0 = 只发一帧。
    pub interval_ms: u32,
    /// 发送次数；0 = 不限（此时由 `stop_id` 决定何时停）。
    pub send_times: u32,
    /// 起始 ID。
    pub can_id: u32,
    /// 是否递增 ID。
    pub id_inc: bool,
    /// ID 步进；仅在 `id_inc` 时有效。
    pub step_id: u16,
    /// 停止 ID（不含）；仅在 `id_inc` 时有效。
    pub stop_id: u32,
    /// 数据是否逐帧 +1（小端整数）。
    pub data_inc: bool,
    /// 帧的其余属性（类型 / 长度 / 数据）。
    pub frame: &'a Frame,
}

impl BulkSendConfig<'_> {
    pub(crate) fn to_raw(&self) -> ffi::dmcan_bulk_send_config_t {
        ffi::dmcan_bulk_send_config_t {
            interval_ms: self.interval_ms,
            send_times: self.send_times,
            can_id: self.can_id,
            id_inc: self.id_inc,
            step_id: self.step_id,
            stop_id: self.stop_id,
            data_inc: self.data_inc,
            canfd: self.frame.canfd,
            ext: self.frame.ext,
            rtr: self.frame.rtr,
            brs: self.frame.brs,
            dlen: self.frame.dlen,
            payload: self.frame.payload.as_ptr(),
        }
    }
}

/// 日志级别（与 C 侧 `dmcan.h` 的级别说明一致）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum LogLevel {
    /// trace（Release 构建通常已裁剪）。
    Trace,
    /// debug（Release 构建通常已裁剪）。
    Debug,
    /// info（默认，写 stdout）。
    Info,
    /// warn（写 stderr 并立即 flush）。
    Warn,
    /// error。
    Error,
    /// critical。
    Critical,
    /// off（静音终端日志）。
    Off,
}

impl LogLevel {
    /// 转成 C 侧级别的整型值。
    pub fn as_i32(self) -> i32 {
        match self {
            LogLevel::Trace => 0,
            LogLevel::Debug => 1,
            LogLevel::Info => 2,
            LogLevel::Warn => 3,
            LogLevel::Error => 4,
            LogLevel::Critical => 5,
            LogLevel::Off => 6,
        }
    }

    /// 从 C 侧级别整型值还原（越界按 `Info` 处理）。
    pub fn from_i32(level: i32) -> Self {
        match level {
            0 => LogLevel::Trace,
            1 => LogLevel::Debug,
            3 => LogLevel::Warn,
            4 => LogLevel::Error,
            5 => LogLevel::Critical,
            6 => LogLevel::Off,
            _ => LogLevel::Info,
        }
    }
}

/// 一条日志（回调参数，`line` 只在本次回调内有效）。
#[derive(Clone, Copy, Debug)]
pub struct LogRecord<'a> {
    /// 日志级别。
    pub level: LogLevel,
    /// 进程内单调递增序号（跨线程按它排序才是确定顺序）。
    pub seq: u64,
    /// 已格式化的整行文本（不含换行）。
    pub line: &'a str,
}

/// 热插拔事件类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HotplugKind {
    /// 设备接入。
    Attached,
    /// 设备移除。
    Detached,
}

/// 热插拔事件内容。
///
/// 刻意**不带设备句柄、不带序列号**（C 侧约定）：要拿句柄请重新 `find_devices()` +
/// `device(index)` —— 那条路径不会让手里的句柄失效。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HotplugEvent {
    /// 接入还是移除。
    pub kind: HotplugKind,
    /// 设备型号。
    pub device: DeviceType,
    /// USB VID。
    pub vendor_id: i32,
    /// USB PID。
    pub product_id: i32,
    /// 总线号。
    pub bus_number: u8,
    /// 端口号（拿不到时恒 0）。
    pub port_number: u8,
}

impl HotplugEvent {
    /// # Safety
    ///
    /// `raw` 必须指向 C 侧本次回调期间有效的设备描述。
    pub(crate) unsafe fn from_raw(kind: HotplugKind, raw: &ffi::dmcan_hotplug_device_t) -> Self {
        HotplugEvent {
            kind,
            device: DeviceType::from_raw(raw.device_type),
            vendor_id: raw.vendor_id,
            product_id: raw.product_id,
            bus_number: raw.bus_number,
            port_number: raw.port_number,
        }
    }
}
