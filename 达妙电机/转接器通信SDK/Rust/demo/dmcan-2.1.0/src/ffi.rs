//! C SDK（`dmcan`）导出符号的裸绑定层，与 `include/dmcan/dmcan.h`、`dmcan_types.h` 逐条对应。
//!
//! 本模块是**唯一**出现 `extern "C"` 声明的地方，也是唯一需要对着 C 头文件核对的地方。
//! 改这里必须同时满足两条：
//!
//! 1. 结构体布局与 C 侧逐字节一致——`dmcan_types.h` 里 `#pragma pack(push,1)` 区间的类型
//!    用 `#[repr(C, packed)]`，`#pragma pack(pop)` 之后的类型用 `#[repr(C)]`，**不能一律 packed**；
//! 2. 文件末尾的 ABI 常量断言继续成立——那是"C 侧布局变了就编译失败"的护栏，
//!    断言里的数字来源于用 C 编译器对交付包头文件跑 `sizeof` / `offsetof` 的实测值。
//!
//! 安全层（`context` / `device` / `subscribe` 等模块）只通过本模块调用 C SDK；
//! 直接使用本模块需要调用方自己保证指针、生命周期与线程约束，属于逃逸通道。
#![allow(non_camel_case_types)]
#![allow(dead_code)]

use core::ffi::{c_char, c_int, c_void};

// ---------------------------------------------------------------- 不透明句柄

/// SDK 上下文（不透明；C 侧 `struct dmcan_context`）。
#[repr(C)]
pub struct dmcan_context {
    _private: [u8; 0],
}

/// 设备句柄（不透明；C 侧 `struct dmcan_device_handle`，由上下文句柄表持有）。
#[repr(C)]
pub struct dmcan_device_handle {
    _private: [u8; 0],
}

/// 设备事件订阅句柄（不透明；由 SDK 持有，用完必须还回 `dmcan_device_unsubscribe`）。
#[repr(C)]
pub struct dmcan_subscription {
    _private: [u8; 0],
}

/// 日志订阅句柄（不透明；由 SDK 持有，用完必须还回 `dmcan_remove_log_callback`）。
#[repr(C)]
pub struct dmcan_log_subscription {
    _private: [u8; 0],
}

// ---------------------------------------------------------------- 常量

/// SDK 主版本号（与 `dmcan_version.h` 一致）。
pub const DMCAN_VERSION_MAJOR: u32 = 2;
/// SDK 次版本号。
pub const DMCAN_VERSION_MINOR: u32 = 1;
/// SDK 构建号。
pub const DMCAN_VERSION_BUILD: u32 = 0;
/// SDK 补丁号。
pub const DMCAN_VERSION_PATCH: u32 = 0;
/// 打包后的版本整型，用于 `dmcan_check_version()`（低字节起：patch/build/minor/major）。
pub const DMCAN_VERSION_INT: u32 = 0x0201_0000;

/// 模块能力位 bit0：支持 `CMD_SWITCH_CAN_RES` 软件开关终端电阻。
pub const DMCAN_MODULE_FEATURE_CAN_RES_SW: u16 = 1 << 0;

/// 单张过滤表的规则总数上限。
pub const DMCAN_FILTER_RULE_MAX: usize = 32;
/// 过滤表里标准 ID 规则的条数上限。
pub const DMCAN_FILTER_STD_RULE_MAX: u8 = 16;
/// 过滤表里扩展 ID 规则的条数上限。
pub const DMCAN_FILTER_EXT_RULE_MAX: u8 = 16;

/// "全部通道"通道掩码；**0 表示什么都没选**（参数错）。
pub const DMCAN_CHANNEL_MASK_ALL: u32 = u32::MAX;

/// 事件位：总线收到的帧。
pub const DMCAN_EVENT_RX_FRAME: u32 = 1 << 0;
/// 事件位：假 echo（这笔 USB 提交已完成）。
pub const DMCAN_EVENT_TX_COMPLETED: u32 = 1 << 1;
/// 事件位：真 echo（固件回报该帧已在总线上被确认）。
pub const DMCAN_EVENT_TX_ON_BUS: u32 = 1 << 2;
/// 事件位：通道错误记录。
pub const DMCAN_EVENT_CHANNEL_ERROR: u32 = 1 << 3;
/// 事件位：USB2UART 收到数据。
pub const DMCAN_EVENT_UART_RX: u32 = 1 << 4;

/// 丢包细分有效位 bit0：接收/echo 侧细分（`0x05`）已收到。
pub const DMCAN_DROP_FLAG_RX_ECHO: u8 = 0x01;
/// 丢包细分有效位 bit1：发送侧细分（`0x07`）已收到。
pub const DMCAN_DROP_FLAG_TX: u8 = 0x02;

/// 错误码：上一次调用成功。
pub const DMCAN_OK: c_int = 0;
/// 错误码：入参非法（空指针、通道号越界等）。
pub const DMCAN_ERR_INVALID_ARG: c_int = 1;
/// 错误码：设备尚未 open（或已 close）。
pub const DMCAN_ERR_NOT_OPEN: c_int = 2;
/// 错误码：设备索引越界 / 当前没有设备。
pub const DMCAN_ERR_DEVICE_NOT_FOUND: c_int = 3;
/// 错误码：内存不足。
pub const DMCAN_ERR_NO_MEMORY: c_int = 4;
/// 错误码：与设备交互失败（USB 超时 / 固件拒绝）。
pub const DMCAN_ERR_IO: c_int = 5;
/// 错误码：设备或固件不支持该能力。
pub const DMCAN_ERR_UNSUPPORTED: c_int = 6;
/// 错误码：内部状态异常（不该发生）。
pub const DMCAN_ERR_INTERNAL: c_int = 7;
/// 错误码：在 SDK 自己的回调线程上调用了会阻塞/自锁的接口，调用被拒绝。
pub const DMCAN_ERR_REENTRANT: c_int = 8;

/// 设备型号：USB2CANFD_S（单路）。
pub const DMCAN_DEVICE_USB2CANFD_S: c_int = 0;
/// 设备型号：USB2CANFD_D（双路）。
pub const DMCAN_DEVICE_USB2CANFD_D: c_int = 1;
/// 设备型号：LinkX4C（四路）。
pub const DMCAN_DEVICE_LINKX4C: c_int = 2;
/// 设备型号：未知。
pub const DMCAN_DEVICE_UNKNOWN: c_int = 3;

/// 发送聚合档位：吞吐优先（默认）。
pub const DMCAN_SEND_PROFILE_THROUGHPUT: c_int = 0;
/// 发送聚合档位：延迟优先。
pub const DMCAN_SEND_PROFILE_LOW_LATENCY: c_int = 1;

/// 节点模式：正常收发（默认）。
pub const DMCAN_CHANNEL_MODE_NORMAL: u8 = 0;
/// 节点模式：内部回环（TX 不外发，帧在芯片内回到接收路径）。
pub const DMCAN_CHANNEL_MODE_LOOPBACK_INTERNAL: u8 = 1;
/// 节点模式：外部回环（帧照常驱动总线并回环到自己；不依赖外部 ACK）。
pub const DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL: u8 = 2;
/// 节点模式：只听不发（不驱动总线、不回 ACK）。
pub const DMCAN_CHANNEL_MODE_LISTEN_ONLY: u8 = 3;

/// 热插拔事件：设备接入。
pub const DMCAN_HOTPLUG_ATTACHED: c_int = 0;
/// 热插拔事件：设备移除。
pub const DMCAN_HOTPLUG_DETACHED: c_int = 1;

/// 过滤模式：白名单（未命中 → 拒收）。
pub const DMCAN_FILTER_MODE_WHITELIST: u8 = 0;
/// 过滤模式：黑名单（未命中 → 正常接收）。
pub const DMCAN_FILTER_MODE_BLACKLIST: u8 = 1;

/// 过滤规则 ID 类型：标准帧（11 位 ID）。
pub const DMCAN_FILTER_ID_STD: u8 = 0;
/// 过滤规则 ID 类型：扩展帧（29 位 ID）。
pub const DMCAN_FILTER_ID_EXT: u8 = 1;

/// 过滤规则匹配类型：范围 `id1 <= rx_id <= id2`。
pub const DMCAN_FILTER_TYPE_RANGE: u8 = 0;
/// 过滤规则匹配类型：双 ID 精确。
pub const DMCAN_FILTER_TYPE_DUAL_ID: u8 = 1;
/// 过滤规则匹配类型：掩码 `(rx_id & id2) == (id1 & id2)`。
pub const DMCAN_FILTER_TYPE_MASK: u8 = 2;

/// 过滤规则命中动作：存 RX FIFO0（正常接收）。
pub const DMCAN_FILTER_ACTION_FIFO0: u8 = 0;
/// 过滤规则命中动作：拒收。
pub const DMCAN_FILTER_ACTION_REJECT: u8 = 1;
/// 过滤规则命中动作：置优先级并存 RX FIFO0。
pub const DMCAN_FILTER_ACTION_PRIORITY: u8 = 2;

// ---------------------------------------------------------------- 结构体
//
// 以下类型位于 C 侧 `#pragma pack(push,1)` 区间内：一律 1 字节对齐。

/// 固件版本号（3B）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct version_t {
    /// 主版本。
    pub major: u8,
    /// 次版本。
    pub minor: u8,
    /// 构建号。
    pub build: u8,
}

/// 设备固件信息（49B）：型号 + 发布版本号 + 构建标识。
///
/// 两个 `*_valid` 分别表示"各自那部分这次读到了没有"；没读到的字段保持零值
/// （版本号 `0.0.0` / 空串），**不会回上一轮的旧值**。
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct dmcan_firmware_info_t {
    /// 设备型号，取值见 `DMCAN_DEVICE_*`。
    pub device: c_int,
    /// 发布版本号（`version_valid` 为假时恒 `0.0.0`）。
    pub version: version_t,
    /// 版本号是否读到；**设备跑在 boot 时为假**（判"还在 boot"用它，不要用接口返回值）。
    pub version_valid: bool,
    /// 构建标识字符串（ASCII，≤ 39 字节，NUL 结尾；固件没提供时为空串）。
    pub build_id: [c_char; 40],
    /// 构建标识是否读到。
    pub build_id_valid: bool,
}

/// 通道波特率配置（简化形态，19B）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_channel_can_info_t {
    /// 是否启用 CAN FD。
    pub canfd: bool,
    /// 是否非 ISO（CAN FD 非 ISO 模式）。
    pub non_iso: bool,
    /// 是否自动重传。
    pub retrans: bool,
    /// 仲裁段波特率。
    pub can_baudrate: u32,
    /// 数据段波特率；> 1 Mbps 时 SDK 自动打开 TDC。
    pub canfd_baudrate: u32,
    /// 仲裁段采样点（0~1）。
    pub can_sp: f32,
    /// 数据段采样点（0~1）。
    pub canfd_sp: f32,
}

/// 通道完整下发结构（22B，与固件布局逐字段对应）。
///
/// 注意首字节是 C 位域：`enable_canfd:1 | enable_tdc:1 | non_iso:1 | enable_retrans:1 | mode:4`，
/// 低位在前（实测 MSVC / MinGW / Clang 三套编译器约定一致）。Rust 没有位域，
/// 因此这里用一个字节承载，由安全层的位访问器读写；**不要**把它当成 5 个独立字段。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_ch_baud_conf_t {
    /// 位域字节：bit0 `enable_canfd`、bit1 `enable_tdc`、bit2 `non_iso`、bit3 `enable_retrans`、bit4~7 `mode`。
    pub flags: u8,
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
    /// 预留，恒 0。
    pub reserved: u8,
}

/// 通道 TDC 硬件实际生效值（4B）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct dmcan_tdc_status_t {
    /// 是否读到硬件实际值。
    pub valid: bool,
    /// 硬件生效的二次采样点偏移。
    pub tdco: u8,
    /// 硬件生效的滤波窗口。
    pub tdcf: u8,
    /// 硬件实测收发器延迟（只有成功发送过数据帧后才非 0）。
    pub tdcv: u8,
}

/// 通道扩展配置（3B）：重传策略与 bus-off 自动恢复间隔。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_channel_ext_config_t {
    /// 1 = 自动重传（默认）；0 = 一次性发送。
    pub enable_retrans: u8,
    /// bus-off 后自动恢复间隔（毫秒）；0 = 不自动恢复。
    pub busoff_autoreset_ms: u16,
}

/// 单条硬件过滤规则（12B，与固件布局逐字节一致）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_filter_rule_t {
    /// ID 类型，取值见 `DMCAN_FILTER_ID_*`。
    pub id_type: u8,
    /// 匹配类型，取值见 `DMCAN_FILTER_TYPE_*`。
    pub rule_type: u8,
    /// 命中动作，取值见 `DMCAN_FILTER_ACTION_*`。
    pub action: u8,
    /// 预留，**必须填 0**。
    pub reserved: u8,
    /// 含义随 `rule_type` 变化。
    pub id1: u32,
    /// 含义随 `rule_type` 变化。
    pub id2: u32,
}

/// 过滤表下发后的用量（2B）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_filter_usage_t {
    /// 表里标准 ID 规则的条数。
    pub std_used: u8,
    /// 表里扩展 ID 规则的条数。
    pub ext_used: u8,
}

/// 过滤表回读结果（389B，与应答数据区逐字节对应）。
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct dmcan_filter_table_t {
    /// 当前模式，取值见 `DMCAN_FILTER_MODE_*`。
    pub mode: u8,
    /// 固件支持的标准 ID 规则上限。
    pub std_max: u8,
    /// 固件支持的扩展 ID 规则上限。
    pub ext_max: u8,
    /// 表里实际规则条数。
    pub count: u16,
    /// 规则数组，按匹配顺序排列。
    pub rules: [dmcan_filter_rule_t; DMCAN_FILTER_RULE_MAX],
}

/// 接收/发送回传的一帧（84B，与固件回传逐字节一致）。
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct dmcan_rx_frame_t {
    /// 帧 ID。
    pub can_id: u32,
    /// 单位 µs；真 echo / 接收帧 / 错误记录是设备时间戳，假 echo 是上位机系统时间。
    pub timestamp: u64,
    /// 通道号。
    pub channel: u8,
    /// 是否扩展帧。
    pub ext: u8,
    /// 是否远程帧。
    pub rtr: u8,
    /// 是否 CAN FD。
    pub canfd: u8,
    /// 是否 BRS。
    pub brs: u8,
    /// 是否错误状态指示（ESI）。
    pub esi: u8,
    /// ack = 0 假 echo / 1 真 echo。
    pub ack: u8,
    /// 数据长度（字节）。
    pub dlen: u8,
    /// 数据；`dlen` 之外的字节无意义。
    pub payload: [u8; 64],
}

/// 通道错误记录（16B，与固件 `dmcan_channel_error_payload_t` 逐字节一致）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_channel_error_t {
    /// 设备时间戳，单位 µs。
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

/// 发送帧（75B，旧式 hook 回调用它回传）。
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct dmcan_tx_frame_t {
    /// 帧 ID。
    pub can_id: u32,
    /// 通道号。
    pub channel: u8,
    /// 是否扩展帧。
    pub ext: u8,
    /// 是否远程帧。
    pub rtr: u8,
    /// 是否 CAN FD。
    pub canfd: u8,
    /// 是否 BRS。
    pub brs: u8,
    /// 是否错误状态指示。
    pub esi: u8,
    /// 数据长度。
    pub dlen: u8,
    /// 数据。
    pub payload: [u8; 64],
}

/// echo 诊断计数（24B）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_echo_stats_t {
    /// 超过阈值仍未收到总线确认的帧数（累计）。
    pub timeout_count: u64,
    /// 超时后才回来、按迟到丢弃的条数（累计）。
    pub late_discarded: u64,
    /// 当前仍在等总线确认的帧数。
    pub pending: u32,
    /// 当前超时阈值（毫秒）。
    pub timeout_ms: u32,
}

// 以下是 C 侧 `#pragma pack(pop)` 之后的类型：自然对齐，**不能**标 packed。

/// UART 收数据的事件负载（16B）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct dmcan_uart_chunk_t {
    /// 数据首地址（仅本次回调有效）。
    pub data: *const u8,
    /// 字节数。
    pub length: u32,
    /// 预留，恒 0。
    pub reserved: u32,
}

/// UART 线参数（8B）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_uart_config_t {
    /// 波特率；0 = 不改线参数。
    pub baud_rate: u32,
    /// 数据位：5~8。
    pub data_bits: u8,
    /// 校验：0 = 无 / 1 = 奇 / 2 = 偶。
    pub parity: u8,
    /// 停止位：1 或 2。
    pub stop_bits: u8,
    /// 预留，恒 0。
    pub reserved: u8,
}

/// UART 统计（32B）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_uart_stats_t {
    /// 已投递给用户的字节数。
    pub rx_bytes: u64,
    /// 已写入端口的字节数。
    pub tx_bytes: u64,
    /// 用户回调来不及/队列满而丢掉的字节数。
    pub rx_dropped: u64,
    /// 串口层错误次数。
    pub errors: u32,
    /// 端口当前是否打开。
    pub is_open: u8,
    /// 预留，恒 0。
    pub reserved: [u8; 3],
}

/// 订阅事件（40B，零拷贝：`frame` / `error` / `uart` 指向 SDK 内部缓冲）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct dmcan_event_t {
    /// 本次事件类型（单个位），取值见 `DMCAN_EVENT_*`。
    pub event_type: u32,
    /// 通道号。
    pub channel: u8,
    /// 预留，恒 0。
    pub reserved: [u8; 3],
    /// 帧类事件（RX / TX 两类）有效，其余为 NULL。
    pub frame: *const dmcan_rx_frame_t,
    /// 错误事件有效，其余为 NULL。
    pub error: *const dmcan_channel_error_t,
    /// `DMCAN_EVENT_UART_RX` 有效，其余为 NULL。
    pub uart: *const dmcan_uart_chunk_t,
    /// 预留，恒 NULL。
    pub reserved2: [*mut c_void; 1],
}

/// 订阅配置（24B）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct dmcan_subscribe_config_t {
    /// 事件位图，取值见 `DMCAN_EVENT_*`；0 = 参数错。
    pub events: u32,
    /// 关心的通道，见 `DMCAN_CHANNEL_MASK_ALL`。
    pub channel_mask: u32,
    /// 事件回调；不可为 NULL。
    pub callback: dmcan_event_callback,
    /// 原样回传给 `callback`。
    pub user_data: *mut c_void,
}

/// 批量/递增发送配置（40B，主机侧结构，不参与协议线格式）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct dmcan_bulk_send_config_t {
    /// 发送间隔（毫秒）；0 = 只发一帧。
    pub interval_ms: u32,
    /// 发送次数；0 = 不限。
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
    /// 帧类型：CAN FD。
    pub canfd: bool,
    /// 帧类型：扩展帧。
    pub ext: bool,
    /// 帧类型：远程帧。
    pub rtr: bool,
    /// 是否启用 BRS。
    pub brs: bool,
    /// 数据长度（0~64）。
    pub dlen: u8,
    /// 数据；`dlen` 为 0 时可传 NULL。
    pub payload: *const u8,
}

/// 一帧待发送报文（76B，`dmcan_device_send_can_batch` 用）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct dmcan_frame_t {
    /// 帧 ID。
    pub can_id: u32,
    /// 帧类型：CAN FD。
    pub canfd: bool,
    /// 帧类型：扩展帧。
    pub ext: bool,
    /// 帧类型：远程帧。
    pub rtr: bool,
    /// 是否启用 BRS。
    pub brs: bool,
    /// 数据长度（0~64）。
    pub dlen: u8,
    /// 数据；`dlen` 之外的字节被忽略。
    pub payload: [u8; 64],
}

// ---------------------------------------------------------------- 位域字节（packed 区恢复）

/// 通道总线状态快照（5B，与固件 `CMD_GET_BUS_STATE` 应答一一对应）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_bus_state_t {
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

/// 通道统计快照（70B，来自固件每秒一次的周期上报）。
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_channel_stats_t {
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
    pub enabled: u8,
    /// 对齐保留，恒 0。
    pub reserved: u8,
    /// 丢包细分有效位，见 `DMCAN_DROP_FLAG_*`。
    pub drop_flags: u8,
    /// 对齐保留，恒 0。
    pub reserved2: [u8; 3],
    /// 接收队列满丢弃的帧数（需 `DMCAN_DROP_FLAG_RX_ECHO` 置位才有效）。
    pub rx_dropped_queue: u32,
    /// 接收帧写不进 msgc 丢弃的帧数。
    pub rx_dropped_msgc: u32,
    /// echo 待回传队列满丢弃的条数。
    pub echo_dropped_queue: u32,
    /// echo 写不进 msgc 丢弃的条数。
    pub echo_dropped_msgc: u32,
    /// 已写入 msgc 的 echo 条数。
    pub echo_sent: u32,
    /// 到达时 credit 耗尽，本帧不受理（需 `DMCAN_DROP_FLAG_TX` 置位才有效）。
    pub tx_dropped_no_credit: u32,
    /// 到达时软件发送队列满，本帧不受理。
    pub tx_dropped_queue_full: u32,
    /// 提交 MCAN 发送 FIFO 失败。
    pub tx_dropped_submit_fail: u32,
    /// 通道已关闭。
    pub tx_dropped_ch_disabled: u32,
}

/// 热插拔事件里的设备描述（不带句柄、不带序列号，见 C 侧注释）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct dmcan_hotplug_device_t {
    /// 设备型号，取值见 `DMCAN_DEVICE_*`。
    pub device_type: c_int,
    /// USB VID。
    pub vendor_id: c_int,
    /// USB PID。
    pub product_id: c_int,
    /// 总线号。
    pub bus_number: u8,
    /// 端口号（拿不到时恒 0）。
    pub port_number: u8,
}

// ---------------------------------------------------------------- 回调类型

/// 旧式收帧回调（已弃用，保留仅为兼容）。
pub type dev_recv_callback =
    Option<unsafe extern "C" fn(*mut dmcan_device_handle, *mut dmcan_rx_frame_t)>;
/// 旧式发送回调（已弃用，保留仅为兼容）。
pub type dev_sent_callback =
    Option<unsafe extern "C" fn(*mut dmcan_device_handle, *mut dmcan_rx_frame_t)>;
/// 旧式错误回调（已弃用，保留仅为兼容）。
pub type dev_err_callback =
    Option<unsafe extern "C" fn(*mut dmcan_device_handle, *mut dmcan_rx_frame_t)>;
/// 订阅事件回调：在 SDK 的通知线程上同步执行。
pub type dmcan_event_callback =
    Option<unsafe extern "C" fn(*mut dmcan_device_handle, *const dmcan_event_t, *mut c_void)>;
/// 日志回调：在产生日志的那条线程上同步执行。
pub type dmcan_log_callback = Option<unsafe extern "C" fn(c_int, u64, *const c_char, *mut c_void)>;
/// 热插拔回调：在 libusb 事件线程（或轮询线程）上同步执行。
pub type dmcan_hotplug_callback = Option<
    unsafe extern "C" fn(*mut dmcan_context, c_int, *const dmcan_hotplug_device_t, *mut c_void),
>;

// ---------------------------------------------------------------- 导出函数

extern "C" {
    // ---- 上下文 / 版本 / 日志 ----

    /// 创建 SDK 上下文。
    pub fn dmcan_context_create(ctx: *mut *mut dmcan_context);
    /// 销毁 SDK 上下文。
    pub fn dmcan_context_destroy(ctx: *mut dmcan_context);
    /// 打印 SDK 版本到终端。
    pub fn dmcan_print_version(ctx: *mut dmcan_context);
    /// 读取 SDK 版本（打包整型）。
    pub fn dmcan_get_sdk_version(ctx: *mut dmcan_context, version: *mut u32);
    /// 设置终端日志级别（0=trace … 6=off）。
    pub fn dmcan_set_log_level(level: c_int);
    /// 读取当前终端日志级别。
    pub fn dmcan_get_log_level() -> c_int;
    /// 立即刷新日志缓冲。
    pub fn dmcan_flush_log();
    /// 注册日志订阅（最多 4 个）。
    pub fn dmcan_add_log_callback(
        cb: dmcan_log_callback,
        min_level: c_int,
        user_data: *mut c_void,
        out_subscription: *mut *mut dmcan_log_subscription,
    ) -> bool;
    /// 注销日志订阅。
    pub fn dmcan_remove_log_callback(subscription: *mut dmcan_log_subscription) -> bool;
    /// 注册旧式日志回调（已弃用；同一槽位重复注册即覆盖，NULL 等于注销）。
    pub fn dmcan_set_log_callback(cb: dmcan_log_callback, min_level: c_int);
    /// 注销旧式日志回调（可重复调用）。
    pub fn dmcan_unregister_log_callback();

    // ---- 设备枚举 ----

    /// 枚举设备并刷新句柄表，返回表内设备总数（含已拔掉的）。
    pub fn dmcan_find_devices(ctx: *mut dmcan_context) -> c_int;
    /// 按型号枚举设备，返回表内该型号设备总数。
    pub fn dmcan_find_devices_with_type(ctx: *mut dmcan_context, device_type: c_int) -> c_int;
    /// 打印句柄表内全部设备信息到终端。
    pub fn dmcan_show_all_devices(ctx: *mut dmcan_context);
    /// 按句柄表下标取设备句柄。
    pub fn dmcan_device_get(
        ctx: *mut dmcan_context,
        dev_handle: *mut *mut dmcan_device_handle,
        index: c_int,
    ) -> bool;
    /// 查询该设备是否还在总线上。
    pub fn dmcan_device_is_present(
        dev_handle: *mut dmcan_device_handle,
        present: *mut bool,
    ) -> bool;
    /// 读取设备的枚举信息（型号 / PID / VID / 序列号）。
    pub fn dmcan_device_get_info(
        dev_handle: *mut dmcan_device_handle,
        device_type: *mut c_int,
        pid: *mut c_int,
        vid: *mut c_int,
        serialnum: *mut *mut u8,
    );

    // ---- 设备开关与版本 ----

    /// 打开设备。
    pub fn dmcan_device_open(dev_handle: *mut dmcan_device_handle) -> bool;
    /// 关闭设备（可重复调用）。
    pub fn dmcan_device_close(dev_handle: *mut dmcan_device_handle);
    /// 读设备固件信息：一条 `CMD_GET_VERSION` 拿全发布版本号与构建标识。
    pub fn dmcan_device_get_firmware_info(
        dev_handle: *mut dmcan_device_handle,
        out: *mut dmcan_firmware_info_t,
    ) -> bool;
    /// 把固件信息格式化成人读的一行（不访问设备）。
    pub fn dmcan_firmware_info_to_string(
        info: *const dmcan_firmware_info_t,
        buffer: *mut c_char,
        buffer_size: u32,
    ) -> u32;

    // ---- 通道控制 ----

    /// 使能通道。
    pub fn dmcan_device_enable_channel(dev_handle: *mut dmcan_device_handle, channel: u8) -> bool;
    /// 关闭通道。
    pub fn dmcan_device_disable_channel(dev_handle: *mut dmcan_device_handle, channel: u8) -> bool;
    /// 读通道波特率（简化形态）。
    pub fn dmcan_device_get_channel_baudrate(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        baud_info: *mut dmcan_channel_can_info_t,
    ) -> bool;
    /// 读通道完整下发结构（含 TDC 字段）。
    pub fn dmcan_device_get_channel_baudrate_details(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        config: *mut dmcan_ch_baud_conf_t,
    ) -> bool;
    /// 写通道波特率（TDC 由 SDK 自动决定开关）。
    pub fn dmcan_device_set_channel_baudrate(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        baud_info: *const dmcan_channel_can_info_t,
    ) -> bool;
    /// 按完整下发结构写通道参数。
    pub fn dmcan_device_set_channel_baudrate_details(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        config: *const dmcan_ch_baud_conf_t,
    ) -> bool;
    /// 读通道 TDC 硬件实际生效值。
    pub fn dmcan_device_get_channel_tdc(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        out: *mut dmcan_tdc_status_t,
    ) -> bool;
    /// 把当前运行期参数保存进设备闪存。
    pub fn dmcan_device_save_param(dev_handle: *mut dmcan_device_handle) -> bool;

    // ---- 过滤 / 看门狗 ----

    /// 下发硬件过滤表。
    pub fn dmcan_device_set_channel_filter(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        mode: u8,
        rules: *const dmcan_filter_rule_t,
        count: u16,
        usage: *mut dmcan_filter_usage_t,
    ) -> bool;
    /// 回读硬件过滤表。
    pub fn dmcan_device_get_channel_filter(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        table: *mut dmcan_filter_table_t,
    ) -> bool;
    /// 使能/关闭通道看门狗。
    pub fn dmcan_device_enable_watchdog(
        dev_handle: *mut dmcan_device_handle,
        ms: u16,
        state: bool,
    ) -> bool;

    // ---- 订阅 / 旧式回调 ----

    /// 订阅设备事件（每设备最多 4 个订阅者）。
    pub fn dmcan_device_subscribe(
        dev_handle: *mut dmcan_device_handle,
        config: *const dmcan_subscribe_config_t,
        out_subscription: *mut *mut dmcan_subscription,
    ) -> bool;
    /// 注销订阅并释放该槽位。
    pub fn dmcan_device_unsubscribe(
        dev_handle: *mut dmcan_device_handle,
        subscription: *mut dmcan_subscription,
    ) -> bool;
    /// 注册旧式收帧回调（已弃用）。
    pub fn dmcan_device_hook_recv_callback(
        dev_handle: *mut dmcan_device_handle,
        callback: dev_recv_callback,
    );
    /// 注册旧式发送回调（已弃用）。
    pub fn dmcan_device_hook_sent_callback(
        dev_handle: *mut dmcan_device_handle,
        callback: dev_sent_callback,
    );
    /// 注册旧式错误回调（已弃用）。
    pub fn dmcan_device_hook_err_callback(
        dev_handle: *mut dmcan_device_handle,
        callback: dev_err_callback,
    );

    // ---- 发送 ----

    /// 单次发送一帧。
    pub fn dmcan_device_send_can(
        dev_handle: *mut dmcan_device_handle,
        ch: u8,
        can_id: u32,
        canfd: bool,
        ext: bool,
        rtr: bool,
        brs: bool,
        dlen: u8,
        payload: *mut u8,
    ) -> bool;
    /// 周期发送（`interval_ms = 0` 等价单次发送）。
    pub fn dmcan_device_send_can_details(
        dev_handle: *mut dmcan_device_handle,
        ch: u8,
        interval_ms: u32,
        send_times: u32,
        can_id: u32,
        canfd: bool,
        ext: bool,
        rtr: bool,
        brs: bool,
        dlen: u8,
        payload: *mut u8,
    ) -> bool;
    /// 停止指定通道的周期发送任务。
    pub fn dmcan_device_stop_send(dev_handle: *mut dmcan_device_handle, channel: u8) -> bool;
    /// 设置发送聚合档位。
    pub fn dmcan_device_set_send_profile(
        dev_handle: *mut dmcan_device_handle,
        profile: c_int,
    ) -> bool;
    /// 读取发送聚合档位。
    pub fn dmcan_device_get_send_profile(
        dev_handle: *mut dmcan_device_handle,
        out: *mut c_int,
    ) -> bool;
    /// 批量/递增发送。
    pub fn dmcan_device_send_can_bulk(
        dev_handle: *mut dmcan_device_handle,
        ch: u8,
        config: *const dmcan_bulk_send_config_t,
    ) -> bool;
    /// 一次交一批帧。
    pub fn dmcan_device_send_can_batch(
        dev_handle: *mut dmcan_device_handle,
        ch: u8,
        frames: *const dmcan_frame_t,
        count: u32,
    ) -> bool;

    // ---- USB2UART 透传 ----

    /// 开关"打开设备时自动打开 UART"（默认开）。
    pub fn dmcan_device_set_uart_auto_open(
        dev_handle: *mut dmcan_device_handle,
        enable: bool,
    ) -> bool;
    /// 设置 UART 线参数。
    pub fn dmcan_device_set_uart_config(
        dev_handle: *mut dmcan_device_handle,
        config: *const dmcan_uart_config_t,
    ) -> bool;
    /// 查询 UART 端口是否已打开。
    pub fn dmcan_device_uart_is_open(
        dev_handle: *mut dmcan_device_handle,
        is_open: *mut bool,
    ) -> bool;
    /// 取当前 UART 端口名（如 `COM70` / `/dev/ttyACM0`）。
    pub fn dmcan_device_uart_get_port_name(
        dev_handle: *mut dmcan_device_handle,
        buffer: *mut c_char,
        buffer_size: u32,
    ) -> bool;
    /// 手动打开 UART 端口。
    pub fn dmcan_device_uart_open(
        dev_handle: *mut dmcan_device_handle,
        config: *const dmcan_uart_config_t,
    ) -> bool;
    /// 关闭 UART 端口（可重复调用）。
    pub fn dmcan_device_uart_close(dev_handle: *mut dmcan_device_handle);
    /// 往 UART 写一段数据。
    pub fn dmcan_device_uart_write(
        dev_handle: *mut dmcan_device_handle,
        data: *const u8,
        length: u32,
    ) -> bool;
    /// 读 UART 统计。
    pub fn dmcan_device_get_uart_stats(
        dev_handle: *mut dmcan_device_handle,
        out: *mut dmcan_uart_stats_t,
    ) -> bool;

    // ---- 通道扩展配置 / 诊断 ----

    /// 读通道扩展配置（重传策略与 bus-off 自动恢复间隔）。
    pub fn dmcan_device_get_channel_ext_config(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        config: *mut dmcan_channel_ext_config_t,
    ) -> bool;
    /// 写通道扩展配置。
    pub fn dmcan_device_set_channel_ext_config(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        config: *const dmcan_channel_ext_config_t,
    ) -> bool;
    /// 纯函数：长度 → DLC（向上取整）；越界回 -1。
    pub fn dmcan_utils_get_dlc_from_len(dlen: c_int) -> c_int;
    /// 纯函数：DLC → 长度；越界回 -1。
    pub fn dmcan_utils_get_len_from_dlc(dlc: c_int) -> c_int;
    /// 读取设备的模块能力位。
    pub fn dmcan_device_get_module_features(
        dev_handle: *mut dmcan_device_handle,
        features: *mut u16,
    ) -> bool;
    /// 读取本线程上一次 API 调用的错误码。
    pub fn dmcan_get_last_error() -> c_int;
    /// 手动把本线程的错误码清成 `DMCAN_OK`。
    pub fn dmcan_clear_last_error();
    /// 开关通道的软件终端电阻。
    pub fn dmcan_device_enable_can_res(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        state: bool,
    ) -> bool;
    /// 读软件终端电阻的当前状态。
    pub fn dmcan_device_get_can_res(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        enabled: *mut bool,
    ) -> bool;
    /// 开关通道的总线 echo。
    pub fn dmcan_device_set_channel_echo(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        enable: bool,
    ) -> bool;
    /// 取 echo 诊断计数。
    pub fn dmcan_device_get_echo_stats(
        dev_handle: *mut dmcan_device_handle,
        stats: *mut dmcan_echo_stats_t,
    ) -> bool;
    /// 主动清一次通道的 bus-off。
    pub fn dmcan_device_clear_bus_off(dev_handle: *mut dmcan_device_handle, channel: u8) -> bool;
    /// 读最近一次周期上报的通道统计快照。
    pub fn dmcan_device_get_channel_stats(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        out: *mut dmcan_channel_stats_t,
    ) -> bool;
    /// 按需读一次指定通道的总线状态。
    pub fn dmcan_device_get_bus_state(
        dev_handle: *mut dmcan_device_handle,
        channel: u8,
        out: *mut dmcan_bus_state_t,
    ) -> bool;
    /// 设置 echo 超时阈值（毫秒，内部 clamp 到 [10, 60000]）。
    pub fn dmcan_device_set_echo_timeout_ms(dev_handle: *mut dmcan_device_handle, timeout_ms: u32);

    // ---- 热插拔 / 版本校验 ----

    /// 注册热插拔回调（同一上下文只保留一个）。
    pub fn dmcan_context_register_hotplug(
        ctx: *mut dmcan_context,
        cb: dmcan_hotplug_callback,
        user_data: *mut c_void,
    ) -> bool;
    /// 注销热插拔回调并停掉监听。
    pub fn dmcan_context_unregister_hotplug(ctx: *mut dmcan_context);
    /// 本机 libusb 的**原生**热插拔能力是否可用（诊断用）。
    pub fn dmcan_context_hotplug_supported(ctx: *mut dmcan_context) -> bool;
    /// 校验"编译用的头文件"与"手里的库"是否配套。
    pub fn dmcan_check_version(expected_version: u32) -> bool;
}

// ---------------------------------------------------------------- ABI 护栏
//
// 断言值来自用 C 编译器对交付包头文件跑 `sizeof` / `offsetof` 的实测输出。
// C 侧一旦改动结构体布局，这里编译失败——比运行时静默错位好得多。

const _: () = {
    use core::mem::{offset_of, size_of};

    // packed 区（1 字节对齐）
    assert!(size_of::<version_t>() == 3);
    assert!(size_of::<dmcan_firmware_info_t>() == 49);
    assert!(offset_of!(dmcan_firmware_info_t, version) == 4);
    assert!(offset_of!(dmcan_firmware_info_t, version_valid) == 7);
    assert!(offset_of!(dmcan_firmware_info_t, build_id) == 8);
    assert!(offset_of!(dmcan_firmware_info_t, build_id_valid) == 48);
    assert!(size_of::<dmcan_channel_can_info_t>() == 19);
    assert!(size_of::<dmcan_ch_baud_conf_t>() == 22);
    assert!(offset_of!(dmcan_ch_baud_conf_t, nominal_brp) == 1);
    assert!(offset_of!(dmcan_ch_baud_conf_t, data_brp) == 9);
    assert!(offset_of!(dmcan_ch_baud_conf_t, busoff_autoreset_ms) == 17);
    assert!(offset_of!(dmcan_ch_baud_conf_t, tdc_offset) == 19);
    assert!(offset_of!(dmcan_ch_baud_conf_t, reserved) == 21);
    assert!(size_of::<dmcan_tdc_status_t>() == 4);
    assert!(size_of::<dmcan_channel_ext_config_t>() == 3);
    assert!(size_of::<dmcan_filter_rule_t>() == 12);
    assert!(size_of::<dmcan_filter_usage_t>() == 2);
    assert!(size_of::<dmcan_filter_table_t>() == 389);
    assert!(offset_of!(dmcan_filter_table_t, count) == 3);
    assert!(offset_of!(dmcan_filter_table_t, rules) == 5);
    assert!(size_of::<dmcan_rx_frame_t>() == 84);
    assert!(offset_of!(dmcan_rx_frame_t, channel) == 12);
    assert!(offset_of!(dmcan_rx_frame_t, dlen) == 19);
    assert!(offset_of!(dmcan_rx_frame_t, payload) == 20);
    assert!(size_of::<dmcan_channel_error_t>() == 16);
    assert!(offset_of!(dmcan_channel_error_t, error_code) == 8);
    assert!(offset_of!(dmcan_channel_error_t, flags) == 15);
    assert!(size_of::<dmcan_tx_frame_t>() == 75);
    assert!(size_of::<dmcan_echo_stats_t>() == 24);
    assert!(size_of::<dmcan_bus_state_t>() == 5);
    assert!(size_of::<dmcan_channel_stats_t>() == 70);
    assert!(offset_of!(dmcan_channel_stats_t, drop_flags) == 30);
    assert!(offset_of!(dmcan_channel_stats_t, rx_dropped_queue) == 34);
    assert!(offset_of!(dmcan_channel_stats_t, tx_dropped_ch_disabled) == 66);

    // pack(pop) 之后（自然对齐）
    assert!(size_of::<dmcan_uart_chunk_t>() == 16);
    assert!(size_of::<dmcan_uart_config_t>() == 8);
    assert!(size_of::<dmcan_uart_stats_t>() == 32);
    assert!(offset_of!(dmcan_uart_stats_t, is_open) == 28);
    assert!(size_of::<dmcan_event_t>() == 40);
    assert!(offset_of!(dmcan_event_t, event_type) == 0);
    assert!(offset_of!(dmcan_event_t, channel) == 4);
    assert!(offset_of!(dmcan_event_t, frame) == 8);
    assert!(offset_of!(dmcan_event_t, error) == 16);
    assert!(offset_of!(dmcan_event_t, uart) == 24);
    assert!(offset_of!(dmcan_event_t, reserved2) == 32);
    assert!(size_of::<dmcan_subscribe_config_t>() == 24);
    assert!(offset_of!(dmcan_subscribe_config_t, callback) == 8);
    assert!(offset_of!(dmcan_subscribe_config_t, user_data) == 16);
    assert!(size_of::<dmcan_bulk_send_config_t>() == 40);
    assert!(offset_of!(dmcan_bulk_send_config_t, id_inc) == 12);
    assert!(offset_of!(dmcan_bulk_send_config_t, stop_id) == 16);
    assert!(offset_of!(dmcan_bulk_send_config_t, dlen) == 25);
    assert!(offset_of!(dmcan_bulk_send_config_t, payload) == 32);
    assert!(size_of::<dmcan_frame_t>() == 76);
    assert!(offset_of!(dmcan_frame_t, canfd) == 4);
    assert!(offset_of!(dmcan_frame_t, dlen) == 8);
    assert!(offset_of!(dmcan_frame_t, payload) == 9);
    assert!(size_of::<dmcan_hotplug_device_t>() == 16);
};
