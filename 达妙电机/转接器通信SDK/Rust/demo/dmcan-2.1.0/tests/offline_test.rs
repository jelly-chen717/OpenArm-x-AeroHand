//! 不需要硬件就能跑的验证（V1–V6 的离线部分）。
//!
//! 真机全链路（枚举 → 打开 → 配置 → 收发）见 `tests/RUST_WRAPPER_SMOKE_TEST.md`，
//! 那部分需要接一块 USB2CANFD 模块。

use dmcan::{
    ffi, utils, ChannelBaudConf, ChannelMode, Context, Error, Events, Frame, SDK_VERSION_INT,
};

/// V1：结构体布局与 C 侧一致。
///
/// 真正的护栏是 `dmcan::ffi` 里的编译期常量断言（C 侧布局一变就编译失败），
/// 这里再做一次运行时复核，便于出问题时直接看到是哪一项。
#[test]
fn abi_layout_is_pinned() {
    use core::mem::size_of;

    assert_eq!(size_of::<ffi::version_t>(), 3);
    assert_eq!(size_of::<ffi::dmcan_firmware_info_t>(), 49);
    assert_eq!(size_of::<ffi::dmcan_channel_can_info_t>(), 19);
    assert_eq!(size_of::<ffi::dmcan_ch_baud_conf_t>(), 22);
    assert_eq!(size_of::<ffi::dmcan_tdc_status_t>(), 4);
    assert_eq!(size_of::<ffi::dmcan_channel_ext_config_t>(), 3);
    assert_eq!(size_of::<ffi::dmcan_filter_rule_t>(), 12);
    assert_eq!(size_of::<ffi::dmcan_filter_usage_t>(), 2);
    assert_eq!(size_of::<ffi::dmcan_filter_table_t>(), 389);
    assert_eq!(size_of::<ffi::dmcan_rx_frame_t>(), 84);
    assert_eq!(size_of::<ffi::dmcan_channel_error_t>(), 16);
    assert_eq!(size_of::<ffi::dmcan_tx_frame_t>(), 75);
    assert_eq!(size_of::<ffi::dmcan_echo_stats_t>(), 24);
    assert_eq!(size_of::<ffi::dmcan_bus_state_t>(), 5);
    assert_eq!(size_of::<ffi::dmcan_channel_stats_t>(), 70);
    assert_eq!(size_of::<ffi::dmcan_uart_chunk_t>(), 16);
    assert_eq!(size_of::<ffi::dmcan_uart_config_t>(), 8);
    assert_eq!(size_of::<ffi::dmcan_uart_stats_t>(), 32);
    assert_eq!(size_of::<ffi::dmcan_event_t>(), 40);
    assert_eq!(size_of::<ffi::dmcan_subscribe_config_t>(), 24);
    assert_eq!(size_of::<ffi::dmcan_bulk_send_config_t>(), 40);
    assert_eq!(size_of::<ffi::dmcan_frame_t>(), 76);
    assert_eq!(size_of::<ffi::dmcan_hotplug_device_t>(), 16);
}

/// V2：`dmcan_ch_baud_conf_t` 首字节的位域编码与 C 侧一致（低位在前）。
#[test]
fn bitfield_byte_matches_c_layout() {
    let mut conf = ChannelBaudConf::default();
    assert_eq!(conf.raw_flags(), 0);
    assert!(!conf.enable_canfd() && !conf.enable_tdc());

    conf.set_enable_canfd(true);
    conf.set_enable_tdc(true);
    conf.set_non_iso(true);
    conf.set_enable_retrans(true);
    conf.set_mode(ChannelMode::ListenOnly);
    // C 侧实测：enable_canfd=1, enable_tdc=1, non_iso=1, enable_retrans=1, mode=3 得到 0x3F
    assert_eq!(conf.raw_flags(), 0x3F);
    assert!(conf.enable_canfd());
    assert!(conf.enable_tdc());
    assert!(conf.non_iso());
    assert!(conf.enable_retrans());
    assert_eq!(conf.mode(), ChannelMode::ListenOnly);

    // mode 只占高 4 位：未定义值也能原样带回来（低位开关不受影响）
    conf.set_mode(ChannelMode::Undefined(0xA));
    assert_eq!(conf.raw_flags(), 0xAF);
    assert_eq!(conf.mode(), ChannelMode::Undefined(0xA));
    conf.set_mode(ChannelMode::Undefined(0x1F));
    assert_eq!(conf.mode(), ChannelMode::Undefined(0xF));
    assert_eq!(conf.raw_flags(), 0xFF);
    conf.set_enable_canfd(false);
    assert_eq!(conf.raw_flags(), 0xFE);

    // 四个已定义模式值能原样往返
    for mode in [
        ChannelMode::Normal,
        ChannelMode::LoopbackInternal,
        ChannelMode::LoopbackExternal,
        ChannelMode::ListenOnly,
    ] {
        conf.set_mode(mode);
        assert_eq!(conf.mode(), mode);
    }
}

/// V3：库与绑定配套（`dmcan_check_version`），并能读回 SDK 版本号。
#[test]
fn sdk_version_is_compatible() {
    let ctx = Context::new().expect("创建上下文");
    let version = ctx.sdk_version();
    assert_eq!((version.major, version.minor, version.build), (2, 1, 0));
    assert_eq!(SDK_VERSION_INT, 0x0201_0000);

    // 配套校验：正确版本为真，乱给一个为假。
    // SAFETY: 纯函数，只有整型参数。
    assert!(unsafe { ffi::dmcan_check_version(SDK_VERSION_INT) });
    assert!(!unsafe { ffi::dmcan_check_version(0x0001_0000) });
}

/// V4：纯函数的边界与越界行为（越界时 C 侧回 -1，包装层映射成 InvalidArg）。
#[test]
fn dlc_conversion_boundaries() {
    // 长度到 DLC：向上取整到能装下的那一档
    assert_eq!(utils::dlc_from_len(0).unwrap(), 0);
    assert_eq!(utils::dlc_from_len(8).unwrap(), 8);
    assert_eq!(utils::dlc_from_len(9).unwrap(), 9);
    assert_eq!(utils::dlc_from_len(12).unwrap(), 9);
    assert_eq!(utils::dlc_from_len(13).unwrap(), 10);
    assert_eq!(utils::dlc_from_len(64).unwrap(), 15);
    assert_eq!(utils::dlc_from_len(65), Err(Error::InvalidArg));
    assert_eq!(utils::dlc_from_len(255), Err(Error::InvalidArg));

    // DLC 到长度：按表原样返回
    assert_eq!(utils::len_from_dlc(0).unwrap(), 0);
    assert_eq!(utils::len_from_dlc(8).unwrap(), 8);
    assert_eq!(utils::len_from_dlc(9).unwrap(), 12);
    assert_eq!(utils::len_from_dlc(15).unwrap(), 64);
    assert_eq!(utils::len_from_dlc(16), Err(Error::InvalidArg));
}

/// V5：错误码是线程局部的，两个线程互不覆盖。
#[test]
fn last_error_is_thread_local() {
    // SAFETY: 只操作本线程的错误码。
    unsafe { ffi::dmcan_clear_last_error() };

    let child = std::thread::spawn(|| {
        // 越界调用会置 INVALID_ARG（并返回 -1）
        let ret = unsafe { ffi::dmcan_utils_get_dlc_from_len(99) };
        assert_eq!(ret, -1);
        unsafe { ffi::dmcan_get_last_error() }
    })
    .join()
    .expect("子线程");

    // SAFETY: 读取本线程（主线程）的错误码。
    let main = unsafe { ffi::dmcan_get_last_error() };
    assert_eq!(child, ffi::DMCAN_ERR_INVALID_ARG);
    assert_eq!(main, ffi::DMCAN_OK);
}

/// V6：上下文生命周期与"没有设备"时的行为。
#[test]
fn context_lifecycle_and_device_lookup() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();

    if count == 0 {
        // 没有设备时下标 0 应当报"设备不存在"，而不是崩掉。
        assert_eq!(ctx.device(0).err(), Some(Error::DeviceNotFound));
    } else {
        // 有设备时：句柄拿得到，且不打开也能读枚举信息。
        let dev = ctx.device(0).expect("取设备 0");
        assert!(dev.info().is_ok());
        // 句柄表只增不减：重新枚举后原来拿到的句柄仍然可用。
        let _ = ctx.find_devices();
        assert!(dev.is_present().is_ok());
    }

    // 上下文可以克隆（内部共享同一个 C 侧上下文），最后一个引用消失时才销毁。
    let cloned = ctx.clone();
    drop(ctx);
    let _ = cloned.find_devices();
}

/// 帧构造按 C 侧同一条规则做前置校验。
#[test]
fn frame_validation() {
    assert!(Frame::classic(0x100, &[0u8; 8]).is_ok());
    assert_eq!(Frame::classic(0x100, &[0u8; 9]), Err(Error::InvalidArg));
    assert!(Frame::fd(0x100, &[0u8; 64], true).is_ok());
    assert_eq!(Frame::fd(0x100, &[0u8; 65], true), Err(Error::InvalidArg));

    let frame = Frame::fd(0x123, &[1, 2, 3], true).unwrap();
    assert_eq!(frame.dlen, 3);
    assert_eq!(frame.data(), &[1, 2, 3]);
    assert!(frame.canfd && frame.brs && !frame.ext);

    let extended = Frame::classic(0x1FFF_FFFF, &[0xFF])
        .unwrap()
        .with_extended(true);
    assert!(extended.ext);
}

/// 订阅事件位图可以直接按位或。
#[test]
fn events_bitmap() {
    let events = Events::RX_FRAME | Events::TX_ON_BUS | Events::CHANNEL_ERROR;
    assert_eq!(events.bits(), 0b1101);
    assert!(!events.is_empty());
    assert_ne!(Events::UART_RX.bits(), 0);
}
