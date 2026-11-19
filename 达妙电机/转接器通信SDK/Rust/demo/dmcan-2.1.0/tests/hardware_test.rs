//! 真机用例（需要接一块 USB2CANFD 模块）。
//!
//! 默认被 `#[ignore]` 跳过，避免没有硬件的机器上跑 CI 时失败；要跑就显式：
//!
//! ```text
//! cargo test --test hardware_test -- --ignored --test-threads=1
//! ```
//!
//! 必须 `--test-threads=1`：同一台设备不能被两个用例同时打开。
//! 结果按仓库约定归档到 `tests/RUST_WRAPPER_SMOKE_TEST_REPORT.md`。

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use dmcan::{ChannelBaudInfo, ChannelMode, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

/// 用例里要操作的设备下标（默认 0；`DMCAN_TEST_DEVICE_INDEX=1` 可切到第二台）。
fn test_device_index() -> usize {
    std::env::var("DMCAN_TEST_DEVICE_INDEX")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0)
}

/// 测试用的 CAN FD 数据段波特率。
///
/// **默认 1 Mbps（与仲裁段同速）**：高数据段波特率（4~5 Mbps）对端接与线长很敏感，
/// 台架上只有一侧能软件控端接时容易丢 ACK、帧发不出去（2026-10-10 实测：1M@4M 下
/// 帧提交后一直不完成，降到 1M@1M 立刻双向通）。要跑高速率用它覆盖：
/// `DMCAN_TEST_DATA_BITRATE=4000000`。
fn test_data_bitrate() -> u32 {
    std::env::var("DMCAN_TEST_DATA_BITRATE")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(1_000_000)
}

/// 把通道配成 CAN FD 1 Mbps + 数据段 `test_data_bitrate()`（BRS），
/// 并用简化接口让 SDK 自动补齐数据段位定时。
fn configure_fd(dev: &dmcan::Device, channel: u8) {
    dev.set_channel_baudrate(
        channel,
        &ChannelBaudInfo {
            canfd: true,
            retrans: true,
            can_baudrate: 1_000_000,
            can_sp: 0.75,
            canfd_baudrate: test_data_bitrate(),
            canfd_sp: 0.75,
            ..Default::default()
        },
    )
    .expect("配通道波特率");
}

/// 等硬件把配置重载完（`CMD_SETUP_BAUD` 只在固件每秒任务里重载，实测 0.086 ~ 1.0 s）。
fn wait_baud_reload() {
    std::thread::sleep(Duration::from_millis(1200));
}

/// 按型号取 D / S 两台设备（不依赖枚举顺序）。
fn find_d_and_s(ctx: &Context) -> (dmcan::Device, dmcan::Device) {
    let count = ctx.find_devices();
    assert!(count >= 2, "需要 D 与 S 两台设备，当前只有 {count} 台");
    let mut d_dev = None;
    let mut s_dev = None;
    for index in 0..count {
        let dev = ctx.device(index).expect("取设备");
        match dev.info().expect("读枚举信息").device {
            dmcan::DeviceType::Usb2CanfdD => d_dev = Some(dev),
            dmcan::DeviceType::Usb2CanfdS => s_dev = Some(dev),
            other => println!("忽略设备[{index}]（{other:?}）"),
        }
    }
    (
        d_dev.expect("没找到 USB2CANFD_D"),
        s_dev.expect("没找到 USB2CANFD_S"),
    )
}

/// 台架就绪：两台都打开、关看门狗、配成 CAN FD 1M@`test_data_bitrate()`、使能，并开 S 的终端电阻。
///
/// 台架前提：**S 接在 D 的通道 0**（差分线 + 共地），S 支持软件端接。
fn setup_pair(d_dev: &dmcan::Device, s_dev: &dmcan::Device) {
    for (role, dev) in [("D", d_dev), ("S", s_dev)] {
        dev.open().expect("打开设备");
        // 关看门狗：本固件会拒绝（返回 IO），但调用本身是安全的（不会武装）。
        println!("{role} 关看门狗 → {:?}", dev.enable_watchdog(1000, false));
        configure_fd(dev, 0);
    }
    wait_baud_reload();
    assert!(enable_until_active(d_dev, 0, "发送方 D"), "D ch0 没使能上");
    assert!(enable_until_active(s_dev, 0, "接收方 S"), "S ch0 没使能上");
    println!(
        "S 开终端电阻 → {:?}（回读 {:?}）",
        s_dev.enable_can_res(0, true),
        s_dev.can_res(0)
    );
    wait_baud_reload();
}

/// 使能通道，并等到**设备侧的周期上报**确认 `enabled = true` 为止。
///
/// 为什么要轮询：参数下发会触发固件在"每秒任务"里重载 MCAN，重载可能把通道置回未使能
/// （实测 D 设备如此），而 `channel_stats` 的值来自每秒一条的周期上报，最长旧 1 秒。
/// 返回最终的 `enabled` 读数。
fn enable_until_active(dev: &dmcan::Device, channel: u8, role: &str) -> bool {
    let mut enabled = false;
    for attempt in 1..=4 {
        if let Err(error) = dev.enable_channel(channel) {
            println!("{role} 第 {attempt} 次使能失败：{error}");
        }
        // 只等一小会儿就确认：实测 D 设备"使能后过一两秒又被关掉"，
        // 等太久反而会错过可用窗口（周期上报最长旧 1 秒，所以这个读数可能滞后）。
        std::thread::sleep(Duration::from_millis(250));
        enabled = dev
            .channel_stats(channel)
            .ok()
            .flatten()
            .map(|stats| stats.enabled)
            .unwrap_or(false);
        println!("{role} 第 {attempt} 次确认：enabled={enabled}");
        if enabled {
            break;
        }
    }
    enabled
}

/// 000 号用例：枚举 → 打开 → 读版本 / 枚举信息 → 关设备。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn smoke_open_and_read_info() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();
    assert!(count > 0, "没有找到设备");

    let dev = ctx.device(0).expect("取设备 0");
    let info = dev.info().expect("读枚举信息");
    println!(
        "设备：{:?} VID=0x{:04X} PID=0x{:04X} 序列号={:?}",
        info.device, info.vid, info.pid, info.serial
    );
    dev.open().expect("打开设备");
    let firmware = dev.firmware_info().expect("读固件信息");
    println!(
        "固件信息：{}",
        firmware.to_string_line().expect("格式化固件信息")
    );
    println!(
        "  型号={:?} 版本={} 版本有效={} 构建标识={:?}",
        firmware.device, firmware.version, firmware.version_valid, firmware.build_id
    );
    assert!(dev.is_present().expect("查在位状态"));
    dev.close();
}

/// 100 号用例：订阅收帧事件，发一帧后必须收到对应的假 echo（`ack = 0`）。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn subscription_receives_fake_echo() {
    let ctx = Context::new().expect("创建上下文");
    assert!(ctx.find_devices() > 0, "没有找到设备");
    let dev = ctx.device(0).expect("取设备 0");
    dev.open().expect("打开设备");

    let seen = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&seen);
    let _sub = dev
        .subscribe(Events::TX_COMPLETED, CHANNEL_MASK_ALL, move |event| {
            if let Event::TxCompleted(frame) = event {
                // 回调里的断言若失败会 panic，被包装层吞掉并计入 panic_count，
                // 于是下面的计数保持为 0，用例照样能发现异常。
                assert_eq!(frame.can_id(), 0x321);
                counter.fetch_add(1, Ordering::SeqCst);
            }
        })
        .expect("订阅 TX_COMPLETED");

    // 顺序有讲究：**先配波特率、再使能通道**。实测（app `9ce398c`）配波特率会让固件重载 MCAN
    // 并把通道置回未使能，先使能再配等于白配、后续帧会被计成 `tx_dropped_ch_disabled`。
    configure_fd(&dev, 0);
    wait_baud_reload();
    dev.enable_channel(0).expect("使能通道 0");
    wait_baud_reload();

    let frame = Frame::fd(0x321, &[0xDE, 0xAD, 0xBE, 0xEF], true).expect("构造帧");
    dev.send_can(0, &frame).expect("发送");

    let deadline = Instant::now() + Duration::from_secs(2);
    while seen.load(Ordering::SeqCst) == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    let received = seen.load(Ordering::SeqCst);
    assert!(received > 0, "2 秒内没收到 TX_COMPLETED 回调");
    println!("模块能力位：{:?}", dev.module_features());
    dev.close();
}

/// 500 号用例：两台模块都在位时，句柄表能区分它们的型号与序列号。
#[test]
#[ignore = "需要接两台 USB2CANFD 模块"]
fn two_devices_are_enumerated() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();
    assert!(count >= 2, "需要两台设备，当前句柄表里只有 {count} 台");

    for index in 0..count {
        let dev = ctx.device(index).expect("取设备");
        let info = dev.info().expect("读枚举信息");
        println!(
            "设备[{index}]：{:?} VID=0x{:04X} PID=0x{:04X} 序列号={:?} 在位={:?}",
            info.device,
            info.vid,
            info.pid,
            info.serial,
            dev.is_present().expect("查在位状态")
        );
        // 顺便把固件版本与构建标识记下来（报告要可溯源）。
        if dev.open().is_ok() {
            match dev.firmware_info() {
                Ok(firmware) => println!(
                    "  固件：{} 版本有效={} 构建标识={:?}",
                    firmware
                        .to_string_line()
                        .unwrap_or_else(|error| format!("（格式化失败：{error}）")),
                    firmware.version_valid,
                    firmware.build_id
                ),
                Err(error) => println!("  读固件信息失败：{error}"),
            }
            dev.close();
        }
    }
}

///
/// 单独拎出来是因为它是"通道可用性"的最小判据：某些失败的下发会把通道打成不可用
/// （见 `dmcan.h` 里 `set_channel_baudrate` 的 @note）。
///
/// 判定口径只认**设备侧的可观测状态**（周期上报里的 `enabled`）：两台在测设备上
/// `dmcan_device_disable_channel()` 都返回 `false`（→ `Err(Io)`），但设备侧确实关掉了通道，
/// 所以本用例断言"关掉后 enabled 变 false、重新使能后变 true"，并把返回值打印出来。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn channel_enable_disable_roundtrip() {
    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");

    let mut enabled_after_enable = None;
    let mut enabled_after_disable = None;
    for round in 1..=2 {
        dev.enable_channel(0).expect("使能通道 0");
        wait_baud_reload();
        enabled_after_enable = dev
            .channel_stats(0)
            .ok()
            .flatten()
            .map(|stats| stats.enabled);
        let disable_return = dev.disable_channel(0);
        wait_baud_reload();
        enabled_after_disable = dev
            .channel_stats(0)
            .ok()
            .flatten()
            .map(|stats| stats.enabled);
        println!(
            "第 {round} 轮：使能后 enabled={enabled_after_enable:?}；disable 返回={disable_return:?}；\
             关闭后 enabled={enabled_after_disable:?}"
        );
    }

    dev.close();
    assert_eq!(enabled_after_enable, Some(true), "使能后设备侧仍不是使能态");
    assert_eq!(enabled_after_disable, Some(false), "关闭后设备侧仍是使能态");
}

/// 070 号用例：逐步骤走查"通道是不是一直使能着"。
///
/// 每一步之后都等一条周期上报（每秒一条）再读 `channel_stats.enabled`，
/// 用来定位是哪一次下发把通道打成 `enabled = false` 的。只做观测，不做断言。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn channel_state_walkthrough() {
    fn show(dev: &dmcan::Device, tag: &str) {
        std::thread::sleep(Duration::from_millis(1400));
        match dev.channel_stats(0) {
            Ok(Some(stats)) => println!(
                "{tag}：enabled={} admitted={} completed={} failed={} dropped={} ch_disabled={} rx={}",
                stats.enabled,
                stats.tx_admitted,
                stats.tx_completed,
                stats.tx_failed,
                stats.tx_dropped,
                stats.tx_dropped_ch_disabled,
                stats.rx_received
            ),
            other => println!("{tag}：读通道统计={other:?}"),
        }
    }

    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");
    show(&dev, "0 刚打开");

    println!("enable_channel → {:?}", dev.enable_channel(0));
    show(&dev, "1 使能后");

    println!(
        "set_channel_baudrate(1M/4M) → {:?}",
        dev.set_channel_baudrate(
            0,
            &ChannelBaudInfo {
                canfd: true,
                retrans: true,
                can_baudrate: 1_000_000,
                canfd_baudrate: 4_000_000,
                ..Default::default()
            }
        )
    );
    show(&dev, "2 配波特率后");

    let mut conf = dev.channel_baudrate_details(0).expect("读回完整配置");
    println!(
        "完整配置：flags=0x{:02X} mode={:?} nom(brp={} seg1={} seg2={} sjw={}) data(brp={} seg1={} seg2={} sjw={}) busoff_autoreset_ms={} tdc(off={} filt={})",
        conf.raw_flags(),
        conf.mode(),
        conf.nominal_brp,
        conf.nominal_seg1,
        conf.nominal_seg2,
        conf.nominal_sjw,
        conf.data_brp,
        conf.data_seg1,
        conf.data_seg2,
        conf.data_sjw,
        conf.busoff_autoreset_ms,
        conf.tdc_offset,
        conf.tdc_filter
    );

    conf.set_mode(ChannelMode::LoopbackExternal);
    println!(
        "set_channel_baudrate_details(mode=LoopbackExternal) → {:?}",
        dev.set_channel_baudrate_details(0, &conf)
    );
    show(&dev, "3 写回环模式后");

    let send = dev.send_can(
        0,
        &Frame::fd(0x5A5, &[0xAA, 0xBB, 0xCC], true).expect("构造帧"),
    );
    println!("发送 → {send:?}");
    show(&dev, "4 发送后");

    // 恢复现场（best-effort）
    conf.set_mode(ChannelMode::Normal);
    println!(
        "恢复 mode=Normal → {:?}",
        dev.set_channel_baudrate_details(0, &conf)
    );
    println!("重新使能 → {:?}", dev.enable_channel(0));
    dev.close();
}

/// 700 号用例：两台模块接在同一条总线上时，A 发 B 收（跨设备收帧）。
/// 800 号用例：真 echo 的顺序——同一帧必须先给假 echo（`ack=0`）、再给真 echo（`ack=1`）。
///
/// 需要总线上有对端 ACK（本台架 S 接在 D 的通道 0），否则真 echo 不会来。
#[test]
#[ignore = "需要 D 接 S 的台架（对端要能 ACK）"]
fn echo_order_fake_then_real() {
    let ctx = Context::new().expect("创建上下文");
    let (d_dev, s_dev) = find_d_and_s(&ctx);
    setup_pair(&d_dev, &s_dev);

    d_dev.set_channel_echo(0, true).expect("开真 echo");

    let events = Arc::new(std::sync::Mutex::new(
        Vec::<(&'static str, u32, bool)>::new(),
    ));
    let recorder = Arc::clone(&events);
    let sub = d_dev
        .subscribe(
            Events::TX_COMPLETED | Events::TX_ON_BUS,
            CHANNEL_MASK_ALL,
            move |event| match event {
                Event::TxCompleted(frame) => {
                    recorder
                        .lock()
                        .expect("锁")
                        .push(("fake", frame.can_id(), frame.ack()))
                }
                Event::TxOnBus(frame) => {
                    recorder
                        .lock()
                        .expect("锁")
                        .push(("real", frame.can_id(), frame.ack()))
                }
                _ => {}
            },
        )
        .expect("订阅收发事件");

    let frame = Frame::fd(0x1A5, &[0xA1, 0xA2, 0xA3], true).expect("构造帧");
    d_dev.send_can(0, &frame).expect("发送");

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        let seen = events.lock().expect("锁");
        let has_fake = seen.iter().any(|(kind, _, _)| *kind == "fake");
        let has_real = seen.iter().any(|(kind, _, _)| *kind == "real");
        drop(seen);
        if has_fake && has_real {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    let seen = events.lock().expect("锁").clone();
    let fake_index = seen.iter().position(|(kind, _, _)| *kind == "fake");
    let real_index = seen.iter().position(|(kind, _, _)| *kind == "real");
    println!("事件顺序：{seen:?}");

    d_dev.set_channel_echo(0, false).expect("关真 echo");
    drop(sub);
    d_dev.close();
    s_dev.close();

    assert!(fake_index.is_some(), "没收到假 echo（ack=0）");
    assert!(real_index.is_some(), "没收到真 echo（ack=1）");
    assert!(
        fake_index.unwrap() < real_index.unwrap(),
        "真 echo 早于假 echo，顺序不对"
    );
    assert!(
        seen.iter().all(|(_, id, _)| *id == 0x1A5),
        "回调里出现了别的 ID：{seen:?}"
    );
    assert!(
        seen.iter().any(|(kind, _, ack)| *kind == "real" && *ack),
        "真 echo 的 ack 不是 true"
    );
}

/// 810 号用例：周期发送 + `stop_send`——停止后对端不再收到新帧。
/// 820 号用例：硬件过滤表——黑名单挡一条、白名单只放行一条、回读校验、最后清空。
///
/// 改表规矩（C 侧）：**必须先关通道**，下发后等固件重载（≤1.2 s）再重新使能。
#[test]
#[ignore = "需要 D 接 S 的台架"]
fn channel_filter_blacklist_and_whitelist() {
    use dmcan::{FilterAction, FilterIdType, FilterMode, FilterRule, FilterRuleType};

    let ctx = Context::new().expect("创建上下文");
    let (d_dev, s_dev) = find_d_and_s(&ctx);
    setup_pair(&d_dev, &s_dev);

    let count = Arc::new(AtomicUsize::new(0));
    let ids = Arc::new(std::sync::Mutex::new(Vec::<u32>::new()));
    let counter = Arc::clone(&count);
    let recorder = Arc::clone(&ids);
    let sub = s_dev
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
            if let Event::RxFrame(frame) = event {
                counter.fetch_add(1, Ordering::SeqCst);
                recorder.lock().expect("锁").push(frame.can_id());
            }
        })
        .expect("S 订阅 RX_FRAME");

    let send_two = |dev: &dmcan::Device| {
        dev.send_can(0, &Frame::classic(0x321, &[0x01]).expect("构造帧"))
            .expect("发 0x321");
        dev.send_can(0, &Frame::classic(0x123, &[0x02]).expect("构造帧"))
            .expect("发 0x123");
        std::thread::sleep(Duration::from_millis(600));
        let mut seen = ids.lock().expect("锁");
        let snapshot = seen.clone();
        seen.clear();
        snapshot
    };

    // 基线：两个 ID 都该收到
    let baseline = send_two(&d_dev);
    println!("基线收到：{baseline:?}");

    let reject_321 = [FilterRule {
        id_type: FilterIdType::Standard,
        rule_type: FilterRuleType::Range,
        action: FilterAction::Reject,
        id1: 0x321,
        id2: 0x321,
    }];
    s_dev.disable_channel(0).expect("关通道（改表前）");
    let usage = s_dev
        .set_channel_filter(0, FilterMode::Blacklist, &reject_321)
        .expect("下发黑名单");
    println!("黑名单用量：{usage:?}");
    wait_baud_reload();
    assert!(
        enable_until_active(&s_dev, 0, "S（黑名单后）"),
        "S 没使能上"
    );

    let blacklist_seen = send_two(&d_dev);
    println!("黑名单后收到：{blacklist_seen:?}");

    let allow_321 = [FilterRule {
        id_type: FilterIdType::Standard,
        rule_type: FilterRuleType::Range,
        action: FilterAction::Fifo0,
        id1: 0x321,
        id2: 0x321,
    }];
    println!("[步骤] 关通道（白名单前）");
    s_dev.disable_channel(0).expect("关通道（改表前）");
    println!("[步骤] 下发白名单");
    s_dev
        .set_channel_filter(0, FilterMode::Whitelist, &allow_321)
        .expect("下发白名单");
    println!("[步骤] 等重载");
    wait_baud_reload();
    println!("[步骤] 重新使能");
    assert!(
        enable_until_active(&s_dev, 0, "S（白名单后）"),
        "S 没使能上"
    );
    println!("[步骤] 回读过滤表");

    let table = s_dev.channel_filter(0).expect("回读过滤表");
    println!(
        "回读：mode={:?} count={} std_max={} ext_max={} 首条={:?}",
        table.mode,
        table.rules.len(),
        table.std_max,
        table.ext_max,
        table.rules.first()
    );

    let whitelist_seen = send_two(&d_dev);
    println!("白名单后收到：{whitelist_seen:?}");

    // 清空：黑名单 + 空表 = 全接受
    s_dev.disable_channel(0).expect("关通道（清表前）");
    s_dev
        .set_channel_filter(0, FilterMode::Blacklist, &[])
        .expect("清空过滤表");
    wait_baud_reload();
    assert!(enable_until_active(&s_dev, 0, "S（清表后）"), "S 没使能上");
    let cleared = s_dev.channel_filter(0).expect("回读过滤表（清空后）");
    let after_clear = send_two(&d_dev);
    println!(
        "清空后收到：{after_clear:?}（表里 count={}）",
        cleared.rules.len()
    );

    drop(sub);
    d_dev.close();
    s_dev.close();

    assert!(
        baseline.contains(&0x321) && baseline.contains(&0x123),
        "基线两个 ID 都该收到：{baseline:?}"
    );
    assert!(
        !blacklist_seen.contains(&0x321) && blacklist_seen.contains(&0x123),
        "黑名单应挡住 0x321、放行 0x123：{blacklist_seen:?}"
    );
    assert!(
        whitelist_seen.contains(&0x321) && !whitelist_seen.contains(&0x123),
        "白名单应只放行 0x321：{whitelist_seen:?}"
    );
    assert_eq!(cleared.rules.len(), 0, "清空后表里应没有规则");
    assert!(
        after_clear.contains(&0x321) && after_clear.contains(&0x123),
        "清空后应恢复全接受：{after_clear:?}"
    );
}

/// 830 号用例：单模块"外部回环"自发自收（`ChannelMode::LoopbackExternal`）。
/// 840 号用例：看门狗保活与关狗——开着狗持续喂狗时通道不被咬，关狗后也不被咬。
///
/// 对应 C 侧两处修复：① 喂狗周期改为超时的一半（原来按整周期喂、零余量，每个周期必被咬）；
/// ② 关狗命令不再被固件按非法参数拒掉（需要固件 `d64e840` 起）。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn watchdog_keeps_channel_alive_then_disarms() {
    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");
    configure_fd(&dev, 0);
    wait_baud_reload();
    assert!(enable_until_active(&dev, 0, "被测设备"), "通道没使能上");

    // 武装看门狗（超时 1000 ms，SDK 内部按 500 ms 喂狗）。
    println!("使能看门狗 → {:?}", dev.enable_watchdog(1000, true));
    std::thread::sleep(Duration::from_millis(2500));
    let during = dev.channel_stats(0).ok().flatten().map(|s| s.enabled);
    println!("武装 2.5 s 后（跨了 2 个超时周期）：enabled={during:?}");

    // 关狗：新固件应当受理。
    let disarm = dev.enable_watchdog(1000, false);
    println!("关闭看门狗 → {disarm:?}");
    std::thread::sleep(Duration::from_millis(2500));
    let after = dev.channel_stats(0).ok().flatten();
    println!(
        "关狗 2.5 s 后：enabled={:?} ch_disabled={:?}",
        after.as_ref().map(|s| s.enabled),
        after.as_ref().map(|s| s.tx_dropped_ch_disabled)
    );

    dev.close();
    assert_eq!(during, Some(true), "喂狗期间通道被咬（enabled 变 false）");
    assert!(disarm.is_ok(), "关狗被拒（需要固件 d64e840 起）");
    assert_eq!(
        after.map(|s| s.enabled),
        Some(true),
        "关狗之后通道又被咬了 —— 若狗没关掉，请给设备断电一次再跑"
    );
}

/// 850 号用例：UART 回环——写进去的字节要原样读回来。
///
/// 台架前提：**被测模块的 UART TX–RX 要短接**。本台架短接的是 **S**（`COM24`），
/// 所以这里优先取 S；只有一台设备时用它自己。
#[test]
#[ignore = "需要模块的 UART TX–RX 短接"]
fn uart_loopback_echo() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();
    assert!(count > 0, "没有找到设备");
    let dev = if count >= 2 {
        find_d_and_s(&ctx).1
    } else {
        ctx.device(0).expect("取设备")
    };
    dev.open().expect("打开设备");

    let is_open = dev.uart_is_open();
    println!("UART 端口：{:?} 名字={:?}", is_open, dev.uart_port_name());
    assert_eq!(
        is_open.ok(),
        Some(true),
        "UART 没打开（open() 默认会自动打开）"
    );

    let payload: Vec<u8> = (0..16u8).collect();
    let received = Arc::new(std::sync::Mutex::new(Vec::<u8>::new()));
    let recorder = Arc::clone(&received);
    let sub = dev
        .subscribe(Events::UART_RX, CHANNEL_MASK_ALL, move |event| {
            if let Event::UartRx(data) = event {
                recorder.lock().expect("锁").extend_from_slice(data);
            }
        })
        .expect("订阅 UART_RX");

    dev.uart_write(&payload).expect("写 UART");
    let deadline = Instant::now() + Duration::from_secs(2);
    while received.lock().expect("锁").len() < payload.len() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    let echoed = received.lock().expect("锁").clone();
    println!(
        "写 {:?}，回读 {:?}；统计 {:?}",
        payload,
        echoed,
        dev.uart_stats()
    );

    drop(sub);
    dev.close();
    assert_eq!(
        echoed, payload,
        "UART 回读与写入不一致：确认被测模块的 UART TX–RX 已短接（本台架短接的是 S，即 index 1）"
    );
}

/// 860 号用例：通道扩展配置的读写与 `save_param`（自动部分；**断电复验需要人工**）。
///
/// 自动部分：读回 → 改 `busoff_autoreset_ms` → 读回校验 → `save_param()` → 还原并再次保存。
/// 断电后是否保留要人工做：写完值、给设备断电、重新上电再读（见用例文档 860）。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn channel_ext_config_and_save_param() {
    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");

    let original = dev.channel_ext_config(0).expect("读扩展配置");
    println!("原配置：{original:?}");
    let target = if original.busoff_autoreset_ms == 1300 {
        1200
    } else {
        1300
    };

    let mut modified = original;
    modified.busoff_autoreset_ms = target;
    dev.set_channel_ext_config(0, &modified)
        .expect("写扩展配置");
    wait_baud_reload();
    let readback = dev.channel_ext_config(0).expect("复读扩展配置");
    println!("写入 {target} 后读回：{readback:?}");
    println!("save_param → {:?}", dev.save_param());

    dev.set_channel_ext_config(0, &original)
        .expect("还原扩展配置");
    wait_baud_reload();
    let restored = dev.channel_ext_config(0).expect("复读（还原后）");
    println!("还原后：{restored:?}");
    println!("再次 save_param → {:?}", dev.save_param());

    dev.close();
    assert_eq!(readback.busoff_autoreset_ms, target, "写进去的值没读回来");
    assert_eq!(
        restored.busoff_autoreset_ms, original.busoff_autoreset_ms,
        "没还原成原值"
    );
}

///
/// 用 **D 的通道 1**（没有接线），避免与 S 那条总线互相干扰。
/// 顺序按 C 侧规矩：**关通道 → 改节点模式 → 等重载 → 再使能**。
#[test]
#[ignore = "需要 USB2CANFD_D（用它的通道 1 自测）"]
fn external_loopback_self_receive() {
    let ctx = Context::new().expect("创建上下文");
    let (d_dev, s_dev) = find_d_and_s(&ctx);
    d_dev.open().expect("打开 D");
    d_dev.enable_watchdog(1000, false).ok();
    configure_fd(&d_dev, 1);
    wait_baud_reload();

    let mut conf = d_dev.channel_baudrate_details(1).expect("读回通道 1 配置");
    println!(
        "通道 1 原配置：mode={:?} nominal_brp={} data_brp={}",
        conf.mode(),
        conf.nominal_brp,
        conf.data_brp
    );
    assert!(conf.data_brp != 0, "数据段位定时是 0，不能拿它回写");

    d_dev.disable_channel(1).ok();
    conf.set_mode(ChannelMode::LoopbackExternal);
    d_dev
        .set_channel_baudrate_details(1, &conf)
        .expect("写外部回环模式");
    wait_baud_reload();
    assert!(
        enable_until_active(&d_dev, 1, "D ch1（外部回环）"),
        "D ch1 没使能上"
    );
    println!(
        "模式回读：{:?}",
        d_dev.channel_baudrate_details(1).map(|c| c.mode())
    );

    let seen = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&seen);
    let sub = d_dev
        .subscribe(Events::RX_FRAME, 1 << 1, move |event| {
            if let Event::RxFrame(frame) = event {
                assert_eq!(frame.can_id(), 0x3C1);
                assert_eq!(frame.channel(), 1);
                counter.fetch_add(1, Ordering::SeqCst);
            }
        })
        .expect("订阅 ch1 收帧");

    d_dev
        .send_can(1, &Frame::fd(0x3C1, &[0x31, 0x32], true).expect("构造帧"))
        .expect("发送");
    let deadline = Instant::now() + Duration::from_secs(2);
    while seen.load(Ordering::SeqCst) == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    let received = seen.load(Ordering::SeqCst);
    println!(
        "外部回环收到 {received} 帧；通道 1 统计：{:?}",
        d_dev.channel_stats(1)
    );

    // 恢复现场：关通道 → 写回正常模式 → 等重载 → 使能
    d_dev.disable_channel(1).ok();
    conf.set_mode(ChannelMode::Normal);
    d_dev
        .set_channel_baudrate_details(1, &conf)
        .expect("恢复 mode=Normal");
    wait_baud_reload();
    let restored = d_dev.channel_baudrate_details(1).map(|c| c.mode());
    println!("恢复后模式：{restored:?}");

    drop(sub);
    d_dev.close();
    s_dev.close();

    assert!(received > 0, "外部回环下没收到自己发的帧");
    assert_eq!(
        restored.ok(),
        Some(ChannelMode::Normal),
        "节点模式没恢复回 Normal"
    );
}

#[test]
#[ignore = "需要 D 接 S 的台架"]
fn periodic_send_then_stop() {
    let ctx = Context::new().expect("创建上下文");
    let (d_dev, s_dev) = find_d_and_s(&ctx);
    setup_pair(&d_dev, &s_dev);

    let count = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&count);
    let sub = s_dev
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
            if let Event::RxFrame(frame) = event {
                if frame.can_id() == 0x2B1 {
                    counter.fetch_add(1, Ordering::SeqCst);
                }
            }
        })
        .expect("S 订阅 RX_FRAME");

    // `send_times = 0` 表示一直发，靠 `stop_send` 停。
    d_dev
        .send_can_periodic(
            0,
            20,
            0,
            &Frame::fd(0x2B1, &[0x21, 0x22], true).expect("构造帧"),
        )
        .expect("启动周期发送");
    std::thread::sleep(Duration::from_millis(600));
    let before_stop = count.load(Ordering::SeqCst);

    d_dev.stop_send(0).expect("停止周期发送");
    std::thread::sleep(Duration::from_millis(200));
    let after_stop_settle = count.load(Ordering::SeqCst);
    std::thread::sleep(Duration::from_millis(500));
    let after_stop_final = count.load(Ordering::SeqCst);
    println!(
        "周期发送：stop 前 {before_stop} 帧，stop 后立刻 {after_stop_settle} 帧，再等 500 ms 后 {after_stop_final} 帧"
    );

    drop(sub);
    d_dev.close();
    s_dev.close();

    assert!(before_stop > 0, "周期发送期间对端一帧都没收到");
    assert_eq!(
        after_stop_final, after_stop_settle,
        "stop_send 之后对端还在继续收到帧"
    );
}

///
/// 台架前提（当前台架就是这样接的）：**S 模块接在 D 模块的通道 0 上**，
/// 并且**要打开 S 侧的软件终端电阻**（`enable_can_res`），否则差分总线没有端接、通信不可靠。
#[test]
#[ignore = "需要 D 与 S 两台模块接在同一条 CAN 总线上（S 接 D 的通道 0）"]
fn cross_device_receive_between_two_modules() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();
    assert!(count >= 2, "需要两台设备，当前句柄表里只有 {count} 台");

    // 按型号认设备，不依赖枚举顺序（C 侧也多处强调"多台同型号设备必须按序列号认"）。
    let mut sender = None;
    let mut receiver = None;
    for index in 0..count {
        let dev = ctx.device(index).expect("取设备");
        match dev.info().expect("读枚举信息").device {
            dmcan::DeviceType::Usb2CanfdD => sender = Some(dev),
            dmcan::DeviceType::Usb2CanfdS => receiver = Some(dev),
            other => println!("忽略设备[{index}]（{other:?}）"),
        }
    }
    let sender = sender.expect("没找到 USB2CANFD_D（发送方）");
    let receiver = receiver.expect("没找到 USB2CANFD_S（接收方）");

    for (role, dev) in [("发送方 D", &sender), ("接收方 S", &receiver)] {
        let info = dev.info().expect("读枚举信息");
        println!("{role}：{:?} 序列号={:?}", info.device, info.serial);
        dev.open().expect("打开设备");
        // 关看门狗：看门狗咬会关掉通道（`tx_dropped_ch_disabled` 的两个来源之一），
        // 台架上没有人在喂狗，留着它只会把通道咬掉。
        println!("{role} 关看门狗 → {:?}", dev.enable_watchdog(0, false));
        configure_fd(dev, 0);
    }
    wait_baud_reload();
    assert!(
        enable_until_active(&sender, 0, "发送方 D"),
        "发送方 D 通道 0 一直没使能上"
    );
    assert!(
        enable_until_active(&receiver, 0, "接收方 S"),
        "接收方 S 通道 0 一直没使能上"
    );

    // 接收侧开软件终端电阻：差分总线两端要有端接，否则波形反射会导致收不到/误帧。
    println!("接收方能力位：{:?}", receiver.module_features());
    println!("接收方开终端电阻 → {:?}", receiver.enable_can_res(0, true));
    wait_baud_reload();
    println!("接收方终端电阻状态：{:?}", receiver.can_res(0));
    println!("发送方终端电阻状态：{:?}", sender.can_res(0));
    println!(
        "双方总线状态：D={:?} S={:?}",
        sender.bus_state(0),
        receiver.bus_state(0)
    );

    let seen = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&seen);
    let _sub = receiver
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
            if let Event::RxFrame(frame) = event {
                assert_eq!(frame.can_id(), 0x7A5);
                counter.fetch_add(1, Ordering::SeqCst);
            }
        })
        .expect("订阅 RX_FRAME");

    for _ in 0..3 {
        sender
            .send_can(
                0,
                &Frame::fd(0x7A5, &[0x11, 0x22, 0x33, 0x44], true).expect("构造帧"),
            )
            .expect("发送");
    }
    std::thread::sleep(Duration::from_millis(500));

    let deadline = Instant::now() + Duration::from_secs(2);
    while seen.load(Ordering::SeqCst) == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    let received = seen.load(Ordering::SeqCst);

    println!("发送方总线状态：{:?}", sender.bus_state(0));
    println!("发送方通道统计：{:?}", sender.channel_stats(0));
    println!("接收方通道统计：{:?}", receiver.channel_stats(0));

    // 收尾：终端电阻保持开启（台架要用），只把会话关掉。
    // 注销订阅与 close 的顺序随意（早前"反过来会 AV"的结论已更正为封装自身的 use-after-free）。
    drop(_sub);
    sender.close();
    receiver.close();

    assert!(
        received > 0,
        "接收方 2 秒内没收到发送方的帧（接线 / 终端电阻 / 波特率？）"
    );
    println!("接收方收到 {received} 帧");
}

/// 730 号诊断用例（临时）：D 的发送窗口——使能后立刻发一帧，看 `tx_completed` 有没有涨。
/// 740 号诊断用例（临时）：两台设备都用最宽容的经典 CAN 500 kbps，双向各发一次。
/// 750 号诊断用例（临时）：订阅注销顺序与 close/open 的组合。
///
/// 背景：早前记录过"`close()` 之后再注销订阅会 AV"，后来查清那是**封装自己的
/// use-after-free**（trampoline 里 `Arc::from_raw` 没先 increment），已在 `src/subscribe.rs` 修掉。
/// 本用例把这个边界固定下来，两条路径都应当正常：
/// ① subscribe → close → **open** → 注销（文档承诺"订阅跨 close/open 保持"）；
/// ② `DMCAN_PROBE_ORDER=close-then-unsubscribe` 时的 subscribe → close → 注销。
///
/// 跑法：`cargo test --test hardware_test unsubscribe_after_close_probe -- --ignored --nocapture`
#[test]
#[ignore = "诊断用"]
fn unsubscribe_after_close_probe() {
    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");

    let sub = dev
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, |_| {})
        .expect("订阅");

    // `DMCAN_PROBE_ORDER=close-then-unsubscribe` 时走"先 close 再注销"这条曾经 AV 的路径。
    if std::env::var("DMCAN_PROBE_ORDER").as_deref() == Ok("close-then-unsubscribe") {
        dev.close();
        drop(sub);
        println!("close → 注销：没崩");
        return;
    }

    dev.close();
    println!("已 close，接下来 open 再注销订阅");
    dev.open().expect("重新打开");
    std::thread::sleep(Duration::from_millis(300));
    drop(sub);
    println!("close → open → 注销：没崩");
    dev.close();
    println!("结束");
}

///
/// 用来排除"高速率 CAN FD + 端接不良导致信号完整性不够"这一类台架问题：
/// 经典 CAN 500 kbps 对线长/端接最宽容，能通就说明接线与端接基本可用。
#[test]
#[ignore = "诊断用：需要 D 与 S 接线"]
fn classic_can_bidirectional_probe() {
    let ctx = Context::new().expect("创建上下文");
    let count = ctx.find_devices();
    assert!(count >= 2, "需要两台设备");

    let mut d_dev = None;
    let mut s_dev = None;
    for index in 0..count {
        let dev = ctx.device(index).expect("取设备");
        match dev.info().expect("读枚举信息").device {
            dmcan::DeviceType::Usb2CanfdD => d_dev = Some(dev),
            dmcan::DeviceType::Usb2CanfdS => s_dev = Some(dev),
            _ => {}
        }
    }
    let d_dev = d_dev.expect("没找到 D");
    let s_dev = s_dev.expect("没找到 S");
    d_dev.open().expect("打开 D");
    s_dev.open().expect("打开 S");

    // 经典 CAN 500 kbps：canfd = false（SDK 会保留设备当前的数据段位定时）。
    for (role, dev) in [("D", &d_dev), ("S", &s_dev)] {
        let result = dev.set_channel_baudrate(
            0,
            &ChannelBaudInfo {
                canfd: false,
                retrans: true,
                can_baudrate: 500_000,
                can_sp: 0.875,
                ..Default::default()
            },
        );
        println!("{role} 配经典 CAN 500k → {result:?}");
    }
    wait_baud_reload();
    println!("D 使能 → {:?}", d_dev.enable_channel(0));
    println!("S 使能 → {:?}", s_dev.enable_channel(0));
    println!("S 开终端电阻 → {:?}", s_dev.enable_can_res(0, true));
    std::thread::sleep(Duration::from_millis(400));

    // D 收
    let d_seen = Arc::new(std::sync::Mutex::new(Vec::<u32>::new()));
    let d_recorder = Arc::clone(&d_seen);
    let _d_sub = d_dev
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
            if let Event::RxFrame(frame) = event {
                d_recorder.lock().expect("锁").push(frame.can_id());
            }
        })
        .expect("D 订阅");
    // S 收
    let s_seen = Arc::new(std::sync::Mutex::new(Vec::<u32>::new()));
    let s_recorder = Arc::clone(&s_seen);
    let _s_sub = s_dev
        .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
            if let Event::RxFrame(frame) = event {
                s_recorder.lock().expect("锁").push(frame.can_id());
            }
        })
        .expect("S 订阅");

    let before_d = d_dev.channel_stats(0).ok().flatten();
    let before_s = s_dev.channel_stats(0).ok().flatten();
    for _ in 0..3 {
        println!(
            "D 发 → {:?}",
            d_dev.send_can(0, &Frame::classic(0x123, &[0x11, 0x22]).expect("构造帧"))
        );
        println!(
            "S 发 → {:?}",
            s_dev.send_can(0, &Frame::classic(0x456, &[0x33, 0x44]).expect("构造帧"))
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    std::thread::sleep(Duration::from_millis(1200));

    let after_d = d_dev.channel_stats(0).ok().flatten();
    let after_s = s_dev.channel_stats(0).ok().flatten();
    println!(
        "D 收到：{:?}（S 发出 3 帧 0x456）",
        d_seen.lock().expect("锁")
    );
    println!(
        "S 收到：{:?}（D 发出 3 帧 0x123）",
        s_seen.lock().expect("锁")
    );
    println!(
        "D：admitted {:?}→{:?} completed {:?}→{:?} failed {:?}→{:?} enabled {:?}→{:?}",
        before_d.as_ref().map(|s| s.tx_admitted),
        after_d.as_ref().map(|s| s.tx_admitted),
        before_d.as_ref().map(|s| s.tx_completed),
        after_d.as_ref().map(|s| s.tx_completed),
        before_d.as_ref().map(|s| s.tx_failed),
        after_d.as_ref().map(|s| s.tx_failed),
        before_d.as_ref().map(|s| s.enabled),
        after_d.as_ref().map(|s| s.enabled)
    );
    println!(
        "S：admitted {:?}→{:?} completed {:?}→{:?} failed {:?}→{:?} enabled {:?}→{:?}",
        before_s.as_ref().map(|s| s.tx_admitted),
        after_s.as_ref().map(|s| s.tx_admitted),
        before_s.as_ref().map(|s| s.tx_completed),
        after_s.as_ref().map(|s| s.tx_completed),
        before_s.as_ref().map(|s| s.tx_failed),
        after_s.as_ref().map(|s| s.tx_failed),
        before_s.as_ref().map(|s| s.enabled),
        after_s.as_ref().map(|s| s.enabled)
    );
    println!(
        "D 总线：{:?}  S 总线：{:?}",
        d_dev.bus_state(0),
        s_dev.bus_state(0)
    );

    let d_received = d_seen.lock().expect("锁").len();
    let s_received = s_seen.lock().expect("锁").len();
    // 收尾：注销订阅后关设备（顺序随意，这里只是按常规先注销）。
    drop(_d_sub);
    drop(_s_sub);
    d_dev.close();
    s_dev.close();
    assert!(s_received > 0, "S 没收到 D 的帧（经典 CAN 500k 双向）");
    assert!(d_received > 0, "D 没收到 S 的帧（经典 CAN 500k 双向）");
}

///
/// 用来区分两种情况：① 通道使能后能正常发（那前面的失败是"使能后过一会儿被关掉"的时序问题）；
/// ② 使能后也发不出去（那是 TX 通路被卡住，需要复位设备）。
#[test]
#[ignore = "诊断用"]
fn send_window_probe() {
    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    dev.open().expect("打开设备");
    println!("使能前：{:?}", dev.channel_stats(0));

    // 先写一次参数，借固件的重载把历史长跑卡住的帧冲掉（实测会看到 tx_completed 跳一截）。
    configure_fd(&dev, 0);
    wait_baud_reload();
    println!("重载后：{:?}", dev.channel_stats(0));

    for round in 1..=3 {
        let enable = dev.enable_channel(0);
        std::thread::sleep(Duration::from_millis(200));
        let before = dev.channel_stats(0).ok().flatten();
        let send = dev.send_can(0, &Frame::fd(0x9C1, &[0x01, 0x02], true).expect("构造帧"));
        std::thread::sleep(Duration::from_millis(800));
        let after = dev.channel_stats(0).ok().flatten();
        println!(
            "第 {round} 轮：enable={enable:?} send={send:?} 发前(admitted={:?} completed={:?} enabled={:?}) \
             发后(admitted={:?} completed={:?} failed={:?} ch_disabled={:?} enabled={:?}) 总线={:?}",
            before.as_ref().map(|s| s.tx_admitted),
            before.as_ref().map(|s| s.tx_completed),
            before.as_ref().map(|s| s.enabled),
            after.as_ref().map(|s| s.tx_admitted),
            after.as_ref().map(|s| s.tx_completed),
            after.as_ref().map(|s| s.tx_failed),
            after.as_ref().map(|s| s.tx_dropped_ch_disabled),
            after.as_ref().map(|s| s.enabled),
            dev.bus_state(0)
        );
    }
    dev.close();
}

/// 200 号用例：回调里 panic 不会穿到 C ABI（被吞掉并计数），进程与后续收发不受影响。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn callback_panic_is_contained() {
    let ctx = Context::new().expect("创建上下文");
    assert!(ctx.find_devices() > 0, "没有找到设备");
    let dev = ctx.device(0).expect("取设备 0");
    dev.open().expect("打开设备");
    // 先配波特率、再使能（配波特率会让固件把通道置回未使能）。
    configure_fd(&dev, 0);
    dev.enable_channel(0).expect("使能通道 0");
    wait_baud_reload();

    let mut sub = dev
        .subscribe(Events::TX_COMPLETED, CHANNEL_MASK_ALL, |_| {
            panic!("故意在订阅回调里 panic");
        })
        .expect("订阅 TX_COMPLETED");

    let frame = Frame::classic(0x7FF, &[0x11]).expect("构造帧");
    dev.send_can(0, &frame).expect("发送");
    std::thread::sleep(Duration::from_millis(300));
    assert!(sub.panic_count() > 0, "panic 没有被包装层记录");

    // 进程还在、还能继续干活：注销订阅后再发一帧。
    assert!(sub.unsubscribe());
    dev.send_can(0, &frame).expect("再发一帧");

    // 诊断接口仍然可读
    let bus = dev.bus_state(0).expect("读总线状态");
    println!("总线状态：{:?}", bus);
    dev.close();
}

/// 300 号用例：读总线状态；若落在 bus-off 就清一次并复核（对应 C++ 侧的 busoff 恢复探针）。
#[test]
#[ignore = "需要接 USB2CANFD 模块"]
fn bus_state_and_recovery() {
    let ctx = Context::new().expect("创建上下文");
    assert!(ctx.find_devices() > 0, "没有找到设备");
    let dev = ctx.device(0).expect("取设备 0");
    dev.open().expect("打开设备");

    let before = dev.bus_state(0).expect("读总线状态");
    println!("恢复前：{:?}", before);
    if before.bus_state == 3 {
        dev.clear_bus_off(0).expect("清 bus-off");
        std::thread::sleep(Duration::from_millis(1200));
        let after = dev.bus_state(0).expect("复读总线状态");
        println!("恢复后：{:?}", after);
        assert_ne!(after.bus_state, 3, "清 bus-off 之后仍然是 bus-off");
    }

    if let Some(stats) = dev.channel_stats(0).expect("读通道统计") {
        println!(
            "通道 0：tx_admitted={} tx_completed={} tx_failed={} rx_received={} rx_dropped={} 细分位=0x{:02X}",
            stats.tx_admitted,
            stats.tx_completed,
            stats.tx_failed,
            stats.rx_received,
            stats.rx_dropped,
            stats.drop_flags
        );
    }
    dev.close();
}

/// 870 号诊断用例（人工）：热插拔回调——运行期间手动拔下再插回任意一台设备。
/// 895 号诊断用例：上下文的创建 / 销毁生命周期（macOS 上发现每次创建都会"隔一次失败"）。
/// 896 号诊断用例：设备开关与上下文创建的相互影响（定位 macOS 上"隔次创建失败"的触发条件）。
///
/// 三组对照：① 开设备（默认自动开 UART）→ 关 → 丢掉上下文；② 开设备但**不开 UART** → 关；
/// ③ 只枚举设备、不开。每轮都新建上下文并打印结果。
#[test]
#[ignore = "诊断用：需要接 USB2CANFD 模块"]
fn diag_device_open_close_loop() {
    let index = test_device_index();
    for (tag, use_uart) in [("①自动开UART", true), ("②不自动开UART", false)] {
        for round in 1..=4 {
            match Context::new() {
                Ok(ctx) => {
                    let count = ctx.find_devices();
                    let mut opened = None;
                    if count > index {
                        if let Ok(dev) = ctx.device(index) {
                            if !use_uart {
                                let _ = dev.set_uart_auto_open(false);
                            }
                            opened = Some(dev.open().is_ok());
                            dev.close();
                        }
                    }
                    println!("{tag} 轮 {round}：ctx=Ok 设备数={count} 打开={opened:?}");
                }
                Err(error) => {
                    // SAFETY: 只读本线程错误码
                    let code = unsafe { dmcan::ffi::dmcan_get_last_error() };
                    println!("{tag} 轮 {round}：ctx=Err({error}) 错误码={code}");
                }
            }
            std::thread::sleep(Duration::from_millis(200));
        }
    }

    // ③ 只枚举，不开设备
    for round in 1..=4 {
        match Context::new() {
            Ok(ctx) => {
                let count = ctx.find_devices();
                println!("③只枚举 轮 {round}：ctx=Ok 设备数={count}");
            }
            Err(error) => println!("③只枚举 轮 {round}：ctx=Err({error})"),
        }
        std::thread::sleep(Duration::from_millis(200));
    }

    // ④ 只枚举，但**销毁后先睡 300 ms 再建**（判断是不是"上一次的清理还没完成"）
    for round in 1..=4 {
        match Context::new() {
            Ok(ctx) => {
                let count = ctx.find_devices();
                println!("④睡300ms再建 轮 {round}：ctx=Ok 设备数={count}");
                drop(ctx);
            }
            Err(error) => println!("④睡300ms再建 轮 {round}：ctx=Err({error})"),
        }
        std::thread::sleep(Duration::from_millis(300));
    }
}

///
/// 只打印不判定，用来把几种可能分开：① 同时存在两个上下文行不行；② 销毁后立刻再建行不行；
/// ③ 销毁后等一会儿再建行不行。附带打印失败时 C 侧的线程局部错误码。
#[test]
#[ignore = "诊断用（真机无关，任何平台都能跑）"]
fn diag_context_lifecycle() {
    for round in 1..=6 {
        let first = Context::new();
        println!(
            "轮 {round}：第一个={:?}",
            first.as_ref().map(|ctx| ctx.sdk_version())
        );

        // ① 同时再来一个
        let second = Context::new();
        println!("轮 {round}：同时第二个={:?}", second.is_ok());

        drop(first);
        drop(second);

        // ② 销毁后立刻再建
        let immediate = Context::new();
        println!("轮 {round}：销毁后立刻={:?}", immediate.is_ok());
        drop(immediate);

        // ③ 睡 300 ms 后再建（同一轮里不销毁）
        std::thread::sleep(Duration::from_millis(300));
        let delayed = Context::new();
        match &delayed {
            Ok(_) => println!("轮 {round}：睡 300ms 后=Ok"),
            Err(error) => {
                // SAFETY: 只是读本线程的错误码
                let code = unsafe { dmcan::ffi::dmcan_get_last_error() };
                println!("轮 {round}：睡 300ms 后=Err({error}) 错误码={code}");
            }
        }
        drop(delayed);
    }
}

/// 880 号诊断用例（临时）：FD 高速数据相位的分界——同一组位定时下对比 BRS 开/关。
/// 890 号用例（人工，两阶段）：`save_param()` 落盘要**断电**才能证明。
///
/// 阶段一：`DMCAN_SAVEPARAM_PHASE=write` —— 把 `busoff_autoreset_ms` 写成 1300 并保存，
/// 然后**人工给该设备断电、重新上电**；
/// 阶段二：`DMCAN_SAVEPARAM_PHASE=check` —— 复读应为 1300，随后还原成 1000 并再次保存。
///
/// 被测设备用 `DMCAN_TEST_DEVICE_INDEX` 选（默认 0）。
#[test]
#[ignore = "需要人工给设备断电：先跑 write，断电重启后再跑 check"]
fn saveparam_power_cycle() {
    const TARGET: u16 = 1300;
    const ORIGINAL: u16 = 1000;

    let ctx = Context::new().expect("创建上下文");
    let index = test_device_index();
    assert!(ctx.find_devices() > index, "没有下标为 {index} 的设备");
    let dev = ctx.device(index).expect("取设备");
    let info = dev.info().expect("读枚举信息");
    dev.open().expect("打开设备");
    // 需要人工断电的用例：**没显式指定阶段就跳过**，避免在整跑套件时改到设备里的落盘值
    // （也避免把空字符串当成 check 阶段）。
    let phase = match std::env::var("DMCAN_SAVEPARAM_PHASE").as_deref() {
        Ok("write") => "write",
        Ok("check") => "check",
        _ => {
            println!(
                "跳过：本用例需要人工给设备断电，请用 DMCAN_SAVEPARAM_PHASE=write / =check 显式跑"
            );
            dev.close();
            return;
        }
    };
    println!(
        "设备[{index}]：{:?} 序列号={:?}；阶段={phase}",
        info.device, info.serial
    );

    let current = dev.channel_ext_config(0).expect("读扩展配置");
    println!("当前扩展配置：{current:?}");

    if phase == "write" {
        let mut target = current;
        target.busoff_autoreset_ms = TARGET;
        dev.set_channel_ext_config(0, &target).expect("写扩展配置");
        wait_baud_reload();
        let readback = dev.channel_ext_config(0).expect("复读扩展配置");
        println!("写入 {TARGET} 后读回：{readback:?}");
        println!("save_param → {:?}", dev.save_param());
        dev.close();
        assert_eq!(readback.busoff_autoreset_ms, TARGET, "写进去的值没读回来");
        println!("*** 现在请给这台设备断电（拔掉再插回），然后用 DMCAN_SAVEPARAM_PHASE=check 再跑一次 ***");
        return;
    }

    // check 阶段：断电重启后应当仍是 TARGET；顺手还原成 ORIGINAL 并再次保存。
    let after_power_cycle = current.busoff_autoreset_ms;
    let mut restore = current;
    restore.busoff_autoreset_ms = ORIGINAL;
    dev.set_channel_ext_config(0, &restore)
        .expect("还原扩展配置");
    wait_baud_reload();
    let restored = dev.channel_ext_config(0).expect("复读（还原后）");
    println!(
        "还原 {ORIGINAL} → {:?}；save_param → {:?}",
        restored,
        dev.save_param()
    );
    dev.close();

    assert_eq!(
        after_power_cycle, TARGET,
        "断电重启后读回的不是 {TARGET}（落盘没生效？上电后有没有被别处写过？）"
    );
    assert_eq!(
        restored.busoff_autoreset_ms, ORIGINAL,
        "没有还原成 {ORIGINAL}"
    );
}

///
/// 目的：把"FD 帧格式本身"与"高速数据相位的信号完整性"分开。BRS 关时数据相位跑仲裁段速率，
/// 若 BRS 关能通、开不通，就说明帧格式没问题、卡在高速数据相位（端接/线长）。
/// 数据段波特率用 `DMCAN_TEST_DATA_BITRATE` 指定（默认 4 Mbps）。
#[test]
#[ignore = "诊断用：需要 D 接 S 的台架"]
fn diag_fd_data_phase_boundary() {
    let ctx = Context::new().expect("创建上下文");
    let (d_dev, s_dev) = find_d_and_s(&ctx);
    setup_pair(&d_dev, &s_dev);

    for (role, dev) in [("D", &d_dev), ("S", &s_dev)] {
        let conf = dev.channel_baudrate_details(0).expect("读回配置");
        println!(
            "{role} 位定时：canfd={} nom(brp={} seg1={} seg2={}) data(brp={} seg1={} seg2={}) tdc={}(off={} filt={})",
            conf.enable_canfd(),
            conf.nominal_brp,
            conf.nominal_seg1,
            conf.nominal_seg2,
            conf.data_brp,
            conf.data_seg1,
            conf.data_seg2,
            conf.enable_tdc(),
            conf.tdc_offset,
            conf.tdc_filter
        );
    }

    for brs in [false, true] {
        let seen = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&seen);
        let sub = s_dev
            .subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
                if let Event::RxFrame(_) = event {
                    counter.fetch_add(1, Ordering::SeqCst);
                }
            })
            .expect("S 订阅");

        let before = d_dev.channel_stats(0).ok().flatten();
        for _ in 0..3 {
            d_dev
                .send_can(
                    0,
                    &Frame::fd(0x4D5, &[0x51, 0x52, 0x53, 0x54], brs).expect("构造帧"),
                )
                .expect("发送");
        }
        std::thread::sleep(Duration::from_millis(1200));
        let after = d_dev.channel_stats(0).ok().flatten();
        println!(
            "BRS={brs}：S 收到 {} 帧；D 提交 {:?}→{:?} 完成 {:?}→{:?} sw={:?} hw={:?} tec={:?}",
            seen.load(Ordering::SeqCst),
            before.as_ref().map(|s| s.tx_admitted),
            after.as_ref().map(|s| s.tx_admitted),
            before.as_ref().map(|s| s.tx_completed),
            after.as_ref().map(|s| s.tx_completed),
            after.as_ref().map(|s| s.sw_tx_fifo_level),
            after.as_ref().map(|s| s.hw_tx_fifo_level),
            d_dev.bus_state(0).map(|state| state.tec)
        );
        drop(sub);
        std::thread::sleep(Duration::from_millis(300));
    }

    d_dev.close();
    s_dev.close();
}

///
/// 监听时长用 `DMCAN_HOTPLUG_SECONDS` 指定（默认 15 秒）。本机 libusb 原生热插拔不可用时，
/// SDK 会退回 500 ms 轮询源，事件照样会来。
#[test]
#[ignore = "诊断用：需要人工拔插设备"]
fn hotplug_event_probe() {
    let ctx = Context::new().expect("创建上下文");
    let seconds: u64 = std::env::var("DMCAN_HOTPLUG_SECONDS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(15);

    let events = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let recorder = Arc::clone(&events);
    let guard = ctx
        .register_hotplug(move |event| {
            let text = format!("{event:?}");
            println!("热插拔事件：{text}");
            recorder.lock().expect("锁").push(text);
        })
        .expect("注册热插拔回调");

    println!(
        "监听 {seconds} 秒（原生热插拔支持={}）：请在此期间拔下再插回任意一台设备",
        ctx.hotplug_supported()
    );
    let deadline = Instant::now() + Duration::from_secs(seconds);
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
    }
    println!("共收到 {} 条事件", events.lock().expect("锁").len());

    // 按 C 侧约定：不要在回调里注销；这里在分发线程之外 drop 掉守卫即可。
    drop(guard);
}
