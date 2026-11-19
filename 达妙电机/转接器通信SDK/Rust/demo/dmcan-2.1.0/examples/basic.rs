//! 最小示例：枚举设备 → 打开 → 读版本 → 订阅收帧 → 配 1M/4M → 发一帧 → 关设备。
//!
//! 直接跑：`cargo run --example basic`（没有接模块时会在枚举后正常退出，不算失败）。

use std::time::Duration;

use dmcan::{ChannelBaudInfo, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

fn main() -> dmcan::Result<()> {
    let ctx = Context::new()?;
    println!("SDK 版本：{}", ctx.sdk_version());

    let count = ctx.find_devices();
    println!("句柄表里的设备数：{count}（含已拔掉的）");
    if count == 0 {
        println!("没有找到设备，示例到此结束");
        return Ok(());
    }

    let dev = ctx.device(0)?;
    let info = dev.info()?;
    println!(
        "设备：{:?}  VID=0x{:04X} PID=0x{:04X} 序列号={:?}",
        info.device, info.vid, info.pid, info.serial
    );

    dev.open()?;
    let firmware = dev.firmware_info()?;
    println!("固件：{}", firmware.to_string_line()?);
    if !firmware.version_valid {
        println!("注意：没读到 app 版本号，板子可能还停在 boot");
    }

    // 收帧：回调跑在 SDK 的通知线程上，这里只做一次拷贝丢进自己的 channel。
    let (tx, rx) = std::sync::mpsc::channel();
    let _sub = dev.subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
        if let Event::RxFrame(frame) = event {
            let _ = tx.send(frame.to_owned());
        }
    })?;

    // 顺序：**先配波特率、再使能通道**。参数下发会让固件重载 MCAN 并把通道置回未使能，
    // 先使能再配等于白配，之后发的帧会被计成 `tx_dropped_ch_disabled` 丢掉。
    dev.set_channel_baudrate(
        0,
        &ChannelBaudInfo {
            canfd: true,
            retrans: true,
            can_baudrate: 1_000_000,
            can_sp: 0.75,
            canfd_baudrate: 1_000_000,
            canfd_sp: 0.75,
            ..Default::default()
        },
    )?;
    std::thread::sleep(Duration::from_millis(1200));
    dev.enable_channel(0)?;

    // 下发与生效不同步：固件每秒任务里重载硬件（实测 0.086 ~ 1.0 s），等一会儿再发更稳。
    std::thread::sleep(Duration::from_millis(1200));

    let frame = Frame::fd(0x123, &[1, 2, 3, 4, 5, 6, 7, 8], true)?;
    dev.send_can(0, &frame)?;
    println!("已发一帧 0x{:X}（CAN FD/BRS）", frame.can_id);

    match rx.recv_timeout(Duration::from_secs(1)) {
        Ok(received) => println!(
            "收到帧 0x{:X} ch{} dlen={} data={:?}",
            received.can_id,
            received.channel,
            received.dlen,
            received.data()
        ),
        Err(_) => println!("1 秒内没收到帧（总线上没有别的节点发帧时属正常）"),
    }

    if let Ok(Some(stats)) = dev.channel_stats(0) {
        println!(
            "通道 0 统计：tx_admitted={} tx_completed={} rx_received={} rx_dropped={}",
            stats.tx_admitted, stats.tx_completed, stats.rx_received, stats.rx_dropped
        );
    }

    // 顺序：**先注销订阅（drop guard），再关设备**。反过来（close 之后注销）实测会让 C 侧 AV。
    drop(_sub);
    // Device 不拥有句柄、Drop 不会关设备，要关得显式来。
    dev.close();
    Ok(())
}
