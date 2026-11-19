//! dmcan（USB2CANFD Rust 封装层）最小示例：枚举 → 打开 → 读固件信息 → 配参数 → 订阅收帧 → 发一帧 → 读统计。
//!
//! 跑法：`cargo run`（没有接模块时会提示并正常退出）。
//!
//! 运行前请确认 libusb 已就位：Windows 把 `libusb-1.0.dll` 放到本目录或 `PATH` 里；
//! Linux 装 `libusb-1.0-0`；macOS 把 `libusb-1.0.0.dylib` 放到与 `libdmcan.dylib` 同目录。

use std::time::Duration;

use dmcan::{ChannelBaudInfo, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

fn main() -> dmcan::Result<()> {
    // 1) 创建上下文（内部会校验「库」与「封装」的版本是否配套）
    let ctx = Context::new()?;
    println!("SDK 版本：{}", ctx.sdk_version());

    // 2) 枚举设备（返回句柄表里的设备数；拔掉的设备会留在表里）
    let count = ctx.find_devices();
    println!("发现 {count} 台设备");
    if count == 0 {
        println!("没有找到设备，示例结束");
        return Ok(());
    }

    // 3) 取第 0 台、读设备信息与固件版本，然后打开
    let dev = ctx.device(0)?;
    let info = dev.info()?;
    println!(
        "设备：{:?}  VID=0x{:04X} PID=0x{:04X}  序列号={:?}",
        info.device, info.vid, info.pid, info.serial
    );
    dev.open()?;
    println!("固件：{}", dev.firmware_info()?.to_string_line()?);

    // 4) 订阅收帧事件。回调在 SDK 自己的线程上同步执行，所以这里只做一次拷贝再转给主线程；
    //    要留存到回调之外的数据必须自己拷贝（to_owned()）。
    let (tx, rx) = std::sync::mpsc::channel();
    let _sub = dev.subscribe(Events::RX_FRAME, CHANNEL_MASK_ALL, move |event| {
        if let Event::RxFrame(frame) = event {
            let _ = tx.send(frame.to_owned());
        }
    })?;

    // 5) 配置通道：CAN FD，仲裁段与数据段都是 1 Mbps，采样点 0.75。
    //    注意顺序：**先配参数、再使能通道**——参数下发后通道会回到未使能状态，
    //    先使能再配会让之后发出的帧被丢弃。
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
    std::thread::sleep(Duration::from_millis(1200)); // 等固件把位定时重载完
    dev.enable_channel(0)?;

    // 6) 发一帧（ID 0x123，8 字节，CAN FD + BRS）
    dev.send_can(0, &Frame::fd(0x123, &[1, 2, 3, 4, 5, 6, 7, 8], true)?)?;
    println!("已发送一帧：ID=0x123，8 字节");

    // 7) 等 1 秒看有没有回帧（总线上有对端时才会收到；也可用外部回环模式做单模块自测）
    match rx.recv_timeout(Duration::from_secs(1)) {
        Ok(frame) => println!(
            "收到帧：ID=0x{:X} 通道={} 长度={} 数据={:?}",
            frame.can_id,
            frame.channel,
            frame.dlen,
            frame.data()
        ),
        Err(_) => println!("1 秒内没有收到帧（总线上没有其它节点时属正常）"),
    }

    // 8) 读通道统计。数据来自固件每秒一次的周期上报，所以刚打开时可能还没有值（None）。
    match dev.channel_stats(0)? {
        Some(stats) => println!(
            "通道 0：发送受理={} 发送完成={} 接收={} 丢弃={}",
            stats.tx_admitted, stats.tx_completed, stats.rx_received, stats.tx_dropped
        ),
        None => println!("通道 0：还没有收到周期上报（打开后约 1 秒再读）"),
    }

    dev.close();
    Ok(())
}
