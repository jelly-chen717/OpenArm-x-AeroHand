//! 订阅示例：把四类事件都订上，发几帧看事件怎么来。
//!
//! 跑法：`cargo run --example subscribe`（需要接模块；没有设备时会正常退出）。

use std::time::Duration;

use dmcan::{ChannelBaudInfo, Context, Event, Events, Frame, CHANNEL_MASK_ALL};

fn main() -> dmcan::Result<()> {
    let ctx = Context::new()?;
    if ctx.find_devices() == 0 {
        println!("没有找到设备，示例到此结束");
        return Ok(());
    }

    let dev = ctx.device(0)?;
    dev.open()?;

    // 一个订阅可以同时关心多类事件；回调在 SDK 的通知线程上跑，必须快速返回。
    let _sub = dev.subscribe(
        Events::RX_FRAME | Events::TX_COMPLETED | Events::TX_ON_BUS | Events::CHANNEL_ERROR,
        CHANNEL_MASK_ALL,
        move |event| match event {
            Event::RxFrame(frame) => println!(
                "收帧 0x{:X} ch{} dlen={} ts={}µs",
                frame.can_id(),
                frame.channel(),
                frame.dlen(),
                frame.timestamp_us()
            ),
            Event::TxCompleted(frame) => println!(
                "假 echo 0x{:X} ch{}（这笔 USB 提交完成）",
                frame.can_id(),
                frame.channel()
            ),
            Event::TxOnBus(frame) => println!(
                "真 echo 0x{:X} ch{} ack={} ts={}µs",
                frame.can_id(),
                frame.channel(),
                frame.ack(),
                frame.timestamp_us()
            ),
            Event::ChannelError(error) => {
                println!(
                    "通道错误 bus_state={} tec={} rec={}",
                    error.bus_state, error.tec, error.rec
                )
            }
            Event::UartRx(data) => println!("UART 收到 {} 字节", data.len()),
            _ => {}
        },
    )?;

    // 顺序：先配波特率、再使能（参数下发会让固件把通道置回未使能）。
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
    std::thread::sleep(Duration::from_millis(1200));

    // 打开真 echo：固件会把真正上总线的帧回传（总线没有对端时收不到确认，
    // 帧会一直挂在 echo 待确认队列里，超时后计入 echo_stats）。这里只为演示事件形状。
    match dev.set_channel_echo(0, true) {
        Ok(()) => println!("已打开通道 0 的真 echo"),
        Err(error) => println!("真 echo 打不开：{error}"),
    }

    for id in [0x101u32, 0x102, 0x103] {
        dev.send_can(0, &Frame::fd(id, &[1, 2, 3, 4], true)?)?;
    }
    println!("已发 3 帧，等 1 秒看回调……");
    std::thread::sleep(Duration::from_secs(1));

    if let Ok(stats) = dev.echo_stats() {
        println!(
            "echo 统计：pending={} timeout={} late={} 阈值={}ms",
            stats.pending, stats.timeout_count, stats.late_discarded, stats.timeout_ms
        );
    }

    dev.set_channel_echo(0, false)?;
    // 先注销订阅、再关设备（反过来会让 C 侧 AV）。
    drop(_sub);
    dev.close();
    Ok(())
}
