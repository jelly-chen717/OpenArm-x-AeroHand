using DM_DeviceSDK_CSharp.core;

// DM USB2CANFD C# SDK 最小示例（对应 C SDK v2.1.0）。
//
//   cd demo
//   dotnet run
//
// 包由 ../pack 作为本地 NuGet 源还原（见 DM_DeviceSDK_Demo.csproj），断网也能跑。
// 没插模块也能跑：会打印库版本、枚举结果后正常结束。

using var context = new DmCanContext();

Console.WriteLine($"dmcan 库版本 {dmcan_def.FormatVersion(context.GetSdkVersion())}，" +
                  $"CheckVersion() = {DmCanContext.CheckVersion()}");
Console.WriteLine($"当前日志级别 {DmCanLog.GetLevel()}（0=trace … 6=off）");

// 枚举：不传型号是全部型号；也可以 context.FindAllDevice(dmcan_device_type.USB2CANFD_D)
int deviceCount = context.FindAllDevice();
context.ShowAllDevices();
Console.WriteLine($"枚举到 {deviceCount} 台设备");

var device = context.GetDevice(0);

if (device is null)
{
    Console.WriteLine("没有可用设备，示例到此结束（插上模块再跑一次即可继续）。");
    return;
}

Console.WriteLine($"使用设备：{device}");

// 帧事件。回调同步跑在 SDK 的通知线程上，必须快速返回。
device.ReceiveEchoEvent += args => Console.WriteLine($"RX  {args.Frame}");
device.SendEchoEvent += args => Console.WriteLine($"TX  ack={args.Frame.Ack} {args.Frame}");
device.ChannelErrorEvent += args => Console.WriteLine($"ERR {args}");

if (!device.Open())
{
    Console.WriteLine($"打开设备失败：{DmCanUtils.GetLastError()}");
    return;
}

// 1) 配通道：CAN FD，仲裁域 1 Mbps、数据域 5 Mbps，采样点 0.75
var baud = new dmcan_channel_can_info
{
    canfd = true,
    retrans = true,
    can_baudrate = 1000000,
    canfd_baudrate = 5000000,
    can_sp = 0.75f,
    canfd_sp = 0.75f
};

if (!device.SetChannelBaudByInfo(0, baud))
    Console.WriteLine($"配置波特率失败：{DmCanUtils.GetLastError()}");

// 2) 使能通道（设备复位/重新插拔后通道默认是关闭的，必须重新使能）
if (!device.EnableChannel(0, true))
    Console.WriteLine($"使能通道失败：{DmCanUtils.GetLastError()}");

// 3) 打开真 echo：固件会把真正发上总线的帧回传，这些帧的 ack = true
device.SetChannelEcho(0, true);

byte[] payload = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };

// 4) 发一帧
if (!device.Send(0, 0x123, canfd: true, ext: false, rtr: false, brs: true, dlen: 8, payload: payload))
    Console.WriteLine($"发送失败：{DmCanUtils.GetLastError()}");

// 5) 一次交一批帧（校验全部前置：有一帧非法则这批一帧都不发）
var batch = new[]
{
    dmcan_frame.Create(0x200, payload, canfd: true, ext: false, rtr: false, brs: true),
    dmcan_frame.Create(0x201, new byte[] { 0x01, 0x02, 0x03 }, canfd: true, ext: false, rtr: false, brs: true)
};

Console.WriteLine(device.SendBatch(0, batch)
    ? "批量发送已提交"
    : $"批量发送失败：{DmCanUtils.GetLastError()}");

// 6) 周期发送 1 秒后停止（send_times = 0 表示不限次数）
device.SendPeriodic(0, interval_ms: 10, send_times: 0, canid: 0x300, canfd: true, ext: false, rtr: false,
    brs: true, dlen: 8, payload: payload);

Thread.Sleep(1000);
device.StopSend(0);

// 7) 读总线状态与通道统计（统计来自固件每秒一条的周期上报，最多旧 1 秒）
if (device.GetBusState(0, out var bus))
    Console.WriteLine($"总线状态：state={bus.bus_state} tec={bus.tec} rec={bus.rec} credits={bus.credits}");

if (device.GetChannelStats(0, out var stats))
    Console.WriteLine($"通道统计：tx_admitted={stats.tx_admitted} tx_completed={stats.tx_completed} " +
                      $"rx_received={stats.rx_received} tx_dropped={stats.tx_dropped}");

// 8) 固件信息：一次拿到发布版本号与构建标识（判"升级是否生效"只看构建标识）
if (device.GetFirmwareInfo(out var firmware))
    Console.WriteLine($"固件信息：{DmCanDevice.FormatFirmwareInfo(firmware)}" +
                      $"（构建标识 {firmware.BuildIdString()}，有效={firmware.build_id_valid}）");

Thread.Sleep(500);
device.Close();
Console.WriteLine("示例结束");
