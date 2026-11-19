"""
@file demo.py
@brief DM-DeviceSDK Python 版最小示例：枚举设备 → 打开 → 使能通道 → 订阅收帧 → 发一帧 → 退出。

运行前提：
    1. 装好 Python 包：`pip install ../pack/dmcan_sdk-2.1.0-py3-none-any.whl`（或 `pip install dmcan_sdk`）；
    2. 后端动态库放在本目录的 `dlls/` 下（`libdmcan.dll` 与其依赖 DLL 必须同目录），
       脚本会把该目录显式告诉 SDK（等价于设置环境变量 `DMCAN_LIBRARY_PATH`）。

注意：
    - `open()` 会**独占**设备的 UART COM 口（默认自动打开 UART）；
    - 所有接口失败都会抛 `DmCanError`（错误码在 `exc.status`），不再返回 `False`。
"""

import os
import pathlib
import time

# 显式指定后端库位置：本目录 dlls/libdmcan.dll（也可以用环境变量覆盖）
_DLL_PATH = pathlib.Path(__file__).resolve().parent / "dlls" / "libdmcan.dll"
os.environ.setdefault("DMCAN_LIBRARY_PATH", str(_DLL_PATH))

from dmcan import DmCanContext, DmCanError, dmcan_event_flag  # noqa: E402


def main() -> int:
    """
    @brief 跑一遍最小示例。

    @return 进程退出码（0 = 成功）
    """
    with DmCanContext() as context:
        print("后端库版本：", context.sdk_version())
        count = context.find_devices()
        print("找到设备：", count, "台")
        context.show_all_devices()
        if count == 0:
            print("没有找到设备，请检查 USB 连接与驱动")
            return 2

        device = context.get_device(0)
        print("设备信息：", device.get_info())
        device.open()
        try:
            print("固件版本：", device.get_version())

            # 通道 0 按设备里已保存的位定时使能（改位定时要先关通道，见 README）
            device.enable_channel(0, True)
            baud = device.get_channel_baudrate(0)
            print("CH0：canfd=%s 标称=%d 数据段=%d" % (bool(baud.canfd), baud.can_baudrate, baud.canfd_baudrate))

            # 收帧推荐走订阅；回调在 SDK 的通知线程上同步执行，必须快速返回
            subscription = device.subscribe(dmcan_event_flag.RX_FRAME,
                                            lambda dev, event: print("  RX", event.frame),
                                            channel_mask=1 << 0)

            # 发一帧：数据长度直接由 payload 决定（2.x 不再单独传 dlen）
            device.send_can(0, 0x123, bytes([0x11, 0x22, 0x33, 0x44]),
                            canfd=bool(baud.canfd), brs=bool(baud.canfd))
            time.sleep(1.0)

            subscription.cancel()
            device.disable_channel(0)
        finally:
            device.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DmCanError as exc:
        print("SDK 调用失败：%s（错误码 %s）" % (exc, exc.status))
        raise SystemExit(1)
