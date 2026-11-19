/**
 * @file minimal_c_api.c
 * @brief 对外 C API 的最小示例（**C 语言**编译，用来兜住"对外是 C 接口"这条承诺）。
 *
 * 内容与 `README.md` §2 的示例一致：打开设备 → 配置通道 → 注册收帧回调 → 发一帧 →
 * 启动周期发送 → 停止 → 关闭。编译期即验证头文件能被 C（不是 C++）编译器接受。
 *
 * 用法：minimal_c_api.exe [通道号]（缺省 0）；需要接一台设备，会在该通道发两帧（单次 + 周期）。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dmcan.h"

static void on_recv(dmcan_device_handle *handle, dmcan_rx_frame_t *frame)
{
    (void)handle;
    printf("recv: ch=%u id=0x%X dlen=%u ts=%llu us\n", frame->channel, frame->can_id, frame->dlen,
        (unsigned long long)frame->timestamp);
}

int main(int argc, char *argv[])
{
    const unsigned char channel = (argc > 1) ? (unsigned char)atoi(argv[1]) : 0;

    dmcan_context *ctx = NULL;
    dmcan_context_create(&ctx);
    if (ctx == NULL || dmcan_find_devices(ctx) <= 0)
    {
        printf("no device\n");
        return 1;
    }

    dmcan_device_handle *dev = NULL;
    if (!dmcan_device_get(ctx, &dev, 0) || dev == NULL || !dmcan_device_open(dev))
    {
        printf("open device failed\n");
        dmcan_context_destroy(ctx);
        return 1;
    }

    /* 日志：默认 info 到终端；要落盘就注册回调自己写文件（见 tools/log_probe.cpp） */
    dmcan_set_log_level(2);
    dmcan_device_hook_recv_callback(dev, on_recv);
    dmcan_device_enable_channel(dev, channel);

    dmcan_channel_can_info_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.canfd = true;
    cfg.retrans = true;
    cfg.can_baudrate = 1000000;
    cfg.can_sp = 0.75f;
    cfg.canfd_baudrate = 5000000;
    cfg.canfd_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(dev, channel, &cfg))
    {
        printf("set baudrate failed\n");
        dmcan_device_close(dev);
        dmcan_context_destroy(ctx);
        return 1;
    }

    unsigned char payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    /* 单次发送 */
    const bool sent = dmcan_device_send_can(dev, channel, 0x123, true, false, false, true, 8, payload);
    printf("send_can: %s\n", sent ? "accepted" : "rejected");

    /* 周期发送：10 ms 一次、send_times=0 表示无限 */
    const bool periodic =
        dmcan_device_send_can_details(dev, channel, 10, 0, 0x124, true, false, false, true, 8, payload);
    printf("periodic send: %s\n", periodic ? "accepted" : "rejected");
    dmcan_device_stop_send(dev, channel);

    dmcan_device_close(dev);
    dmcan_context_destroy(ctx);
    printf("minimal example done\n");
    return 0;
}
