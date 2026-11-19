/*
 * USB-CAN SDK - Basic Example
 * 基础示例：扫描设备 -> 打开 -> 配置 -> 发送 -> 接收 -> 关闭
 *
 * 编译方法 (MSVC):
 *   cl basic_can.c /I ..\include /link usb_can.lib
 *
 * 编译方法 (CMake):
 *   cmake --build build --target can_example
 *
 * 运行前确保:
 *   1. usb_can.dll 和 PCANBasic.dll 在同一目录
 *   2. PCAN 设备已连接
 */

#include <stdio.h>
#include <string.h>
#include <can/usb_can_sdk.h>

int main(void)
{
    can_status_t status;
    can_handle_t handle = NULL;

    /* Step 1: 初始化 SDK */
    printf("=== USB-CAN SDK Basic Example ===\n\n");
    status = can_sdk_init();
    if (status != CAN_OK) {
        printf("SDK init failed: %s\n", can_status_str(status));
        return 1;
    }
    printf("[OK] SDK initialized\n");

    /* Step 2: 扫描设备 */
    can_device_info_t devices[CAN_MAX_DEVICES];
    int count = can_discover(devices, CAN_MAX_DEVICES);
    if (count <= 0) {
        printf("No CAN devices found\n");
        can_sdk_shutdown();
        return 1;
    }
    printf("[OK] Found %d device(s):\n", count);
    for (int i = 0; i < count; i++) {
        printf("  [%d] %s - %s (Driver: %s, Caps: 0x%02X)\n",
            devices[i].index,
            devices[i].name,
            devices[i].description,
            devices[i].driver,
            devices[i].capabilities);
    }

    /* Step 3: 打开第一个设备 */
    status = can_open(&handle, 0);
    if (status != CAN_OK) {
        printf("Open failed: %s\n", can_status_str(status));
        can_sdk_shutdown();
        return 1;
    }
    printf("\n[OK] Opened: %s\n", can_get_name(handle));

    /* Step 4: 配置波特率 500kbps */
    can_config_t config;
    memset(&config, 0, sizeof(config));
    config.bitrate = 500000;
    config.sample_point = 875;
    config.canfd_enabled = 0;

    status = can_configure(handle, &config);
    if (status != CAN_OK) {
        printf("Configure failed: %s\n", can_status_str(status));
        can_close(handle);
        can_sdk_shutdown();
        return 1;
    }
    printf("[OK] Configured: %u bps\n", config.bitrate);

    /* Step 5: 启动通信 */
    status = can_start(handle);
    if (status != CAN_OK) {
        printf("Start failed: %s\n", can_status_str(status));
        can_close(handle);
        can_sdk_shutdown();
        return 1;
    }
    printf("[OK] CAN bus started\n");

    /* Step 6: 发送测试帧 */
    can_msg_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.id = 0x123;
    tx_msg.dlc = 8;
    tx_msg.flags = 0;  /* Standard frame */
    tx_msg.data[0] = 0x01;
    tx_msg.data[1] = 0x02;
    tx_msg.data[2] = 0x03;
    tx_msg.data[3] = 0x04;
    tx_msg.data[4] = 0x05;
    tx_msg.data[5] = 0x06;
    tx_msg.data[6] = 0x07;
    tx_msg.data[7] = 0x08;

    status = can_send(handle, &tx_msg);
    if (status == CAN_OK) {
        printf("[OK] Sent: ID=0x%03X DLC=%d Data=", tx_msg.id, tx_msg.dlc);
        for (int i = 0; i < tx_msg.dlc; i++) printf("%02X ", tx_msg.data[i]);
        printf("\n");
    } else {
        printf("[ERR] Send failed: %s\n", can_status_str(status));
    }

    /* Step 7: 接收帧 (等待 2 秒) */
    printf("\n[...] Waiting for messages (2 seconds)...\n");
    can_msg_t rx_msgs[16];
    int rx_count = can_receive(handle, rx_msgs, 16, 2000);

    if (rx_count > 0) {
        printf("[OK] Received %d message(s):\n", rx_count);
        for (int i = 0; i < rx_count; i++) {
            printf("  ID=0x%03X DLC=%d Flags=0x%02X Data=",
                rx_msgs[i].id, rx_msgs[i].dlc, rx_msgs[i].flags);
            for (int j = 0; j < rx_msgs[i].dlc; j++)
                printf("%02X ", rx_msgs[i].data[j]);
            printf("\n");
        }
    } else {
        printf("[--] No messages received\n");
    }

    /* Step 8: 查看统计 */
    can_stats_t stats;
    can_get_stats(handle, &stats);
    printf("\n[STATS] Rx: %llu  Tx: %llu  RxErr: %u  TxErr: %u  State: %u\n",
        (unsigned long long)stats.rx_frames,
        (unsigned long long)stats.tx_frames,
        stats.rx_errors, stats.tx_errors, stats.bus_state);

    /* Step 9: 清理 */
    can_stop(handle);
    can_close(handle);
    can_sdk_shutdown();
    printf("\n[OK] Done.\n");

    return 0;
}
