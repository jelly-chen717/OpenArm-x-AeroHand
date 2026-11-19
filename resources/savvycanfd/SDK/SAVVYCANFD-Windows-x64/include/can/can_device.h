/*
 * USB-CAN SDK - Device Control API
 * Copyright (c) 2026 SavvyCAN. All rights reserved.
 *
 * API for discovering, opening, configuring, and communicating
 * with CAN/CAN FD interfaces (PCAN and WinUSB/LibUSB devices).
 */
#ifndef USB_CAN_DEVICE_H
#define USB_CAN_DEVICE_H

#include "can_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== SDK Lifecycle ===== */

/* Initialize SDK (call once at startup) */
USB_CAN_API can_status_t can_sdk_init(void);

/* Shutdown SDK (call once at exit) */
USB_CAN_API can_status_t can_sdk_shutdown(void);

/* ===== Device Discovery ===== */

/* Scan for available CAN devices.
 * devices: array to fill with device info
 * max_count: max number of entries in array
 * Returns: number of devices found, or negative error code */
USB_CAN_API int can_discover(can_device_info_t *devices, int max_count);

/* ===== Device Open/Close ===== */

/* Open a CAN device by index (from can_discover).
 * handle: output handle pointer
 * device_index: index from can_device_info_t.index
 * Returns: CAN_OK or error code */
USB_CAN_API can_status_t can_open(can_handle_t *handle, int device_index);

/* Close a CAN device and free resources */
USB_CAN_API can_status_t can_close(can_handle_t handle);

/* ===== Configuration ===== */

/* Configure CAN interface (bitrate, CAN FD, etc.)
 * Must be called before can_start().
 * config: pointer to configuration struct */
USB_CAN_API can_status_t can_configure(can_handle_t handle, const can_config_t *config);

/* Get current configuration */
USB_CAN_API can_status_t can_get_config(can_handle_t handle, can_config_t *config);

/* Get device capabilities (CAN_CAP_xxx bitmask) */
USB_CAN_API uint32_t can_get_capabilities(can_handle_t handle);

/* ===== Bus Control ===== */

/* Start CAN communication (go bus-on) */
USB_CAN_API can_status_t can_start(can_handle_t handle);

/* Stop CAN communication (go bus-off) */
USB_CAN_API can_status_t can_stop(can_handle_t handle);

/* Reset CAN bus (clear errors, flush queues) */
USB_CAN_API can_status_t can_reset(can_handle_t handle);

/* Get current bus state (CAN_STATE_xxx) */
USB_CAN_API uint32_t can_get_state(can_handle_t handle);

/* ===== Message Send/Receive ===== */

/* Send a CAN message.
 * msg: pointer to message struct
 * Returns: CAN_OK or error code */
USB_CAN_API can_status_t can_send(can_handle_t handle, const can_msg_t *msg);

/* Receive CAN messages (non-blocking).
 * msgs: array to fill with received messages
 * max_count: max number of messages to read
 * timeout_ms: timeout in milliseconds (0 = non-blocking)
 * Returns: number of messages received, or negative error code */
USB_CAN_API int can_receive(can_handle_t handle, can_msg_t *msgs, int max_count, uint32_t timeout_ms);

/* ===== Statistics ===== */

/* Get device statistics */
USB_CAN_API can_status_t can_get_stats(can_handle_t handle, can_stats_t *stats);

/* Reset statistics counters */
USB_CAN_API can_status_t can_reset_stats(can_handle_t handle);

/* ===== Device Info ===== */

/* Get device name string */
USB_CAN_API const char* can_get_name(can_handle_t handle);

/* Get driver version string */
USB_CAN_API const char* can_get_version(can_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* USB_CAN_DEVICE_H */
