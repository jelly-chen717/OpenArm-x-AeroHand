/*
 * USB-CAN SDK - Type Definitions
 * Copyright (c) 2026 SavvyCAN. All rights reserved.
 */
#ifndef USB_CAN_TYPES_H
#define USB_CAN_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Error Codes ===== */
typedef int can_status_t;

#define CAN_OK                   0
#define CAN_ERR_INVALID_PARAM   -1
#define CAN_ERR_NO_DEVICE       -2
#define CAN_ERR_OPEN_FAILED     -3
#define CAN_ERR_NOT_OPEN        -4
#define CAN_ERR_SEND_FAILED     -5
#define CAN_ERR_TIMEOUT         -6
#define CAN_ERR_BUS_OFF         -7
#define CAN_ERR_BUFFER_FULL     -8
#define CAN_ERR_NOT_SUPPORTED   -9
#define CAN_ERR_INTERNAL       -10

/* ===== Bus State ===== */
#define CAN_STATE_OK             0
#define CAN_STATE_WARNING        1
#define CAN_STATE_PASSIVE        2
#define CAN_STATE_BUS_OFF        3
#define CAN_STATE_STOPPED        4
#define CAN_STATE_UNKNOWN        5

/* ===== Device Capabilities (bitmask) ===== */
#define CAN_CAP_CANFD            0x01
#define CAN_CAP_LISTEN_ONLY      0x02
#define CAN_CAP_ONE_SHOT         0x08
#define CAN_CAP_AUTO_RESTART     0x10

/* ===== CAN Message Flags (bitmask) ===== */
#define CAN_FLAG_EXTENDED        0x01   /* 29-bit extended ID */
#define CAN_FLAG_RTR             0x02   /* Remote transmission request */
#define CAN_FLAG_FD              0x04   /* CAN FD frame */
#define CAN_FLAG_BRS             0x08   /* Bit rate switch (FD only) */
#define CAN_FLAG_ERROR           0x10   /* Error frame */

/* ===== CAN Message Structure ===== */
typedef struct {
    uint32_t id;                /* CAN ID (11-bit or 29-bit) */
    uint8_t  dlc;               /* Data length code (0-8 for CAN, 0-64 for CAN FD) */
    uint8_t  flags;             /* CAN_FLAG_xxx bitmask */
    uint8_t  reserved[2];       /* Alignment padding */
    uint8_t  data[64];          /* Payload data */
    uint64_t timestamp_us;      /* Timestamp in microseconds */
} can_msg_t;

/* ===== Device Info Structure ===== */
#define CAN_MAX_NAME_LEN  64
#define CAN_MAX_DEVICES   16

typedef struct {
    int      index;                        /* Device index (for open) */
    char     name[CAN_MAX_NAME_LEN];       /* Device name (e.g. "PCAN-USB1") */
    char     description[CAN_MAX_NAME_LEN];/* Device description */
    char     driver[CAN_MAX_NAME_LEN];     /* Driver name (e.g. "PCAN", "LibUsbCAN") */
    uint32_t capabilities;                 /* CAN_CAP_xxx bitmask */
} can_device_info_t;

/* ===== Device Configuration ===== */
typedef struct {
    uint32_t bitrate;           /* Arbitration bitrate in bps (e.g. 500000) */
    uint32_t sample_point;      /* Sample point in 0.1% (e.g. 875 = 87.5%) */
    uint8_t  canfd_enabled;     /* 0 = Classic CAN, 1 = CAN FD */
    uint32_t data_bitrate;      /* CAN FD data phase bitrate (e.g. 2000000) */
    uint32_t data_sample_point; /* CAN FD data phase sample point */
} can_config_t;

/* ===== Device Statistics ===== */
typedef struct {
    uint64_t rx_frames;
    uint64_t tx_frames;
    uint32_t rx_errors;
    uint32_t tx_errors;
    uint32_t rx_overruns;
    uint32_t tx_dropped;
    uint32_t bus_state;         /* CAN_STATE_xxx */
} can_stats_t;

/* Opaque device handle */
typedef void* can_handle_t;

/* Convert status code to string */
USB_CAN_API const char* can_status_str(can_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* USB_CAN_TYPES_H */
