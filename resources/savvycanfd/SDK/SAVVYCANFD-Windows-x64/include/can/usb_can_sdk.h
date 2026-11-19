/*
 * USB-CAN SDK - Main Include Header
 * Copyright (c) 2026 SavvyCAN. All rights reserved.
 *
 * Single-include header for USB-CAN SDK.
 * Supports PCAN and WinUSB/LibUSB CAN interfaces.
 */
#ifndef USB_CAN_SDK_H
#define USB_CAN_SDK_H

#ifdef _WIN32
  #ifdef USB_CAN_EXPORTS
    #define USB_CAN_API __declspec(dllexport)
  #else
    #define USB_CAN_API __declspec(dllimport)
  #endif
#else
  #ifdef USB_CAN_EXPORTS
    #define USB_CAN_API __attribute__((visibility("default")))
  #else
    #define USB_CAN_API
  #endif
#endif

#include "can_types.h"
#include "can_device.h"

#endif /* USB_CAN_SDK_H */
