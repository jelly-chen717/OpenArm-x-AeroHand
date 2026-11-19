"""Raw ctypes bindings — direct 1:1 mapping of the C API in
``include/can/can_types.h`` and ``can_device.h``.

The high-level ``usb_can`` package (``__init__.py``) wraps these into a
Pythonic interface. Most users do not need to import this module directly.
"""
from __future__ import annotations

import ctypes
from ctypes import (POINTER, Structure, c_char, c_int, c_uint8, c_uint32,
                    c_uint64, c_char_p, c_void_p)

from ._loader import load_library


_lib = load_library()


# ---------------------------------------------------------------------------
# Constants (mirror can_types.h)
# ---------------------------------------------------------------------------

CAN_OK                = 0
CAN_ERR_INVALID_PARAM = -1
CAN_ERR_NO_DEVICE     = -2
CAN_ERR_OPEN_FAILED   = -3
CAN_ERR_NOT_OPEN      = -4
CAN_ERR_SEND_FAILED   = -5
CAN_ERR_TIMEOUT       = -6
CAN_ERR_BUS_OFF       = -7
CAN_ERR_BUFFER_FULL   = -8
CAN_ERR_NOT_SUPPORTED = -9
CAN_ERR_INTERNAL      = -10

CAN_STATE_OK       = 0
CAN_STATE_WARNING  = 1
CAN_STATE_PASSIVE  = 2
CAN_STATE_BUS_OFF  = 3
CAN_STATE_STOPPED  = 4
CAN_STATE_UNKNOWN  = 5

CAN_CAP_CANFD        = 0x01
CAN_CAP_LISTEN_ONLY  = 0x02
CAN_CAP_ONE_SHOT     = 0x08
CAN_CAP_AUTO_RESTART = 0x10

CAN_FLAG_EXTENDED = 0x01
CAN_FLAG_RTR      = 0x02
CAN_FLAG_FD       = 0x04
CAN_FLAG_BRS      = 0x08
CAN_FLAG_ERROR    = 0x10

CAN_MAX_NAME_LEN = 64
CAN_MAX_DEVICES  = 16


# ---------------------------------------------------------------------------
# Structures (layout must match can_types.h exactly)
# ---------------------------------------------------------------------------

class CanMsg(Structure):
    _fields_ = [
        ("id",           c_uint32),
        ("dlc",          c_uint8),
        ("flags",        c_uint8),
        ("reserved",     c_uint8 * 2),
        ("data",         c_uint8 * 64),
        ("timestamp_us", c_uint64),
    ]


class CanDeviceInfo(Structure):
    _fields_ = [
        ("index",        c_int),
        ("name",         c_char * CAN_MAX_NAME_LEN),
        ("description",  c_char * CAN_MAX_NAME_LEN),
        ("driver",       c_char * CAN_MAX_NAME_LEN),
        ("capabilities", c_uint32),
    ]


class CanConfig(Structure):
    _fields_ = [
        ("bitrate",           c_uint32),
        ("sample_point",      c_uint32),
        ("canfd_enabled",     c_uint8),
        ("data_bitrate",      c_uint32),
        ("data_sample_point", c_uint32),
    ]


class CanStats(Structure):
    _fields_ = [
        ("rx_frames",   c_uint64),
        ("tx_frames",   c_uint64),
        ("rx_errors",   c_uint32),
        ("tx_errors",   c_uint32),
        ("rx_overruns", c_uint32),
        ("tx_dropped",  c_uint32),
        ("bus_state",   c_uint32),
    ]


# ---------------------------------------------------------------------------
# Function bindings (signatures match can_device.h)
# ---------------------------------------------------------------------------

def _bind(name, argtypes, restype):
    fn = getattr(_lib, name)
    fn.argtypes = argtypes
    fn.restype  = restype
    return fn


can_sdk_init         = _bind("can_sdk_init",         [],                                             c_int)
can_sdk_shutdown     = _bind("can_sdk_shutdown",     [],                                             c_int)
can_discover         = _bind("can_discover",         [POINTER(CanDeviceInfo), c_int],                c_int)
can_open             = _bind("can_open",             [POINTER(c_void_p), c_int],                    c_int)
can_close            = _bind("can_close",            [c_void_p],                                    c_int)
can_configure        = _bind("can_configure",        [c_void_p, POINTER(CanConfig)],                c_int)
can_get_config       = _bind("can_get_config",       [c_void_p, POINTER(CanConfig)],                c_int)
can_get_capabilities = _bind("can_get_capabilities", [c_void_p],                                    c_uint32)
can_start            = _bind("can_start",            [c_void_p],                                    c_int)
can_stop             = _bind("can_stop",             [c_void_p],                                    c_int)
can_reset            = _bind("can_reset",            [c_void_p],                                    c_int)
can_get_state        = _bind("can_get_state",        [c_void_p],                                    c_uint32)
can_send             = _bind("can_send",             [c_void_p, POINTER(CanMsg)],                   c_int)
can_receive          = _bind("can_receive",          [c_void_p, POINTER(CanMsg), c_int, c_uint32],  c_int)
can_get_stats        = _bind("can_get_stats",        [c_void_p, POINTER(CanStats)],                 c_int)
can_reset_stats      = _bind("can_reset_stats",      [c_void_p],                                    c_int)
can_get_name         = _bind("can_get_name",         [c_void_p],                                    c_char_p)
can_get_version      = _bind("can_get_version",      [c_void_p],                                    c_char_p)
_status_str          = _bind("can_status_str",       [c_int],                                       c_char_p)


def status_str(code: int) -> str:
    s = _status_str(int(code))
    return s.decode("utf-8", "replace") if s else f"<{code}>"
