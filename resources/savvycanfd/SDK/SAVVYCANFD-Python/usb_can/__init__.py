"""USB-CAN SDK — Python interface.

Quick start::

    import usb_can

    usb_can.can_sdk_init()
    devs = usb_can.discover()
    h = usb_can.open_device(devs[0].index)
    usb_can.configure(h, bitrate=500000)
    usb_can.can_start(h)
    usb_can.send(h, 0x123, bytes(range(8)))
    for can_id, dlc, flags, data, ts_us in usb_can.receive(h, timeout_ms=1000):
        print(hex(can_id), data.hex())
    usb_can.can_stop(h)
    usb_can.can_close(h)
    usb_can.can_sdk_shutdown()
"""
from __future__ import annotations

from ctypes import byref, c_void_p

from . import _raw as r
from ._raw import (  # re-exports
    CAN_OK, CAN_ERR_INVALID_PARAM, CAN_ERR_NO_DEVICE, CAN_ERR_OPEN_FAILED,
    CAN_ERR_NOT_OPEN, CAN_ERR_SEND_FAILED, CAN_ERR_TIMEOUT, CAN_ERR_BUS_OFF,
    CAN_ERR_BUFFER_FULL, CAN_ERR_NOT_SUPPORTED, CAN_ERR_INTERNAL,
    CAN_STATE_OK, CAN_STATE_WARNING, CAN_STATE_PASSIVE, CAN_STATE_BUS_OFF,
    CAN_STATE_STOPPED, CAN_STATE_UNKNOWN,
    CAN_CAP_CANFD, CAN_CAP_LISTEN_ONLY, CAN_CAP_ONE_SHOT, CAN_CAP_AUTO_RESTART,
    CAN_FLAG_EXTENDED, CAN_FLAG_RTR, CAN_FLAG_FD, CAN_FLAG_BRS, CAN_FLAG_ERROR,
    CAN_MAX_NAME_LEN, CAN_MAX_DEVICES,
    CanMsg, CanDeviceInfo, CanConfig, CanStats,
    can_sdk_init, can_sdk_shutdown, can_close, can_start, can_stop, can_reset,
    can_get_capabilities, can_get_state, can_reset_stats,
)
from ._raw import status_str

__version__ = "5.0.0"


# ---------------------------------------------------------------------------
# Exception
# ---------------------------------------------------------------------------

class CanError(IOError):
    """SDK-level error.  ``code`` is the underlying ``can_status_t`` value."""

    def __init__(self, code: int, where: str = ""):
        self.code = int(code)
        msg = status_str(self.code)
        if where:
            msg = f"{where}: {msg}"
        super().__init__(f"{msg} (code={self.code})")


def _check(code, where: str = "") -> int:
    code = int(code)
    if code < 0:
        raise CanError(code, where)
    return code


# ---------------------------------------------------------------------------
# High-level wrappers
# ---------------------------------------------------------------------------

def discover(max_count: int = CAN_MAX_DEVICES):
    """Return a list of ``CanDeviceInfo`` for available devices."""
    arr = (CanDeviceInfo * max_count)()
    n = r.can_discover(arr, max_count)
    if n < 0:
        raise CanError(n, "can_discover")
    return [arr[i] for i in range(n)]


def open_device(index: int):
    """Open device by index. Returns an opaque ``c_void_p`` handle."""
    h = c_void_p()
    _check(r.can_open(byref(h), int(index)), "can_open")
    return h


def configure(handle, bitrate: int = 500000, sample_point: int = 875,
              canfd: bool = False,
              data_bitrate: int = 2000000, data_sample_point: int = 750):
    cfg = CanConfig(
        bitrate=int(bitrate),
        sample_point=int(sample_point),
        canfd_enabled=1 if canfd else 0,
        data_bitrate=int(data_bitrate),
        data_sample_point=int(data_sample_point),
    )
    _check(r.can_configure(handle, byref(cfg)), "can_configure")


def get_config(handle):
    cfg = CanConfig()
    _check(r.can_get_config(handle, byref(cfg)), "can_get_config")
    return cfg


def send(handle, can_id: int, data, flags: int = 0):
    """Send one frame. ``data`` is bytes (<=8 classic, <=64 FD)."""
    msg = CanMsg()
    msg.id    = int(can_id)
    msg.dlc   = len(data)
    msg.flags = int(flags)
    for i, b in enumerate(data):
        msg.data[i] = b & 0xFF
    _check(r.can_send(handle, byref(msg)), "can_send")


def receive(handle, max_count: int = 16, timeout_ms: int = 100):
    """Receive up to ``max_count`` frames. Returns a list of tuples
    ``(id, dlc, flags, data_bytes, timestamp_us)``."""
    arr = (CanMsg * max_count)()
    n = r.can_receive(handle, arr, max_count, int(timeout_ms))
    if n < 0:
        raise CanError(n, "can_receive")
    out = []
    for i in range(n):
        m = arr[i]
        out.append((m.id, m.dlc, m.flags, bytes(m.data[:m.dlc]), m.timestamp_us))
    return out


def stats(handle):
    s = CanStats()
    _check(r.can_get_stats(handle, byref(s)), "can_get_stats")
    return s


def name(handle) -> str:
    s = r.can_get_name(handle)
    return s.decode("utf-8", "replace") if s else ""


def version(handle=None) -> str:
    s = r.can_get_version(handle)
    return s.decode("utf-8", "replace") if s else ""


# ---------------------------------------------------------------------------
# Context-manager-friendly Device class
# ---------------------------------------------------------------------------

class Device:
    """Convenience wrapper. Manages SDK init + device lifecycle::

        with usb_can.Device(0) as d:
            d.configure(bitrate=500000)
            d.start()
            d.send(0x123, b"\\x01\\x02")
            for frame in d.receive(timeout_ms=1000):
                ...
    """

    def __init__(self, index: int = 0):
        self._index = int(index)
        self._handle = None
        self._sdk_initialized = False
        self._started = False

    def __enter__(self):
        if r.can_sdk_init() != CAN_OK:
            raise CanError(CAN_ERR_INTERNAL, "can_sdk_init")
        self._sdk_initialized = True
        self._handle = open_device(self._index)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._started:
            try: r.can_stop(self._handle)
            except Exception: pass
            self._started = False
        if self._handle is not None:
            r.can_close(self._handle)
            self._handle = None
        if self._sdk_initialized:
            r.can_sdk_shutdown()
            self._sdk_initialized = False
        return False

    def configure(self, **kw):       configure(self._handle, **kw)
    def get_config(self):            return get_config(self._handle)
    def start(self):                 _check(r.can_start(self._handle), "can_start"); self._started = True
    def stop(self):                  _check(r.can_stop(self._handle),  "can_stop");  self._started = False
    def reset(self):                 _check(r.can_reset(self._handle), "can_reset")
    def send(self, can_id, data, flags=0):  send(self._handle, can_id, data, flags)
    def receive(self, **kw):         return receive(self._handle, **kw)
    def stats(self):                 return stats(self._handle)
    def state(self):                 return r.can_get_state(self._handle)
    def capabilities(self):          return r.can_get_capabilities(self._handle)
    def name(self):                  return name(self._handle)
