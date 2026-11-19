# usb_can — Python interface for the USB-CAN SDK

`usb_can` is a Python package that wraps the SAVVYCANFD USB-CAN / CAN FD
SDK. It uses `ctypes` to call the same shared library (`usb_can.dll` on
Windows, `libusb_can.so` on Linux, `libusb_can.dylib` on macOS) as the C
example program and the Qt viewer, so behaviour matches across all three
languages.

The standalone `SAVVYCANFD-Python` package bundles the native libraries
for Windows x64, Linux x64 and Linux ARM64 inside `usb_can/_libs/<platform>/`.
Extract, install Python deps, run — no build step required.

---

## Requirements

| Component | Version |
|---|---|
| Python  | 3.8 – 3.12 |

No external Python dependencies — pure-`ctypes` wrapper.

On Linux, bring up a `canX` interface before discovery:

```bash
sudo ip link set can0 up type can bitrate 500000
```

Windows and macOS device driver requirements depend on the adapter you
plug in — see the adapter vendor's documentation.

---

## Install

### From the standalone customer package

```
SAVVYCANFD-Python/
├── usb_can/
│   └── _libs/
│       ├── windows-x64/   usb_can.dll (+ its runtime deps)
│       ├── linux-x64/     libusb_can.so.5.0.0 + symlinks
│       └── linux-arm64/   libusb_can.so.5.0.0 + symlinks
├── examples/
├── tests/
├── pyproject.toml
├── README.md
├── run_basic_capture.{bat,sh}
├── run_recv_loop.{bat,sh}
└── install_deps.{bat,sh}
```

**Windows:**
```bat
unzip SAVVYCANFD-Python.zip
cd SAVVYCANFD-Python
install_deps.bat              :: optional — no required external deps
run_basic_capture.bat
```

**Linux:**
```bash
unzip SAVVYCANFD-Python.zip
cd SAVVYCANFD-Python
chmod +x *.sh
./install_deps.sh             # optional
./run_basic_capture.sh
```

### Override the bundled library

```bash
# Linux / macOS
export USB_CAN_LIB=/path/to/libusb_can.so
python examples/basic_capture.py

# Windows
set USB_CAN_LIB=C:\path\to\usb_can.dll
python examples\basic_capture.py
```

---

## Quick start

```python
import usb_can

# Function-style API
usb_can.can_sdk_init()
devs = usb_can.discover()
h = usb_can.open_device(devs[0].index)
usb_can.configure(h, bitrate=500000)
usb_can.can_start(h)

usb_can.send(h, 0x123, bytes(range(8)))

for can_id, dlc, flags, data, ts_us in usb_can.receive(h, timeout_ms=1000):
    print(f"RX 0x{can_id:X}  {data.hex()}")

usb_can.can_stop(h)
usb_can.can_close(h)
usb_can.can_sdk_shutdown()
```

```python
# Class-style API with context manager
import usb_can

with usb_can.Device(0) as d:
    d.configure(bitrate=500000)
    d.start()
    d.send(0x123, bytes(range(8)))
    for frame in d.receive(timeout_ms=1000):
        print(frame)
```

---

## API reference

### Module-level functions

| Function                  | Returns                | Notes                     |
|---------------------------|------------------------|---------------------------|
| `can_sdk_init()`          | int (0 = OK)           | call once at start        |
| `can_sdk_shutdown()`      | int (0 = OK)           | call once at exit         |
| `discover(max_count=16)`  | list[CanDeviceInfo]    | scan for devices          |
| `open_device(index)`      | c_void_p handle        | raises `CanError` on fail |
| `can_close(handle)`       | int                    |                           |
| `configure(handle, bitrate=, sample_point=, canfd=, data_bitrate=, data_sample_point=)` | — | raises `CanError` |
| `get_config(handle)`      | CanConfig              |                           |
| `can_start(handle)`       | int                    |                           |
| `can_stop(handle)`        | int                    |                           |
| `can_reset(handle)`       | int                    |                           |
| `can_get_state(handle)`   | int (CAN_STATE_*)      |                           |
| `send(handle, can_id, data, flags=0)`           | —          | raises `CanError`         |
| `receive(handle, max_count=16, timeout_ms=100)` | list[tuple] | (id, dlc, flags, bytes, ts_us) |
| `stats(handle)`           | CanStats               |                           |
| `name(handle)`            | str                    |                           |
| `version(handle=None)`    | str                    |                           |
| `status_str(code)`        | str                    |                           |

### `usb_can.Device` (context manager)

| Method                    | Returns       |
|---------------------------|---------------|
| `Device(index=0)`         | Device        |
| `.configure(**kw)`        | —             |
| `.get_config()`           | CanConfig     |
| `.start()` / `.stop()` / `.reset()` | —    |
| `.send(can_id, data, flags=0)` | —        |
| `.receive(**kw)`          | list[tuple]   |
| `.stats()`                | CanStats      |
| `.state()`                | int           |
| `.capabilities()`         | int           |
| `.name()`                 | str           |

### Constants

`CAN_OK`, `CAN_ERR_*`, `CAN_STATE_*`, `CAN_FLAG_*`, `CAN_CAP_*`,
`CAN_MAX_NAME_LEN`, `CAN_MAX_DEVICES`.

### Exception

`usb_can.CanError` (subclass of `OSError`). Carries `.code` (SDK status
code) and a human-readable message from `status_str()`.

---

## Library search order

`usb_can` loads the native library from these locations, first hit wins:

1. `$USB_CAN_LIB` environment variable (full path)
2. Bundled `usb_can/_libs/<platform>/`
3. Sibling files in the package directory
4. System loader search path

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `OSError: Failed to locate usb_can shared library` | Set `USB_CAN_LIB` to point at your library |
| Windows: `OSError: [WinError 126]` | Companion runtime DLL missing next to `usb_can.dll`; copy the full contents of `_libs/windows-x64/` alongside it |
| Linux: `libusb_can.so: cannot open shared object file` | Library not in package; use the standalone customer package, or set `LD_LIBRARY_PATH=...` |
| `discover()` returns empty | No supported device plugged in / not enumerated. Linux: check `ip link show`; Windows: check Device Manager. |
| `configure()` raises `CanError: Not supported` | Adapter doesn't support CAN FD, or bitrate is invalid |
