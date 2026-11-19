# DM-DeviceSDK Python 使用说明

`libdmcan`（DM-DeviceSDK 的 C 后端）的 **Python 封装**：纯 ctypes，无编译依赖。
支持设备枚举与打开、通道与位定时、CAN / CAN FD 收发、周期与批量发送、真 echo、硬件过滤表、
USB2UART 透传、热插拔与日志订阅。

## 版本对应

| Python 包 | 对应的 libdmcan | 说明 |
| --- | --- | --- |
| **2.1.1**（当前） | **2.1.0** | 包版本 major.minor 跟随所绑定的后端库；2.1.1 相对 2.1.0 只有元数据/文档变化 |
| 2.1.0 | 2.1.0 | 同一 ABI 的前一版（`pack/` 里保留） |
| 1.0.4（旧） | 1.1.x（旧） | 旧接口，与 2.x **不兼容**，升级见下文“从 1.x 升级” |

`pack/` 里同时保留历史版本（2.1.0 与 1.0.4 的 wheel），方便对照与仍停留在旧版的项目；
**新项目请用 2.1.1**。

`DmCanContext` 在构造时会校验后端库：主版本必须相同、次版本不低于 2.1，不配套会直接报错
（缺符号会点名是哪个），不会静默读到错位数据。

## 环境要求

- **Python 3.9 及以上**（3.9.13 / 3.12.10 / 3.13.5 均已实跑验证）
- **libdmcan 2.1.0** 后端库：见本仓库 [`../C&C++/lib/v2.1.0/`](../C&C++/lib/v2.1.0/)，
  Windows 下文件名是 `libdmcan.dll`
- 操作系统：Windows 10/11 (x64)；Linux / macOS 同样可用（后端库换成 `libdmcan.so` / `libdmcan.dylib`）
- 硬件：USB2CANFD / USB2CANFD_Dual / LinkX4C 等模块；**固件需支持 2 字节通道开关应答**（见“常见问题”）

## 目录结构

```
Python/
├── pack/     发布产物：dmcan_sdk-2.1.1-py3-none-any.whl / .tar.gz（另附 2.1.0、1.0.4 历史包）
├── demo/     最小示例：demo.py + dlls/（后端库与依赖 DLL）
└── README.md 本说明
```

## 安装

离线安装（推荐，用 `pack/` 里的 wheel）：

```bash
pip install pack/dmcan_sdk-2.1.1-py3-none-any.whl
```

或从 PyPI 安装：`pip install dmcan_sdk`（<https://pypi.org/project/dmcan-sdk/>）

## 后端库怎么放（重要）

SDK 按下面的顺序找后端库，命中第一个就用：

1. 环境变量 `DMCAN_LIBRARY_PATH` 指向的完整路径；
2. Python 包内的 `dmcan/dlls/`；
3. 当前程序目录下的 `dlls/`；
4. 当前程序目录；
5. 交给系统搜索路径。

Windows 上 `libdmcan.dll` 还依赖 `libusb-1.0.dll`、`libstdc++-6.dll`、`libgcc_s_seh-1.dll`、
`libwinpthread-1.dll`：**这几个必须和 `libdmcan.dll` 放在同一个目录**（`demo/dlls/` 里已按这个摆好）。
SDK 会把后端库所在目录加进 DLL 搜索路径，所以只要 `dlls/libdmcan.dll` 到位，依赖就能被找到。

## 快速开始

```python
import os, pathlib
os.environ.setdefault("DMCAN_LIBRARY_PATH",
                      str(pathlib.Path(__file__).resolve().parent / "dlls" / "libdmcan.dll"))

from dmcan import DmCanContext, dmcan_event_flag

with DmCanContext() as ctx:
    print("库版本 / 设备数：", ctx.sdk_version(), ctx.find_devices())
    dev = ctx.get_device(0)
    dev.open()                              # 会独占设备的 UART COM 口
    print(dev.get_info(), dev.get_version())  # 型号/VID/PID/序列号、固件

    dev.enable_channel(0, True)
    sub = dev.subscribe(dmcan_event_flag.RX_FRAME,
                        lambda d, e: print(e.frame), channel_mask=1 << 0)
    dev.send_can(0, 0x123, b"\x11\x22\x33\x44", canfd=True, brs=True)
    sub.cancel()
    dev.disable_channel(0)
    dev.close()
```

完整可跑版本见 `demo/demo.py`：

```bash
cd demo
python demo.py
```

## 常见问题

- **失败一律抛 `DmCanError`**（`exc.status` 是错误码），不再返回 `False`；
- `open()` 默认会**独占**设备的 UART COM 口，此时串口助手 / 别的程序打不开；
- 通道参数默认只改运行期，**掉电就回**：要保留得调 `save_param()`；
- `get_channel_stats()` 在固件首条周期上报（1 Hz）到达前返回 `None`，那是“还没有值”不是失败；
- **固件要求**：通道开 / 关要求固件回 **2 字节** `{操作结果, 状态}` 应答；老固件回 1 字节时这两个接口
  会返回 false，库会打 ERROR 日志提示升级固件；
- 看门狗：使能后 SDK 按**超时的一半**周期喂狗；老固件上“关狗”命令可能被拒，此时会话结束后通道会被咬，
  重新 `enable_channel()`（或给设备断电）即可恢复。

## 从 1.x 升级到 2.x（不兼容）

| 旧写法（1.0.x） | 新写法（2.x） |
| --- | --- |
| 后端库 `libdm_device.dll` | **`libdmcan.dll`**（文件名变了） |
| `send_can(ch, id, dlen, payload, ...)` | `send_can(ch, id, payload, ...)`（长度由 payload 决定） |
| `usb_rx_frame` / `frame.head.dlc` | `dmcan_rx_frame` / `frame.dlen`（字节长度） |
| `device.print_version()` | `device.get_version()`（读固件信息并格式化） |
| `dmcan_channel_can_config` | `dmcan_ch_baud_conf` |
| 失败返回 `False` | 失败抛 `DmCanError` |

## 相关

- C / C++ SDK（同仓库）：[`../C&C++/`](../C&C++/)，版本更新与迁移说明见 [`../C&C++/UPDATE.md`](../C&C++/UPDATE.md)
- Rust SDK（同仓库）：[`../Rust/`](../Rust/)
