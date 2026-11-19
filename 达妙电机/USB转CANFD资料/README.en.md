# DM-USB2CANFD

[EN](./README.en.md) | [中文](./README.md)

Firmware, documentation and enclosure models for the DM USB2CANFD module (Gitee release repository).

## What's inside

| Path | Content |
| --- | --- |
| [固件/](固件/) | Factory app firmware (encrypted `.enc`, flashed with the vendor host tool) |
| [达妙科技-USB转CANFD模块使用说明书V1.0.pdf](达妙科技-USB转CANFD模块使用说明书V1.0.pdf) | User manual (Chinese): driver, connection, host tool and firmware upgrade steps |
| [外壳模型/](外壳模型/) | Official enclosure model (v2.1, PDF preview + STP) |
| [网友分享-开源外壳/](网友分享-开源外壳/) | Community shared enclosure (STEP) |

## Quick start

- **New to the module**: read the user manual PDF first.
- **Upgrade or roll back firmware**: pick the `.enc` under [固件/](固件/) in the vendor host tool; detailed steps are in the manual.
- **Custom development**: use the **[DM Device SDK](https://gitee.com/kit-miao/dm-device-sdk)** (C / C++ / C# / Python bindings, examples, update notes). The legacy in-repo SDK is no longer maintained here.
- **Enclosure design**: [外壳模型/](外壳模型/) or [网友分享-开源外壳/](网友分享-开源外壳/).

## Firmware

| File | Version | Note |
| --- | --- | --- |
| [固件/dm_usb2canfd_app_v2.0.0.enc](固件/dm_usb2canfd_app_v2.0.0.enc) | 2.0.0 | Current factory app firmware (encrypted image) |

## Status

- ZH: primary entry ([README.md](./README.md)); EN: this lightweight entry.
- This repository mirrors materials from the internal governed source; see the [GitHub / Gitee mirror scope note](https://gitee.com/kit-miao/damiao/blob/master/docs/repository/mirror-scope.md).
