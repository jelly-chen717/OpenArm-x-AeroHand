# DM-USB2CANFD

中文 | [EN](./README.en.md) | GitHub 展示仓：`dmBots/dmBot`

达妙科技 **USB 转 CAN FD 模块**的对外资料与固件发布仓：出厂固件、说明书、外壳模型都在这里；
二次开发用的 SDK 已独立成仓，见下方「SDK（二次开发）」章节。

## 仓库内容

| 路径 | 内容 |
| --- | --- |
| [固件/](固件/) | 模块出厂固件（加密 `.enc`，用模块配套上位机做固件升级） |
| [达妙科技-USB转CANFD模块使用说明书V1.0.pdf](达妙科技-USB转CANFD模块使用说明书V1.0.pdf) | 说明书：驱动安装、连接、上位机使用与固件升级步骤 |
| [外壳模型/](外壳模型/) | 官方外壳模型（v2.1，PDF 预览 + STP） |
| [网友分享-开源外壳/](网友分享-开源外壳/) | 网友分享的开源外壳（STEP） |

## 快速开始

- **刚拿到模块**：先看[说明书 PDF](达妙科技-USB转CANFD模块使用说明书V1.0.pdf)，完成驱动、权限与上位机准备。
- **升级 / 回退固件**：用模块配套上位机选择 [固件/](固件/) 里的 `.enc` 刷写，具体步骤见说明书。
- **想自己写程序**：直接进 [DM Device SDK](https://gitee.com/kit-miao/dm-device-sdk)。
- **要画结构件**：取[外壳模型/](外壳模型/)或[网友分享-开源外壳/](网友分享-开源外壳/)。
- **遇到使用问题**：官方论坛 <https://bbs.dmbot.cn>，或官方 QQ 群 `1015902491`。

## SDK（二次开发）

SDK 已拆到独立仓库，包含 C / C++ / C# / Python 封装、示例与更新说明：

- **[DM Device SDK](https://gitee.com/kit-miao/dm-device-sdk)**

> - 早期随本仓发布的 USB2CANFD 旧版 SDK 资料不再随本仓维护，请统一使用上面的 DM Device SDK。
> - 模块底层通信协议不开放；二次开发请走官方 SDK，不要直接按抓包结果对接。

## 固件

| 文件 | 版本 | 说明 |
| --- | --- | --- |
| [固件/dm_usb2canfd_app_v2.0.0.enc](固件/dm_usb2canfd_app_v2.0.0.enc) | 2.0.0 | 当前出厂 app 固件（加密镜像，供配套上位机升级用） |

- 升级方式：模块配套上位机 → 固件升级 → 选择 `.enc` 文件；详细步骤见[说明书](达妙科技-USB转CANFD模块使用说明书V1.0.pdf)。

## 状态与维护

- ZH：主版入口（本页）；EN：[README.en.md](README.en.md)（轻量英文入口）。
- 本仓内容从内部受控资料同步而来；GitHub 镜像的覆盖范围见 [GitHub / Gitee 覆盖范围说明](https://gitee.com/kit-miao/damiao/blob/master/docs/repository/mirror-scope.md)。
- 文档与固件的更新说明，以后续发布的公开资料为准。
