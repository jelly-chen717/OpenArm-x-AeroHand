# USB2CANFD 通用升级工具

中文 | [EN](README.en.md)

## 概述

本目录是 `USB2CANFD` 系列升级工具的 canonical 源头，后续建议独立成共享工具子仓 `usb2canfd-upgrade-tool`。

它同时服务以下入口：

- 父仓全局入口：`工具和上位机/USB2CANFD通用升级工具`
- `USB2CANFD` 模块仓内部入口：`工具/USB2CANFD通用升级工具`
- `USB2CANFD-Dual` 模块仓内部入口：`工具/USB2CANFD通用升级工具`

多入口发布时只允许维护这一份工具内容，模块仓内部入口通过子模块挂载同一个工具仓，不复制 zip。

## 资源

- [安装包/USB2CANFD系列升级工具.zip](安装包/USB2CANFD系列升级工具.zip)

## 维护规则

- `安装包/` 只放公开发布包或安装包。
- 根目录只放 README、版本说明、校验文件等入口文档。
- 具体升级步骤和版本兼容说明后续补到本目录，不分散到模块目录中重复维护。
- Gitee 端保留中文目录；GitHub 端由发布脚本映射为 `USB2CANFD-Upgrade-Tool`。
