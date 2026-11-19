# USB2CANFD Upgrade Tool

[中文](README.zh.md) | EN

## Overview

This directory is the canonical source for the USB2CANFD series upgrade tool. It is intended to become the shared tool subrepository `usb2canfd-upgrade-tool`.

It serves these visible entries:

- Parent repository entry: `工具和上位机/USB2CANFD通用升级工具`
- `USB2CANFD` module repository entry: `工具/USB2CANFD通用升级工具`
- `USB2CANFD-Dual` module repository entry: `工具/USB2CANFD通用升级工具`

Only this canonical copy should be maintained. Module repository entries should mount the same tool repository as a submodule instead of copying the zip package.

## Resources

- [安装包/USB2CANFD系列升级工具.zip](安装包/USB2CANFD系列升级工具.zip)

## Maintenance Policy

- `安装包/` stores public release packages and installers.
- The repository root stores README files, release notes, checksums, and other entry documents.
- Upgrade steps and version compatibility notes should be added here instead of being duplicated in module folders.
- Gitee keeps the Chinese source path; GitHub publishing maps it to `USB2CANFD-Upgrade-Tool`.
