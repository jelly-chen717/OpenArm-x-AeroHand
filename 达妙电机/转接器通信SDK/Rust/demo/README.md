# Rust 示例（dmcan 封装层）

最小可跑示例：枚举设备 → 打开 → 读固件信息 → 配置通道 → 订阅收帧 → 发一帧 → 读通道统计。
源码见 [`src/main.rs`](src/main.rs)，每一步都写了注释。

## 环境要求

| 项 | 要求 |
| --- | --- |
| Rust | 1.77 及以上 |
| libusb（本示例不含） | **Windows**：把 `libusb-1.0.dll` 放到本目录或 `PATH`；**Linux**：装系统包（如 `apt install libusb-1.0-0`）；**macOS**：把 `libusb-1.0.0.dylib` 放到与 `libdmcan.dylib` 同目录 |
| 模块 | USB2CANFD 系列（单路 / 双路 / LinkX4C），固件 V2.0.0 及以上；固件构建标识要求见上一层 [README](../README.md) |

## 目录结构

```
demo/
├─ Cargo.toml        本示例工程（path 依赖同目录下的 dmcan 封装层）
├─ src/main.rs       示例源码
└─ dmcan-2.1.0/      dmcan 封装层（由 ../pack/dmcan-2.1.0.crate 解出，自带各平台原生库）
```

## 运行

```bash
cargo run
```

- 接了模块：打印设备信息、固件版本、发送 / 接收情况与通道统计；
- 没接模块：打印"没有找到设备"后正常退出（退出码 0）；
- 想用单模块做自发自收：把通道切到"外部回环"模式（不依赖外部 ACK），本示例用的是普通收发模式。

> 首次 `cargo run` 会在 `demo/target/` 下编译（已在 `.gitignore` 里）。

## 注意事项

- **先配参数、再使能通道**：参数下发后通道会回到未使能状态，顺序反了之后发的帧会被丢弃；
- 回调在 SDK 自己的线程上同步执行：不要在里面做落盘 / 阻塞操作；回调里的帧数据只在回调期间有效，要留存请 `to_owned()`；
- `open()` 会独占模块的 USB2UART 串口（默认自动打开），此时串口助手打不开那个口。
