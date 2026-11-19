RobotControl VR 工作站 · Windows 11 / WSL2 完整控制部署包
版本：1.0.0-rc1 / 2026-11-11

这不是远程网页客户端。USB-CAN 和三台相机连接本台 Windows 电脑，
通过 usbipd 交给本机 WSL2 Linux，再由同一套 ROS/CAN、IK、相机服务控制。
安装包含完整控制 Docker 镜像、Ubuntu 22.04 WSL 根文件系统、应用源码和界面。
Windows硬件透传尚未在新机验收，不能承诺装包后所有USB设备直接可用。
RealSense 官方不将虚拟机安装列为支持方案；WSL的USB链路需实际测三路连续采集。

前提
Windows 11 x64，CPU支持虚拟化和AVX2，建议32GB内存、60GB以上空余空间。
Linux/WSL可见内存至少12GiB。安装和可选内核编译仍需网络及管理员权限。
同 Ubuntu 包限定 PEAK 0c72:0011 双路FD、D435+两台D405。先核对本套设备零位与方向。
头显与Windows在同一可互通局域网；使用固定/保留的Windows IPv4，避免DHCP地址变化。
此版本使用WSL默认NAT模式，由脚本将Windows 4448转发到WSL 4448。

首次安装顺序
将ZIP完整解压到固定目录（桌面入口依赖其中的windows脚本，不要随后移动）。
以管理员身份打开PowerShell，进入解压目录：
  powershell -ExecutionPolicy Bypass -File .\windows\Install-RobotControl.ps1
按提示输入头显将访问的 Windows 局域网 IPv4。
若首次启用WSL需要重启，重启后重复运行同一安装命令。
该脚本创建独立 RobotControl 发行版，不接管同名的非本项目发行版。

  powershell -ExecutionPolicy Bypass -File .\windows\Attach-Devices.ps1
按 usbipd 列表选择 PEAK 和三台 RealSense 的 BUSID。
设备透传期间由 Linux 独占，Windows 相机软件不可同时占用。
若 doctor 报 SocketCAN/peak_usb 不可用，则先运行：
  powershell -ExecutionPolicy Bypass -File .\windows\Build-WSL-Kernel.ps1
这会从固定的 Microsoft 官方内核提交构建含 CAN_RAW/PEAK_USB 的内核。
编译可能较久，额外预留20GB空间。脚本先备份 .wslconfig，输入APPLY后才切换。
自定义内核影响本用户所有WSL2发行版，切换时会停止WSL；先保存其他WSL工作。
完成后重新执行 Attach-Devices.ps1。默认内核已有所需驱动时无需编译。

  powershell -ExecutionPolicy Bypass -File .\windows\Configure-Devices.ps1
确认 can0右 / can1左，并输入本套三台相机序列号。

  powershell -ExecutionPolicy Bypass -File .\windows\Start-RobotControl.ps1
以后也可双击桌面 RobotControl 启动。脚本仅启动后台并打开网页，不使能电机。
头显访问 https://Windows局域网IP:4448/ ，首次确认该机HTTPS证书。
网页操作流程、手柄 A/X/Y/B 和采集目录见 README-Ubuntu.txt。

USB与网络
重插USB、重启或WSL关闭后，可能需要重新运行 Attach-Devices.ps1。
保留本项目端口代理回执；脚本拒绝覆盖无法确认归属的既有4448转发。
Windows当前网络应为受信任的专用网络。脚本防火墙只开放本地子网的4448。
换IP时需要更新证书、允许的公开地址和端口转发，不能只改浏览器URL。
如果 USB-CAN 已进入 WSL 但没有 can0/can1，先核对内核和驱动，不要直接使能。
如果三路画面断流，检查USB3带宽/供电及透传，完整控制环境不等于透传性能已验证。

诊断
  wsl -d RobotControl -u root -- python3 /opt/robotcontrol/doctor.py
关闭服务前先在网页关闭双臂，再运行：
  wsl -d RobotControl -u root -- python3 /opt/robotcontrol/stop.py
不要在电机使能时执行 wsl --shutdown 或拔除 USB-CAN。

本轮验证为脚本语法/逻辑测试和Ubuntu上的隔离运行；
未执行这台Windows的WSL安装、内核编译或实际USB-CAN/RealSense透传验收。
资料：
https://learn.microsoft.com/en-us/windows/wsl/connect-usb
https://github.com/microsoft/WSL2-Linux-Kernel
https://github.com/realsenseai/librealsense/blob/master/doc/installation.md
