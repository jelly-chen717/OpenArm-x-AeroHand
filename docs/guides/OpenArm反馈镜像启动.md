# OpenArm 反馈镜像

独立入口为 `apps/mirror_openarm_mujoco.py`；不会修改 8090/8769 等纯仿真服务。此版本实现反馈镜像，不包含仿真轨迹驱动实机。

在工作区根目录运行：

```powershell
& 'openarm/.venv/Scripts/python.exe' 'apps/mirror_openarm_mujoco.py' --duration 10
```

默认使用明确标记为 synthetic 的双臂样例，不打开 CAN。窗口左上角显示来源、左右有效性和数据年龄；终端同步输出状态。样例包含左臂缺帧及恢复，陈旧时该臂保留最后有效姿态。`--headless` 用于无窗口验证。日志默认保存至 `logs/openarm/sync/mirror_*.jsonl`，可通过 `--replay 日志路径` 回放。回放含有 NaN 的外部数据须用 JSON null 表示；不会将缺帧补成假反馈。

当前 `configs/openarm_joint_mapping.json` 的 `model_candidate` 用于回放，`hardware_mapping` 已按用户确认的零位、方向及1∶1比例填写，版本为hardware-user-confirmed-v1-20261115。真实模式按 --side 选择的臂校验映射标记与证据，每臂必须齐备七轴；both 要求14轴。未满足时在打开 CAN 前拒绝启动。

真实采集接入条件：先完成 A02 总线唯一归属、A03 通道/状态核对、A04 周期与阈值测量，以及 A06/A07 映射适用性核对。下面仅是条件满足后的命令模板，本次未执行：

```powershell
& 'openarm/.venv/Scripts/python.exe' 'apps/mirror_openarm_mujoco.py' --mode hardware --side right --right-channel 0x52 --left-channel 0x51 --period 0.03 --max-age 0.15 --max-skew 0.05 --confirm-bus-ownership --duration 60
```

真实源仅调用 `query_motor()`，查询七个臂电机反馈，不调用 enable、MIT、disable 或参数写入。读取也会发送状态查询帧并配置 PCAN 适配器，不是总线监听。打开或查询失败后会清空该臂批次、释放旧源并等待0.5秒重建；下一完整有效批次恢复后刷新姿态。2026-11-15已完成真实USB断开重接恢复验收。

`--max-age`、`--max-skew` 和 `--period` 默认值为离线初值，现场必须根据测量调整。窗口关闭会停止采集线程、释放适配器；不主动改变电机状态。

验证命令：

```powershell
& 'openarm/.venv/Scripts/python.exe' -m pytest apps/test_openarm_sync.py apps/test_openarm_driver.py -q
```

2026-11-15 相关测试29 passed。双臂真实反馈十秒采样通过，每侧413轮完整反馈，周期中位数约24.1 ms。右臂手动动作确认对应0x52。后续零位、比例确认及USB恢复结果已更新到统一实施计划。

## 现场标定候选预览

2026-11-15已确认物理右臂0x52、左臂0x51。双臂姿态已人工对照，用户已确认零位和1∶1比例。以下候选预览供后续临时映射对照使用。使用下列显式选项可查看真实反馈的候选模型姿态，窗口及日志均显示UNVERIFIED MODEL CANDIDATE。该选项使用model_candidate，正式镜像使用已填写的hardware_mapping。

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/mirror_openarm_mujoco.py --mode hardware --side right --right-channel 0x52 --left-channel 0x51 --preview-candidate-mapping --confirm-bus-ownership --period 0.03 --max-age 0.15 --max-skew 0.05 --duration 60
```

所选侧以外的模型关节保留默认姿态。观察结果按实际选定侧记录。

2026-11-15右臂60秒候选预览退出码0，用户确认姿态一致且手动变化可同步显示；有效显示记录873条，年龄最大47ms、采样差最大16ms。自动退出后双臂均在线失能。候选预览与精确标定验收状态分别记录在统一实施计划中。

实机执行、通道交接及宿主权限说明见 `docs/guides/OpenArm标定与轨迹执行.md`。
