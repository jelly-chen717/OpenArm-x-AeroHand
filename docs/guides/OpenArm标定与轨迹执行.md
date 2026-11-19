# OpenArm 标定与轨迹执行

> 2026-11-16 限幅更新：专用硬件入口默认 `--limit-profile rated`，左右J1/J2采用20Nm、10.471976rad/s，J3/J4采用9Nm、3.769911rad/s，J5～J7采用3Nm、12.566371rad/s（24V额定输出轴参数）。加载旧settings也会覆盖其中速度/力矩阈值；显式 `--limit-profile custom` 才沿用任务阈值。历史请求/日志不改写，新策略完成87项离线测试，尚无本轮实机验证。

> 2026-11-16 坐标更新：当前映射版本为 `hardware-unified-forward-inward-v2-20261116`。左 J1/J6/J7、右 J2 逻辑角度取反，双臂 J1/J7 正向前、J2/J6 正向里。下文历史验收角度及旧请求仍为原坐标；新执行必须按新坐标重新准备请求，不能仅替换版本号。模型、发送与反馈已一致换算，原零位和比例保留；新约定只完成离线验证。完整参数与迁移说明见 [关节详细资料](../../关节详细资料.md)。

## 已验证范围

2026-11-15完成双臂反馈镜像、共用USB适配器断连后自动恢复、右J7保持与小幅往返，以及MuJoCo预演→实机执行→真实反馈回写MuJoCo的专用链路。用户已确认硬件零位、双臂方向对应及1∶1比例，软件映射版本为 `hardware-user-confirmed-v1-20261115`。

专用实机入口已扩展为同一臂任意已映射关节组合，单J7旧命令继续可用。右J6/J7一秒实机保持已通过；2026-11-16左J7+25°/J6+25°/J5+45°的23秒序列按用户指定5°容差通过。新增J4+15°与四轴持续保持因软件力矩阈值停止，仍未完成现场验收。双臂协调及钢琴前端实时目标接入继续待扩展。已验收的右J7往返目标幅度0.02rad，使用kp=30、kd=2，途经点容差0.01rad。最终链路最大跟踪误差0.008703rad、RMS 0.005556rad、回程误差0.0003815rad；实测运动范围0.011444rad，静态跟踪精度仍有提升空间。

## 总线归属及运行环境

真实硬件命令需要可访问Windows设备驱动的宿主环境。在Codex的沙箱中，PCAN可能返回0x1400，CIM设备及进程查询也可能受限。设备状态应结合宿主查询判断。

物理左臂对应0x51，右臂对应0x52。镜像、状态查询及执行入口共用通道租约；其他PCAN工具的占用仍需现场核对。切换执行前，关闭镜像窗口或创建启动时指定的stop文件，待日志出现exit后再连接。执行期间由专用执行入口负责收发。

## 反馈镜像与断连恢复

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/mirror_openarm_mujoco.py --mode hardware --side both --left-channel 0x51 --right-channel 0x52 --confirm-bus-ownership --period 0.03 --max-age 0.15 --max-skew 0.05 --duration 3600 --stop-file logs/openarm/sync/mirror.stop
```

启动前清理上一轮已使用的stop文件，或指定新路径。查询源遇到打开或读取异常后释放旧源，等待0.5秒重建。该侧无有效数据期间保持最后有效姿态，恢复七轴完整、有效的新批次后更新。现场USB断开测试的无效区间约12秒，包含实际拔插等待，摘要保存在 `logs/openarm/sync/reconnect_onsite_20261115_02_summary.json`。

## 专用轨迹格式

JSON请求字段包含request_id、mapping_version、expires_at（Unix秒）、joints、initial_model_position、limits和points。每个point包含相对时间t与模型角q，角度rad、时间s。初始时间为0，时间严格递增。limits包含velocity_rad_s、acceleration_rad_s2、start_error_rad。

每段采用端点速度和加速度为零的五次插值，完整经过路径点。相同位置的相邻点形成停留段。硬件预检使用当前实测起点，执行前再次确认起点变化≤0.01rad，并检查模型限位、当前位置相对行程、速度、MIT模式及协议范围。

执行器使用高分辨率perf_counter调度。过期目标跳过，发送当前时刻目标，超时或跟踪异常进入FAULT并尝试失能。途经点存在停留段时，在停留结束检查该点误差，其他途经点在到达时检查。request_id在实机执行前以排他文件记录到 `logs/openarm/executed_requests/`，跨进程重复请求被拒绝。重试使用新的有效请求并重新检查起点。

本轮可复核的请求为 `logs/openarm/sync/execution_onsite_20261115/mujoco_roundtrip_request.json`。它的有效期用于当次现场执行，后续运行应生成新的请求ID、有效期及实测起点。

## MuJoCo预演与实机执行

### 单臂多关节配置

轨迹的joints按发送顺序列出同一臂的关节，每个q数组及initial_model_position使用相同顺序。扩展至J7以外的关节时，传入 `--joint-settings 配置.json`。配置为JSON数组，每行含side、motor_id、kp、kd、max_travel、max_speed、max_torque、max_temperature、max_age，且与轨迹参与轴精确匹配。角度rad、速度rad/s、力矩Nm、温度℃、反馈年龄s。各行必须填写全部字段，增益满足MIT协议范围，保护阈值为正有限数。

`configs/openarm_execution_right_wrist.json` 为右J6/J7示例配置，J6采用kp=20、kd=2，J7采用kp=30、kd=2，逐轴行程0.04rad、速度12.566371rad/s、力矩3Nm、温度50℃、反馈年龄150ms。速度/力矩已更新为额定上限，此组合未完成实机验收。额定上限不自动缩短轨迹时长，任务请求的速度、加速度和时序仍需单独设置。

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/drive_openarm_from_sim.py 新双轴请求.json --mode hardware --action preflight --joint-settings configs/openarm_execution_right_wrist.json --left-channel 0x51 --right-channel 0x52 --confirm-bus-ownership --period 0.04 --log 新双轴预检.jsonl
```

预检读取双臂失能状态和各参与轴模式、协议范围、实测起点，并验证完整轨迹。执行时使用同样参数，改为 `--action execute`，附加 `--cancel-file 新取消文件.stop` 及需要的 `--mirror`。日志逐轴保存实际增益和保护参数，motor_feedback保留各轴接收时间，可据此统计采样时间差。

各轴共用同一次轨迹采样时刻，按joints顺序逐轴发送并读取反馈。总线发送和反馈存在轴间时间差。参与轴增加后需实测周期，超过执行时间预算或反馈年龄阈值会进入故障退出。中途使能或发送失败时，对所有已尝试使能的轴执行失能和反馈读回。每次控制先校验整批目标再发送。

先在独立入口做虚拟预演。

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/drive_openarm_from_sim.py 新请求.json --mirror --period 0.04 --log 新预演日志.jsonl
```

使用硬件模式只查询并预检。

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/drive_openarm_from_sim.py 新请求.json --mode hardware --action preflight --left-channel 0x51 --right-channel 0x52 --confirm-bus-ownership --period 0.04 --kp 30 --kd 2 --log 新预检日志.jsonl
```

现场机械支撑及停止手段就绪、预演和预检通过后执行。

```powershell
& 'openarm/.venv/Scripts/python.exe' apps/drive_openarm_from_sim.py 新请求.json --mode hardware --action execute --mirror --left-channel 0x51 --right-channel 0x52 --confirm-bus-ownership --period 0.04 --kp 30 --kd 2 --endpoint-tolerance 0.01 --cancel-file logs/openarm/sync/cancel.stop --log 新实机日志.jsonl
```

当前保护默认值为相对起点行程0.12rad、反馈及目标速度/反馈力矩采用顶部逐轴额定值、温度50℃、反馈年龄150ms。完整逐轴settings仍覆盖行程、增益、温度和反馈年龄。日志 `preflight.effective_limits` 与 `joint_settings.joints` 为最终实际生效值。示例增益仅经过右J7上述小幅轨迹验证。执行时可创建cancel文件或使用Ctrl+C请求停止，退出执行器进行最多三次失能及反馈读回，日志保留STOPPED或STOP_UNCONFIRMED和双臂退出状态。物理断链时可能无法收到失能确认，应使用现场停止手段。

`--mirror` 更新无窗口MuJoCo。虚拟模式反馈标记为synthetic_virtual，实机模式标记为hardware。实机模式先载入双臂启动快照，运行中仅更新参与执行轴的真实反馈，日志mujoco_live_joints列出这些轴。

## 目标误差、时间阈值与持续保持

| 参数 | 含义 | 默认值 | 本轮用户确认值 |
| --- | --- | --- | --- |
| --endpoint-tolerance | 停留末尾及最终目标误差，rad | 0.015 | 0.08726646259971647（5°） |
| --max-error | 每周期目标跟踪误差，rad | 0.05 | 0.08726646259971647（5°） |
| --max-lateness | 控制循环允许迟到，s | 0.1 | 0.2 |
| --hold-after | 动作通过后持续发送末目标并读取反馈 | 关闭 | 四轴任务开启 |

默认rated策略覆盖逐轴settings中的速度与力矩字段，其他字段继续从settings加载；custom策略使用settings原值。例如J4 DM4340现行反馈力矩阈值9Nm，历史任务为2Nm，峰值27Nm、协议编码范围±28Nm均不是本次采用值。反馈阈值用于检测后停止，不是写入固件的硬力矩限幅。官方来源为[OpenArm V1电机规格](https://docs.openarm.dev/1.0/hardware/specifications/motor/)。

使用--hold-after必须提供--cancel-file。所有目标检查通过后写入HOLDING_TARGET，继续以既定周期发送末位置、零目标速度并检查误差与反馈；进入保持后运动请求的expires_at不再终止保持。创建该次取消文件或发生故障会退出并失能。后台运行时记录PID和取消文件，不能仅因父启动命令退出就认定机器人已停止。

最近两轮四轴实机均在J4阶段FAULT/STOPPED，未进入HOLDING_TARGET；该功能目前仅通过离线测试。下次现场应核对真实保持样本、四轴state=1和持续时间后再验收。最新四轴证据目录为logs/openarm/sync/left_four_retry2nm_20261116/，旧请求不能直接重复执行。

## 证据及检查

- 断连恢复日志和摘要位于 `logs/openarm/sync/reconnect_onsite_20261115_02*`。
- 实机保持、首轮跟踪未达标及调整后的往返证据位于 `logs/openarm/sync/execution_onsite_20261115/`。
- 最终链路摘要为 `mujoco_hardware_summary.json`，包含源请求与预演日志SHA256。
- 原始增益10首轮仅回程误差达标，去程未达标。该次日志中的COMPLETED已在统一计划纠正，执行器现已补齐逐途经点检查。

```powershell
& 'openarm/.venv/Scripts/python.exe' -m pytest apps/test_openarm_driver.py apps/test_openarm_sync.py apps/test_openarm_execution.py apps/test_openarm_execution_hardware.py -q
```

2026-11-16本轮结果为87 passed in 5.39s，包含逐轴额定边界、旧设置覆盖、持续保持、取消与故障失能。驱动使用替身，未连接实机。证据位于 `logs/openarm/rated_limits_20261116/`，阶段验收与下一步维护在 `docs/project/仿真与实机联通_两日实施计划.md`。

## 可选映射拟合

`apps/calibrate_openarm_mapping.py` 可处理独立测量的电机/模型角对，输出方向、比例、参考角及残差。当前已采用用户确认的零位与1∶1比例，拟合工具供后续需要量化测量时使用。输入要求rad、measurement_method、fit_pairs、validation_pairs、min_span_rad和max_error_rad，至少三个拟合姿态与两个独立验证姿态。计算结果的hardware_verified保持false，需结合实际测量依据审核。
