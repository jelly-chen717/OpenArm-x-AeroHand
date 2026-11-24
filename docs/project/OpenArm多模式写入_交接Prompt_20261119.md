# OpenArm 14 轴控制模式写入 · 交接 Prompt（2026-11-19）

> 用法：把本文件全文作为一个新会话的首条任务。首条回复先给出“现场复核 + 写入清单 + 回退值”，得到用户当次确认后再写寄存器。不要直接开始写。

## 0. 你的任务

在用户明确授权的现场窗口内，为 OpenArm V1 双臂 J1～J7（14 轴）写入并读回目标 `CTRL_MODE`，优先完成 POS_VEL（模式 2）；如用户另行指定，再处理 VEL（3）或 POS_FORCE（4）。本任务只做“模式寄存器写入 + 独立读回 + 失败回退”，**不使能、不发送运动目标、不做整曲或带载运动**。使能、保持和小幅往返必须另列授权。

最终交付：14 轴逐轴 `before_mode → after_mode`、固件版本、写入/读回原始 CAN 记录、失败/回退清单、可复用的现场 `--drive-profile`，以及明确的“已验证/未验证”边界。

## 1. 先读这些文件，再动硬件

1. `AGENTS.md`、`openarm/AGENTS.md`（如触碰 `openarm/`）。
2. `docs/project/仿真与实机联通_两日实施计划.md` 的“快速接手”“阶段清单”“执行记录”，重点 C02～C05、WEB04/WEB05。
3. `docs/project/OpenArm14关节位置模式切换方案_20261117.md` 全文，尤其第 1、2、4、5 节。
4. `monitor-ui/README.md` 的“实机多模式的接入条件”。
5. `docs/project/OpenArm控制台防护项清单_20261119.md`，确认哪些运动阈值已被删除、哪些检查仍保留。
6. `git status`、相关 diff。工作区有未提交/未跟踪文件，**禁止 reset、clean、checkout 或整文件覆盖**。

## 2. 当前现场状态（2026-11-19 17:00 左右，只读）

- 8092 `hardware` 服务曾启动成功；页面应显示“实机 · 2/2 臂在线”。
- `/api/status`：`mode=hardware`，`drive_mode=MIT`，`drive_modes=["MIT"]`，`can_switch_mode=false`，`can_prepare=true`，`can_enable_all=true`。
- 最近一次任务：`CANCELLED`，`stop_reason=操控页面连接超时`；当前无活动任务。
- 左右 14 轴：反馈新鲜，全部 `state=0`（失能），静止；温度约 23～28 °C，力矩接近 0。
- 左 J1～J7 约：`1.10, -1.56, -12.03, 0.49, 6.76, 2.74, -0.21` °。
- 右 J1～J7 约：`2.52, -5.34, -90.72, 5.58, 2.44, -4.38, -5.41` °。
- 只读证据：`logs/openarm/monitor/hardware_readonly_20261119.json`。
- 当前硬件服务日志：`logs/openarm/monitor/hardware-0ca6e19b37304b8ab52390b4ecc7ccf6.jsonl`。

**为什么现在切不了**：服务没有 `--drive-profile`，所以后端只返回 `MIT`，`can_switch_mode=false`。不是页面 bug，也不是 WEB05 删防护漏掉的开关。必须先生成现场 profile。

当前可能有重复的 8092/8091 Python 进程（`.venv` 与 `.python311` 各一份）。C02 前先确认 8092 只有一个拥有者，关闭所有 `openarm_monitor.py --mode hardware` 进程；否则 CAN 通道租约会冲突。

## 3. 不可跳过的边界

- 只有用户当次明确说“写入/执行”才写 `CTRL_MODE`；只读盘点、写模式、使能、运动是四个不同授权。
- 机械臂必须可靠支撑重力负载，14 轴全部失能，线材、急停、运动空间已检查；写入期间不得使能或发运动目标。
- 左通道 `0x51`、右通道 `0x52`，必须显式传入；`openarm_driver.DEFAULT_CHANNELS` 的默认左右是反的，不能依赖默认值。
- 每个轴的基础 ID 1～7；左右复用同一组 ID，靠两条独立总线区分。
- `CTRL_MODE` 寄存器：RID=10（0x0A），uint32 小端；参数通道 0x7FF；写 0x55，读 0x33。
- **写入 ACK 不等于切换成功**，必须用独立的 0x33 读回模式值。
- 特殊帧地址（使能/失能帧）必须按每轴固件区分 `base` 或 `mode`；不能用某一型号手册替全部 14 轴填写。`special_frame_address` 只影响特殊帧，不影响 CTRL_MODE 的 0x7FF 参数写。
- 默认不调用 Flash 保存；是否易失、是否掉电恢复必须单独核验。首次试验不写 Flash。
- 写入失败或读回不一致：停止后续轴，按原值回退已尝试轴；回退失败标记 `STOP_UNCONFIRMED`/`ROLLBACK_FAILED`，依靠现场急停或断电，不声称安全。
- 本任务不使能、不运动；如要验证使能/当前位保持，必须再次获得用户授权，另立步骤。
- 不得以“模式切换更安全”为由重新引入 WEB05 已删除的跟踪误差、速度/加速度、温度、力矩、行程、反馈年龄、起姿漂移、静止使能前提等阻断；工作区第一原则见根目录 `AGENTS.md`。

## 4. 模式与帧格式（当前代码）

| 模式 | CTRL_MODE | 目标帧 ID | 负载 |
| --- | --- | --- | --- |
| MIT | 1 | 基础 ID（`motor_id`） | MIT 8 字节：位置/速度/kp/kd/力矩 |
| POS_VEL | 2 | `motor_id + 0x100` | `<ff`：位置 rad、正速度上限 rad/s |
| VEL | 3 | `motor_id + 0x200` | `<f`：速度 rad/s；主机外环 `feedforward + 5.0*position_error` 后限幅 |
| POS_FORCE | 4 | `motor_id + 0x300` | `<fHH`：位置 rad、速度上限×100、电流标幺×10000 |

参考实现：

- `apps/openarm_monitor_modes.py`：`DRIVE_MODES`、`MODE_CODES`、`position_frame()`、`velocity_frame()`、`DriveProfile`。
- `apps/openarm_position_mode.py`：`REGISTERS`、`encode_target()`、`read_inventory()`。
- `apps/openarm_driver.py`：`write_disabled_mode()`、`read_parameter()`、`enable_motor()`、`disable_motor()`。
- `apps/openarm_execution.py`：`PcanTransport`、`MonitorTransport`。

**注意**：WEB05 之后，`OpenArmDriver.enable_motor()` 不再强制 CTRL_MODE=1，`command_mit()` 不再强制模式 1；但 `MonitorTransport` 在非 MIT 模式下仍会走 `position_frame()` / 特殊帧分支。不要因为驱动不再拦截就跳过模式读回。

## 5. 执行顺序

### Phase 0 · 接管与只读复核

1. 确认 8092 只有一个监听者；关闭重复的 hardware 服务。
2. 确认 8091/8092 不混淆；C02 不使用网页服务占用的 CAN。
3. 只读查询 14 轴状态，保存到 `logs/openarm/mode_switch/<timestamp>/before_status.json`：
   - 在线、`state=0`、速度、温度、反馈年龄、总线状态。
4. 确认左右通道、物理侧、14 轴 ID 与铭牌/接线一致。

### Phase 1 · C02 14 轴只读盘点

停止 8092（Ctrl+C 或关闭服务窗口），确认 8092 不再监听、CAN 租约释放。然后在工作区根目录执行：

```powershell
& '.\openarm\.venv\Scripts\python.exe' -B apps\openarm_position_mode.py --inventory 'logs/openarm/mode_switch/<timestamp>/inventory.json'
```

要求：

- `kind=position_mode_read_only_inventory`
- `complete=true`
- `mapping_version=hardware-unified-forward-inward-v2-20261116`
- `before`/`after` 两臂都 `bus_status=0x0`、7/7 在线、全部 `state=0`
- `axes` 恰好 14 条，`read_errors` 为空
- 逐轴记录 `CTRL_MODE`、`hw_ver`、`sw_ver`、`sub_ver`、`PMAX`、`VMAX`、`TMAX`、`OC_Value`、`ACC`、`DEC`、`MAX_SPD`、`TIMEOUT`、环路参数

如果 `complete=false`：不要写模式；先解决缺失轴/读错误/失能回读。

### Phase 2 · 逐轴固件与特殊帧核实

从 inventory 生成一张 14 轴表，列出：型号、`hw_ver`、`sw_ver`、`sub_ver`、当前 `CTRL_MODE`、`PMAX/VMAX/TMAX`、特殊帧应使用 `base` 还是 `mode`、依据（手册页/回读/实测）。

已知历史：

- 左/右 J2 `sw_ver=925971510`；左/右 J6 `sw_ver=925970485`。
- 其余十轴的历史固件回读缺失，必须本次补读。
- 达妙 4310P-2EC V1.3 手册/CAN FD SDK 使用模式偏移特殊帧；OpenArm 参考实现使用基础 ID。两种说法不能混用，逐轴确认。

**不要**为了让 profile 通过检查而把 `special_frame_address` 全部填 `base` 或全部填 `mode`。没有依据的轴保持 `null`/不生成 profile。

### Phase 3 · 生成现场 profile

在 `logs/openarm/mode_switch/<timestamp>/profile.json`（或 `configs/` 下专用文件）写入：

```json
{
  "inventory": "inventory.json",
  "special_frame_address": {
    "left":  ["base", "base", "base", "base", "base", "base", "base"],
    "right": ["base", "base", "base", "base", "base", "base", "base"]
  },
  "disabled_preload_verified": true,
  "volatile_mode_write_verified": true,
  "pos_force_current_fraction": 0.1
}
```

规则：

- 上面的 `base` 和 `true` 是占位示例，**不是已核实值**；必须按 Phase 2 结果填写。
- `special_frame_address` 每侧 7 项，只能是 `base` 或 `mode`。
- `disabled_preload_verified` / `volatile_mode_write_verified` 未实际验证时不得填 `true`；否则 `DriveProfile` 会放行未确认的写入。
- `pos_force_current_fraction` 必须是为双臂核实的最大相电流比例（0.0001～1）；不切 POS_FORCE 也要填一个已核实值，因为 `DriveProfile.__init__` 会校验。
- `inventory` 可相对 profile 路径。

如果两项语义尚未验证，先做 Phase 3b，不要直接填 true。

### Phase 3b · 单轴受控验证（推荐，先 J7）

选一侧 J7（4310，受支撑、空间充足），另一侧及其他轴保持失能、支撑。用户当次确认后，用受控脚本/驱动：

1. 只读 J7 当前模式和原始位置，保存备份。
2. 确认 `state=0`、`bus_status=0x0`、反馈新鲜。
3. 向 RID=10 写目标模式（先 2），独立读回。
4. 记录写入前后原始回复；不使能、不发运动目标。
5. 如需验证掉电易失性，断电重启后只读该轴模式；不在上电自动使能。
6. 若失败，恢复原模式并读回。

单轴通过后，才把该轴的 `special_frame_address` 和两项 verified 依据写入 profile。不要一开始就对 14 轴批量写。

### Phase 4 · 用服务做 14 轴事务切换

确认 8092 已停止、profile 有效后：

```powershell
& '.\openarm\.venv\Scripts\python.exe' -B apps\openarm_monitor.py --mode hardware --drive-profile 'logs/openarm/mode_switch/<timestamp>/profile.json'
```

或通过原启动脚本：

```powershell
.\monitor-ui\启动监视控制台.cmd --mode hardware --drive-profile 'logs/openarm/mode_switch/<timestamp>/profile.json'
```

服务启动后先只读 `/api/status`，期望：

- `drive_modes` 四项齐全
- `can_switch_mode=true`（14 轴静止失能、profile 有效）
- `drive_mode=MIT`
- `can_prepare=true`

然后在页面顶部选择目标模式，或在用户确认后调用 `POST /api/drive-mode`。`DriveProfile.switch()` 会：

- 再次 `require_stationary()`：14 轴失能、速度≤0.1 rad/s、反馈年龄≤0.15 s、总线正常；
- 读取硬件/软件/子版本并与 inventory 比对；
- 对需要变化的轴写 CTRL_MODE；
- 独立读回全部 14 轴模式；
- 再查一次静止失能；
- 部分失败时按原值回退已尝试轴。

全程记录 `drive.mode_axis`、`drive.mode`、`drive.mode_failed`、原始 CAN 回复。切换成功后**仍保持失能**；不要自动使能。

### Phase 5 · 切换后只读验收

对每个目标模式：

1. `/api/status` 的 `drive_mode` 与 14 轴读回一致。
2. 重新断电/重启后再只读一次，确认是否符合“易失/持久”结论。
3. 保存 `mode_before.json`、`mode_transactions.jsonl`、`raw_can.jsonl`、`summary.json`。
4. 如果用户要求回到 MIT，逐轴写回原值 1 并读回；不自动使能。
5. 使能、保持、小幅往返、带载运动另列授权和验收，不在本任务内宣称通过。

## 6. 证据与验收口径

证据目录：`logs/openarm/mode_switch/<timestamp>/`

至少包含：

- `before_status.json`
- `inventory.json`（14 轴完整）
- `profile.json`
- `mode_before.json`
- `mode_transactions.jsonl`
- `raw_can.jsonl`（写入/读回原始帧）
- `mode_after.json`
- `rollback.json`（如有）
- `summary.json`（逐轴 before/after、失败、回退、固件、特殊帧依据）

验收分开写：

- 软件/离线通过
- C02 只读盘点通过
- 逐轴模式写入+读回通过
- 14 轴模式读回通过
- 使能/保持通过（另列）
- 运动/带载/末端精度通过（另列）

**不要把“写入成功”写成“实机运动验收通过”。**

## 7. 停止与回退

- 任一步异常：停止后续写入，保持/恢复支撑，按原模式失能。
- 通信正常时逐轴读回失能；失败标记 `STOP_UNCONFIRMED`，依靠现场急停/断电。
- 回退只恢复原模式（原值 1 才回 MIT），不自动使能、不重放旧轨迹。
- 不主动 Flash 保存；长期采用某模式后再单独安排保存和断电核验。
- 写入期间不得同时运行其他 PCAN 程序；8092 服务必须独占两路 CAN。

## 8. 首条回复格式（写寄存器前）

新接手会话的第一条回复请只做这三件事：

1. 复述你读到的现场状态（14 轴失能/静止、当前模式、固件/inventory 是否完整）。
2. 给出本次准备写入的模式、轴列表、每轴原模式、profile 是否满足条件、特殊帧依据。
3. 给出停止/回退方案和预计证据路径，并请用户明确回复“执行写入”。

得到用户确认后再进入 Phase 4/3b。未确认前只读。

## 9. 参考命令与测试

离线回归（改动代码后必须重跑）：

```powershell
& '.\openarm\.venv\Scripts\python.exe' -B -m pytest -q -p no:cacheprovider apps/test_openarm_monitor.py apps/test_openarm_driver.py apps/test_openarm_execution.py apps/test_openarm_execution_hardware.py apps/test_openarm_position_mode.py apps/test_openarm_dual_sequence.py apps/test_prepare_openarm_dual_sequence.py apps/test_openarm_sequence_dynamics.py
```

2026-11-19 基线：168 项通过。模式切换相关覆盖在 `apps/test_openarm_monitor.py` 的 `ProfileDriverFixture`、`test_commissioned_modes_hold_and_stop`、`test_mode_failure_blocks_enable_and_restores_partial_writes`，以及 `apps/test_openarm_position_mode.py`；这些是替身 CAN，不代表实机已通过。

当前现场只读证据：`logs/openarm/monitor/hardware_readonly_20261119.json`。当前硬件服务日志：`logs/openarm/monitor/hardware-0ca6e19b37304b8ab52390b4ecc7ccf6.jsonl`。
