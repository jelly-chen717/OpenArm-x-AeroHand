# OpenArm 防护删除合规审计 · 定期交接 Prompt（2026-11-21）

> 用法：把本文件全文作为一个新会话的首条任务。**这是只读审计，不是开发任务**。审计员不得修改控制代码、测试、配置或文档，不得启动 8092、不得连接实机、不得读写 CAN、不得使能或运动；只输出审计结果和证据。发现违规先报告，等用户明确授权后再修。

## 0. 角色与目标

你是本工作区的**防护删除合规审计员**。工作区：`D:\桌面\open CyMon\弹钢琴`。目标：检查用户已明确删除的非必要运动阈值防护是否被重新写入，并检查“控制链路默认放行”的第一原则是否被删除、弱化或绕过。

这不是“找 bug”或“提高安全性”的任务。**默认放行是用户明确要求的第一原则**；任何新增的拒绝执行、FAULT、退出控制、自动失能、取消保持的检查，都先按违规候选处理，再按 K01～K12 例外清单分类。

## 1. 先读（按顺序）

1. 根目录 `AGENTS.md`，特别是“第一原则（硬性，优先级最高）”。如果该节缺失、被改写、被加上未授权例外，直接判 **CRITICAL FAIL**，停止后续并报告。
2. `docs/project/OpenArm控制台防护项清单_20261119.md`：P01～P20 为永久禁止恢复项，K01～K12 为仅允许保留项。
3. `monitor-ui/README.md` 的“实验室模式：已删除的非必要防护”。
4. `docs/project/仿真与实机联通_两日实施计划.md` 的“快速接手”“阶段清单”“执行记录”，重点 WEB05、WEB06、C02～C05。
5. `docs/project/OpenArm多模式写入_交接Prompt_20261119.md`（如果本次审计涉及模式写入）。
6. `git status`、`git log -5`、`git diff --stat`、`git diff`。工作区常有大量未提交/未跟踪文件；**禁止 reset、clean、checkout 或整文件覆盖**。

## 2. 审计范围

必查：

- `apps/openarm_monitor.py`
- `apps/openarm_monitor_motion.py`
- `apps/openarm_monitor_modes.py`
- `apps/openarm_execution.py`
- `apps/openarm_driver.py`
- `apps/drive_openarm_from_sim.py`
- `apps/openarm_trajectory.py`（只确认它不在网页路径）
- `apps/openarm_position_mode.py`
- `monitor-ui/app.js`、`monitor-ui/index.html`、`monitor-ui/style.css`
- `apps/test_openarm_monitor.py`、`apps/test_openarm_driver.py`、`apps/test_openarm_execution.py`、`apps/test_openarm_execution_hardware.py`、`apps/test_openarm_position_mode.py`、`apps/test_openarm_dual_sequence.py`、`apps/test_prepare_openarm_dual_sequence.py`、`apps/test_openarm_sequence_dynamics.py`
- `AGENTS.md`、`monitor-ui/README.md`、防护清单、所有交接 Prompt

不主动改 `openarm/` 纯仿真；若其改动影响控制链路入口，记录为“范围外但相关”。

## 3. 当前基线（2026-11-21 静态抽查，供参考，不是完整结论）

- `AGENTS.md` 第一原则仍在。
- 网页路径 `apps/openarm_monitor.py` 的 `_move()` 仍传 `max_error=None`，未恢复跟踪误差阻断。
- 未发现 `PREVIEW_TTL`、`expires_in_s`、`expiresAt`、`60 秒内有效` 的网页预览有效期。
- `apps/openarm_execution.py` 的 `PcanTransport.check()` 仍是 notes + 返回 `False`，未恢复 `raise`。
- `apps/openarm_driver.py` 的 `encode_mit()` 仍是饱和编码；`enable_motor()` / `command_mit()` 没有恢复模式/新鲜度前置阻断。
- 工作区新增了模式写入相关改动：`ModeRegisters`、`DriveProfile`、`--mode-inventory`、`mode_error`、`expected` 读回一致性、`configs/openarm_position_mode.json` 的 `monitor_inventory`。这些属于 K07/K08 模式切换/配置准入，**只有扩展到运动/使能路径时才算违规**。
- 2026-11-19 基线为 168 项相关测试通过；2026-11-21 可能已有新增/修改测试，数量变化本身不是违规，**测试内容**才是。

每次审计必须重新核实，不要直接引用本节的“未发现”。

## 4. P01～P20 逐项检查表

对每一项给出 `PASS`（已删除/仍放行）、`FAIL`（重新写入阻断）、`UNKNOWN`（新代码/无法判定），并附文件:行号证据。

| 项 | 期望现状 | 重点检查 |
| --- | --- | --- |
| P01 | 规划速度/加速度按目标所需放行 | `MotionPlanner._validate` 不得用 `min` 把速度压回百分比、不得因 1 rad/s² 抛错；应是 `max(plan.velocity, ...)` / `max(MAX_ACCELERATION, ...)` |
| P02 | 不做模型接触扫描 | `prepare`/`_validate` 不得调用接触/碰撞拒绝 |
| P03 | 预览后起姿漂移采纳 | `revalidate` 把 `start` 设为当前实测；不得抛“起始姿态已变化” |
| P04 | 跟踪误差只记录 | `execute` 中 `max_error` 只产生 advisory；网页 `_move` 传 `max_error=None` |
| P05 | 路径点/终点偏差只记录 | `endpoint_tolerance` 不得触发 FAULT；`HOLDING_TARGET` 照常进入 |
| P06 | 周期迟到只记录 | `max_lateness` 只 advisory |
| P07 | 运行中过期只记录 | 运行中 `expired` 只 advisory；仅“使能前已过期”仍允许拒绝 |
| P08 | 反馈阈值只 notes | `PcanTransport.check` 不得 `raise`；非有限/age/state/temp/speed/torque/travel/limit 均 notes |
| P09 | 使能/指令不预检行程 | `PcanTransport.enable` 无 `check_start`；`command` 不做发送前整批行程拒绝 |
| P10 | 驱动使能不预检模式/失能、不自动失能 | `enable_motor` 不得读 CTRL_MODE 拒绝、不得要求 DISABLED、不得在超时后自动 disable |
| P11 | MIT 指令不预检新鲜/使能 | `command_mit` 不得因无新鲜/未使能反馈拒绝、不得自动失能 |
| P12 | MIT 超量程饱和 | `encode_mit` 不得因位置/速度/kp/kd/力矩越界抛 `ValueError` |
| P13 | 一键使能不要求静止失能 | `Monitor._idle` 不得要求 `_stable`；`enable_all` 只要求反馈齐全/空闲/模式可用 |
| P14 | 跟踪/终点不触发故障 | `_move` 的 `max_error=None`，终点容差只日志 |
| P15 | 执行前不要求静止/新鲜门槛 | `MonitorTransport._sample` 不得因静止/age 拒绝 |
| P16 | 非指令轴异常只记录 | `MonitorTransport.command` 用 `_note`，不中断 |
| P17 | 阶段超时/保持误差不退出 | `execute_sequence` 记录 `stage_timeout`/`hold_tolerance_exceeded` 后继续，不抛错 |
| P18 | 常量不作拒绝阈值 | `START_ERROR`/`TRAVEL_MARGIN`/`MAX_ACCELERATION` 只用于余量/记录 |
| P19 | 无预览有效期 | 不得有 `PREVIEW_TTL`/`deadline`/`expires_in_s`/`expiresAt`/“60 秒内有效” |
| P20 | 非 MIT 模式无需现场协议配置即可使能和运动 | `_motion_supported` 不得要求 `drive_profile`；`MonitorTransport.__init__` 不得因 `mode != "MIT"` 且无 profile 抛错；特殊帧默认基础 ID |

## 5. K01～K12 例外与新增阻断扫描

以下保留项**不算违规**：关节位置限位、NaN/映射/协议帧、CAN 租约/总线/超时、反馈新鲜度 UI、停止失能回读与“停止未确认”、模式切换静止失能/固件模式回读/回退、C02 盘点/现场协议配置准入、本机 Host/Origin/令牌、操控页心跳停止、独立 CLI `openarm_trajectory.preflight`（确认不在网页路径）。

对 `apps/` 和 `monitor-ui/` 全量搜索：

```powershell
git diff -- apps monitor-ui configs
rg -n "raise (DriverError|ValueError|RuntimeError|Conflict)|FAULT|STOP_UNCONFIRMED|disable|reject|expired|deadline|PREVIEW_TTL|expires" apps monitor-ui
rg -n "max_error|max_lateness|endpoint_tolerance|max_temperature|max_torque|max_travel|max_age|max_speed" apps monitor-ui
```

对每个候选回答：

1. 它会拒绝执行/FAULT/退出/自动失能/取消保持吗？
2. 它属于 P01～P20，还是 K01～K12？
3. 如果是新增且不在 P/K，标 `UNKNOWN`，要求用户裁决；不要自行删除或保留。
4. 如果它是 K07（模式切换前提）但被扩展到运动/使能路径，标 `FAIL`。

重点检查新增模式代码：

- `ModeRegisters` / `DriveProfile` / `--mode-inventory` / `mode_error` / `expected`：
  - 只允许在“模式切换/模式读回/C02 配置准入”路径拒绝；
  - 如果 `mode_error`、`axis_modes`、`_motion_supported` 导致“反馈正常、模式已一致、但一键使能/预览被禁用”，标 `FAIL` 或 `UNKNOWN` 并给出字段。
  - 非 MIT 模式（POS_VEL/VEL/POS_FORCE）在只有盘点、没有 `--drive-profile` 时，`can_enable_all` 必须为 true 且能执行；若被 profile 或“运动协议配置”重新拦住，标 `FAIL`（P20）。
- 8092 一键使能：`/api/status` 的 `can_enable_all` 应在 14 轴反馈齐全、`drive_mode` 可确定时可用；如果 14 轴反馈正常但 `can_enable_all=false`，检查 `_motion_supported`、`mode_error`、`axis_modes`、`drive_mode`，不要直接改代码。
- `openarm_trajectory.preflight` 只能在 CLI/其他调用方；`openarm_monitor_motion` 只导入 `Trajectory`，不得导入/调用 `preflight`。

## 6. 测试审计

运行（从根目录）：

```powershell
& '.\openarm\.venv\Scripts\python.exe' -B -m pytest -q -p no:cacheprovider apps/test_openarm_monitor.py apps/test_openarm_driver.py apps/test_openarm_execution.py apps/test_openarm_execution_hardware.py apps/test_openarm_position_mode.py apps/test_openarm_dual_sequence.py apps/test_prepare_openarm_dual_sequence.py apps/test_openarm_sequence_dynamics.py
```

记录：通过/失败/跳过数量、耗时、失败测试名。然后检查测试内容：

- 是否恢复了对跟踪误差、路径点/终点、迟到、温度、速度/力矩/行程、阶段超时/保持误差的 `pytest.raises`/FAULT 断言？
- 是否删除了“放行/记录/advisory/HOLDING_TARGET”测试？
- 是否把 `PcanTransport.check` 的测试改回 `pytest.raises`？
- 是否把一键使能测试改回“必须静止失能”？
- 是否新增了与 AGENTS 第一原则冲突的测试？

测试数量变化本身不是违规；**测试要求的语义**才是。

## 7. 可选：8091 虚拟 UI/API 冒烟（不碰实机）

如果当前没有 8091 服务，可用临时端口和临时日志目录：

```powershell
& '.\openarm\.venv\Scripts\python.exe' -B apps/openarm_monitor.py --mode virtual --port 18091 --log-directory "$env:TEMP\openarm-audit-<timestamp>"
```

只检查：

- `/api/status`：`mode=virtual`、`can_enable_all=true`；
- 页面或 API 的“全部使能”可用，执行后 14/14；
- 0.1 s / 45° 目标能预览（无速度/加速度拒绝）；
- 到位后 `HOLDING_TARGET`，停止后 0/14；
- 页面没有“预览已过期/60 秒”文案。

**不要**启动 8092、不要连接 CAN、不要操作实机。

## 8. 输出格式

先给总判定：`PASS` / `FAIL` / `UNKNOWN`。

然后：

1. **第一原则**：完整/被弱化/缺失（CRITICAL）。
2. **P01～P20 表**：逐项 PASS/FAIL/UNKNOWN + 文件:行号 + 一句话证据。
3. **新增阻断候选**：文件:行号、行为、分类（K07/K08/UNKNOWN/Pxx）、是否需要用户裁决。
4. **测试**：命令、通过/失败数量、耗时、失败名。
5. **文档/交接 Prompt**：是否出现恢复防护的表述。
6. **结论与建议**：若 FAIL，给出最小修复建议（只描述，不直接改）；若 UNKNOWN，列出需要用户确认的问题。
7. **审计记录**：写入 `logs/openarm/audit/protection_audit_<YYYYMMDD_HHMM>.json`，字段至少：
   - `time`、`git_head`、`workspace_dirty`、`overall`
   - `first_principle_intact`
   - `p_items`（P01～P20 → PASS/FAIL/UNKNOWN）
   - `new_blocking_checks`
   - `tests`（passed/failed/skipped/command）
   - `violations`（file/line/behavior/class/evidence）
   - `notes`
8. 如 `docs/project/OpenArm防护删除_审计记录.md` 不存在，创建它并追加一行摘要（时间、HEAD、结论、PASS/FAIL、证据文件）；如存在，只追加，不改历史行。

## 9. 审计员禁止事项

- 不修改任何代码、测试、配置、AGENTS、README 或交接 Prompt。
- 不 reset/clean/checkout，不覆盖未提交改动。
- 不启动 8092，不连实机，不读写 CAN，不使能，不运动，不写模式。
- 不把“安全/防御性编程/最佳实践”作为恢复防护的理由。
- 不把 K01～K12 误判为 P 项；也不把 P01～P20 以“通信/限位”名义放行。
- 发现违规只报告，等用户明确授权后再修。

## 10. 快速结论模板

```text
审计时间：YYYY-MM-DD HH:MM
工作区 HEAD：<sha>（dirty: yes/no）
第一原则：完整 / 弱化 / 缺失
总判定：PASS / FAIL / UNKNOWN

P01～P20：PASS x / FAIL y / UNKNOWN z
- FAIL：Pxx <file:line> <行为> <证据>
- UNKNOWN：<file:line> <新检查> <为什么无法归类>

测试：<passed> passed / <failed> failed / <skipped> skipped，<耗时>
新增阻断候选：<列表>
文档：<是否出现恢复防护表述>
建议：<最小修复建议；等待用户授权>
证据：logs/openarm/audit/protection_audit_<...>.json
```
