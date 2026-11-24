# OpenArm 防护删除合规审计 · 交接 Prompt（2026-11-23 21:30 版）

> 用法：把本段全文作为新会话的第一条任务。
> 这是只读审计，不是开发任务。审计员不得修改任何代码、测试、配置、文档或交接 Prompt；
> 不得启动 8092、不得连接实机、不得读写 CAN、不得使能、运动或写模式；
> 发现违规只报告，等用户明确授权后再修。

## 0. 这次交接的重点

P01～P24 已全部清除，表只是回归清单。本次首要任务仍是**新阻断全量盘点**：
从行为出发，找出所有会拒绝执行、触发 FAULT、退出控制、自动失能、取消保持的检查，
再判断它属于 P、K，还是新的 UNKNOWN。找不到新阻断时，也要在 `new_blocking_checks`
写明扫描范围与“未发现”。

2026-11-23 21:28 刚完成的清理（只查回退，不要重复改）：
1. **P21** 起始位/0位/回0/钢琴原位不再要求先“全部使能”，计划自行使能 14 轴；
2. **P22** 试弹时手部未连接/会话失效/未标定只跳过指法并记 `hand.score.skipped`，机械臂照跑；
3. **P23** 运动起点模式/固件读回不一致或失败只记 `mode_readback`，不拒绝运动；
4. **P24** 接续执行反馈不完整改用缓存反馈 + 计划起点，记 `motion.continuation_adopted`。

## 1. 先读（按顺序）

1. 根目录 `AGENTS.md`，特别是“第一原则（硬性，优先级最高）”。该节缺失、被改写、
   被加入未授权例外，直接判 CRITICAL FAIL，停止后续并报告。
2. `docs/project/OpenArm控制台防护项清单_20261119.md`：
   P01～P24 为永久禁止恢复项，K01～K12 为仅允许保留项。
3. `docs/project/OpenArm防护删除_审计记录.md`（只追加，不改历史行），以及最近证据：
   - `logs/openarm/audit/protection_audit_20261123_2116.json`（全量盘点 + 199 passed 基线）
   - `logs/openarm/audit/protection_fix_20261123_2128.json`（P21～P24 清除）
4. `monitor-ui/README.md`：“实验室模式：已删除的非必要防护”“双臂快捷定位”
   “《槿》和弦试弹与分指预览”“《槿》双臂空弹”“Aero 手部控制”各节。
5. `docs/project/仿真与实机联通_两日实施计划.md` 的“快速接手”“阶段清单”“执行记录”，
   重点 WEB05～WEB08、C02～C05。
6. `docs/project/OpenArm多模式写入_交接Prompt_20261119.md`（如涉及模式写入）。
7. `git status`、`git log -5`、`git diff --stat`、`git diff`。
   工作区大量未提交/未跟踪文件，且可能被并行修改；禁止 reset、clean、checkout 或整文件覆盖。
   审计期间文件变化时以最新内容为准，并在报告中记录变化时间与影响。

## 2. 审计范围

核心控制链：
- `apps/openarm_monitor.py`
- `apps/openarm_monitor_motion.py`
- `apps/openarm_monitor_modes.py`
- `apps/openarm_execution.py`
- `apps/openarm_driver.py`
- `apps/drive_openarm_from_sim.py`
- `apps/openarm_trajectory.py`（只确认不在网页路径）
- `apps/openarm_position_mode.py`
- `apps/openarm_joint_mapping.py`
- `monitor-ui/app.js`、`index.html`、`style.css`

新增/近期修改路径（必须纳入）：
- `apps/openarm_start_pose.py`（起始位/0位/回0）
- `apps/openarm_piano_air.py`、`openarm_piano_trial.py`、`openarm_piano_origin.py`
- `apps/openarm_monitor_hands.py`（Aero 手部与指法调度）
- `apps/build_openarm_piano_points.py`（构建期脚本，确认不被运行时导入）
- monitor-ui 的起始位、钢琴、手部按钮与状态显示

测试必查（10 文件）：`test_openarm_monitor.py`、`test_openarm_monitor_hands.py`、
`test_openarm_piano_points.py`、`test_openarm_driver.py`、`test_openarm_execution.py`、
`test_openarm_execution_hardware.py`、`test_openarm_position_mode.py`、
`test_openarm_dual_sequence.py`、`test_prepare_openarm_dual_sequence.py`、
`test_openarm_sequence_dynamics.py`。

`openarm/` 纯仿真入口不在范围；若手部调用的 `openarm/tools/aero_*` 改动影响机械臂入口，
记“范围外但相关”。不得借纯仿真入口恢复控制链路防护。

## 3. 当前基线（2026-11-23 21:30 快照，必须重新核实）

- HEAD：`9e0424e`（所有防护删除与修复仍未提交）。
- dirty：51 项修改 / 81 项未跟踪 / 1 删除（含审计证据与工作区记忆）。
- 最近审计：2026-11-23 21:16，P01～P20 PASS / 0 FAIL，总判定 UNKNOWN（当时 4 项待裁决）；
  同日晚 21:28 用户“四项全部扫清”授权后，4 项改为 P21～P24 并已实现。
- 最近一次测试（10 文件）：**199 passed / 1 failed / 109.51 s**。
  唯一失败 `test_openarm_piano_points.py::test_all_physical_white_keys_match_their_fixed_wrist_pose`
  与本轮代码改动无关：`configs/openarm_piano_white_keys.json` 于 21:18:15 被外部改为
  “右食指原位 C4（physical 15）”，点位 CSV 仍是 20:51 的 B4（physical 21）版本，右侧 Y 差 135 mm。
  同日晚 21:16 的运行中该用例通过，可用于确认差异来自 21:18 的外部数据修改。
- 已授权修复（只查是否回退）：U1 模式故障解除、P12-adj 饱和、P20 非 MIT 无 profile 放行、
  deadline 用例 LateWait 稳定化、P21～P24。

## 4. 新阻断全量盘点（主任务）

入口逐个走：`/api/enable-all`、`/api/prepare`、`/api/execute`、`/api/stop`、
`/api/start/zero`、`/api/start/return`、`/api/start/pose`、`/api/start/piano-origin`、
`/api/hand`、`/api/hand-score/start`、`/api/hand-score/stop`、`/api/drive-mode`。
每个入口检查：正常输入是否被新检查拦住；模式/反馈正常时 `can_prepare`、
`can_enable_all`、`can_start_pose` 是否可用。

建议命令：

```
git status --porcelain
git diff --stat -- apps monitor-ui configs
git diff -- apps monitor-ui configs
rg -n "raise (DriverError|ValueError|RuntimeError|Conflict)|FAULT|STOP_UNCONFIRMED|disable|reject|expired|deadline|PREVIEW_TTL|expires|return False" apps monitor-ui
rg -n "max_error|max_lateness|endpoint_tolerance|max_temperature|max_torque|max_travel|max_age|max_speed|speed_percent|倍率|tolerance|settle|stationary|stale|state.*== *1" apps monitor-ui
```

重点新分支：
- `_start_ready` / `_start_plan` / `start_pose_motion`（`openarm_monitor.py:265,295`）
- `prepare_piano_air` / `prepare_piano_trial` / `prepare_piano_origin` / `compile_air` / `compile_trial`
- `ConsoleHands.connect / action / start_score / schedule_score / disconnect_all`
- `MonitorTransport.start_pose`（`openarm_monitor_motion.py:439`）、特殊帧、profile 可选
- `_move.continuation`（`openarm_monitor.py:712`）
- `PcanTransport.check / enable / command / stop`

对每条候选回答：是否阻断；属 P/K/UNKNOWN；UNKNOWN 需写文件:行、触发条件、字段、影响，
请用户裁决；K07 的模式切换前提被扩展到运动/使能路径 → FAIL。

## 5. P01～P24 回归表

| 项 | 期望现状 | 重点检查 |
| --- | --- | --- |
| P01 | 速度/加速度按目标所需放行 | `_validate` 用 `max(...)`，不因 1 rad/s² 抛错 |
| P02 | 不做模型接触扫描 | `prepare`/`_validate` 无接触/碰撞拒绝 |
| P03 | 预览后起姿漂移采纳 | `revalidate` 的 start = 当前实测 |
| P04 | 跟踪误差只记录 | `execute` 只 advisory；网页 `max_error=None` |
| P05 | 路径点/终点偏差只记录 | 容差只日志，仍进入 `HOLDING_TARGET` |
| P06 | 周期迟到只记录 | `max_lateness` 只 advisory |
| P07 | 运行中过期只记录 | 仅“使能前已过期”可拒绝 |
| P08 | 反馈阈值只 notes | `PcanTransport.check` 无 raise |
| P09 | 使能/指令不预检行程 | `enable` 无 `check_start`；`command` 不拒绝整批 |
| P10 | 驱动使能不预检模式/失能 | `enable_motor` 不读模式、不要求 DISABLED、不自动失能 |
| P11 | MIT 指令不预检新鲜/使能 | `command_mit` 直接发送 |
| P12 | MIT 超量程饱和 | `encode_mit` 不抛 ValueError |
| P13 | 一键使能不要求静止失能 | `_idle` 不要求 `_stable` |
| P14 | 跟踪/终点不触发故障 | `_move` 的 `max_error=None` |
| P15 | 执行前不要求静止/新鲜 | `MonitorTransport._sample` 不因 age 拒绝 |
| P16 | 非指令轴异常只记录 | `command` 用 `_note` |
| P17 | 阶段超时/保持误差不退出 | 只记录 `stage_timeout`/`hold_tolerance_exceeded` |
| P18 | 常量不作拒绝阈值 | `START_ERROR`/`TRAVEL_MARGIN`/`MAX_ACCELERATION` 只用于余量 |
| P19 | 无预览有效期 | 无 `PREVIEW_TTL`/`expires_in_s`/“60 秒” |
| P20 | 非 MIT 无需现场协议配置 | `_motion_supported` 只要求统一模式；特殊帧默认基础 ID |
| P21 | 起始位不要求先全部使能 | `_start_ready` 无 `state==1` 全轴检查；计划自行使能 |
| P22 | 试弹不被手部阻断 | 调度失败记 `hand.score.skipped`，机械臂仍执行 |
| P23 | 起点模式回读只提示 | `start_pose` 用 `_note('mode_readback')`，无 `DriverError` |
| P24 | 接续反馈不完整不 FAULT | 缓存反馈 + 计划起点，记 `motion.continuation_adopted` |

## 6. K01～K12 例外

关节位置限位、NaN/映射/协议帧、CAN 租约/总线/超时、反馈新鲜度 UI、停止失能回读与
“停止未确认”、模式切换静止失能/固件模式回读/回退、C02 盘点/现场协议配置准入、
本机 Host/Origin/令牌、操控页心跳停止、独立 CLI `openarm_trajectory.preflight`（不在网页路径）。

分类：属 P 的新增阻断 → FAIL；属 K → PASS 并说明原因；不在 P/K → UNKNOWN 请用户裁决；
K07 前提扩展到运动/使能 → FAIL；不得用“通信/限位/安全”名义放行 P 项。

## 7. 测试

```
.\.openarm\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider apps/test_openarm_monitor.py apps/test_openarm_monitor_hands.py apps/test_openarm_piano_points.py apps/test_openarm_driver.py apps/test_openarm_execution.py apps/test_openarm_execution_hardware.py apps/test_openarm_position_mode.py apps/test_openarm_dual_sequence.py apps/test_prepare_openarm_dual_sequence.py apps/test_openarm_sequence_dynamics.py
```

记录通过/失败/跳过、耗时、失败名，并核查语义：
- 是否恢复跟踪/终点/迟到/温度/速度力矩/行程/阶段超时的 `pytest.raises`/FAULT 断言；
- 是否删除“放行/记录/advisory/HOLDING_TARGET”用例；
- `check` 是否被改回 `pytest.raises`；一键使能是否被改回“必须静止失能”；
- 起始位/钢琴/手部测试是否要求机械臂 FAULT、自动失能或新运动阈值；
- 是否新增依赖计时过冲的偶发用例。

## 8. 文档

检查 `AGENTS.md`、`monitor-ui/README.md`、防护清单、所有交接 Prompt、审计记录表：
是否删除/弱化第一原则、加入未授权例外、出现“恢复防护”表述，或把新增阻断描述成
“安全/防御性编程/最佳实践”。

## 9. 输出与记录

先给总判定 PASS/FAIL/UNKNOWN，再给：第一原则状态；新阻断全量盘点（扫描范围 + 每条
P/K/UNKNOWN + 是否需裁决，没有则写“未发现”）；P01～P24 表；测试；文档；结论与建议。

写 `logs/openarm/audit/protection_audit_<YYYYMMDD_HHMM>.json`，字段至少：
`time`、`git_head`、`workspace_dirty`、`overall`、`first_principle_intact`、
`new_blocking_checks`、`p_items`、`tests`、`violations`、`notes`。
在 `docs/project/OpenArm防护删除_审计记录.md` 只追加一行摘要，不改历史行
（表头仍写 P01～P19 是历史遗留，输出按 P01～P24）。

## 10. 审计员禁止事项

- 不修改任何代码、测试、配置、AGENTS、README 或交接 Prompt。
- 不 reset/clean/checkout，不覆盖未提交改动。
- 不启动 8092，不连实机，不读写 CAN，不使能，不运动，不写模式。
- 不把“安全/防御性编程/最佳实践”作为恢复防护的理由。
- 不把 K 误判为 P，也不把 P 以“通信/限位”名义放行。
- 不跳过起始位/钢琴/手部路径；发现违规只报告，等用户明确授权后再修。

## 11. 已知未决事项（不在审计范围，但接手需知道）

1. **钢琴点位表已按现场更正统一。** 当前配置、CSV 与控制台均采用左 B3、右 C4 相邻原位，定向回归 51 项通过。第 3 节记录的是更正期间的旧快照。[现行定位与核验](仿真与实机联通_两日实施计划.md#piano-adjacent-origin)
2. `apps/openarm_start_pose.py` 文件头 docstring 写“J3 ±80 / J3 ±60 with J4 +60”，
   实现为 J1/J2/J4 四步，属文档滞后。
3. `docs/project/仿真与实机联通_两日实施计划.md:61,438` 仍写“P01～P19”，未含 P20～P24。

## 12. 快速结论模板

```
审计时间：YYYY-MM-DD HH:MM
工作区 HEAD：<sha>（dirty: yes/no）
第一原则：完整 / 弱化 / 缺失
总判定：PASS / FAIL / UNKNOWN

新阻断全量盘点：
- 扫描范围：<文件/入口>
- 新阻断：<Pxx/Kxx/UNKNOWN + 文件:行 + 行为> 或 “未发现”
- 需用户裁决：<列表>

P01～P24：PASS x / FAIL y / UNKNOWN z
- FAIL：Pxx <file:line> <行为> <证据>
- UNKNOWN：<file:line> <新检查> <无法归类原因>

测试：<passed> passed / <failed> failed / <skipped> skipped，<耗时>
文档：<是否出现恢复防护表述>
建议：<最小修复建议；等待用户授权>
证据：logs/openarm/audit/protection_audit_<...>.json
```
