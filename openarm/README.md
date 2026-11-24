2026-11-15 已统一本机工程，149 项回归通过，双臂运动与《小苹果》完整接触演奏通过。[本机启动与恢复](REPRODUCE_WINDOWS.md) · [新旧资料核对](../docs/guides/仿真统一核对.md)。

日常启动使用[项目根目录的三个入口](../README.md#双击启动)。本目录的 `Start-OpenArm-Debug.cmd` 用于仿真调试。[控制资料与调试指南](调试指南.md)

## 2026-11-09 交接合并说明

当前目录为负责人交接工程，位于 `D:\桌面\open CyMon\弹钢琴\openarm`。统一状态见 [项目总进度](../docs/project/项目总进度.md)，文档导航见 [交接文档索引](交接文档索引.md)。历史进度快照已清理，各模块维护文档见文档索引。

2026-11-10 已使用本目录 `.python311` 中的 Python 3.11.9 重建 `.venv`，运行依赖按 `requirements-lock.txt` 安装，并恢复 Aero SDK 可选依赖。原环境保存在 `.venv.backup-20261109`。两个 OpenArm 源包使用 setuptools 80.9.0 构建后恢复完整模块。本机依赖检查通过，离线 CLI 已加载 V1 模型，完整 pytest 为 149 项通过（38.35 秒）。测试记录位于 `logs/env-rebuild-tests-final.txt`。下文旧 GPU 及测试结果为负责人历史记录。

# OpenArm V1 Windows 离线控制项目 — 阶段 1、2

**HTML 上位机**：启动仿真后双击 `start_web_controller.cmd`，浏览器自动打开无需密码的双臂控制台。支持关节目标、点动、IK 预览、末端执行、实时 3D、XYZ/Rx/Ry/Rz 反馈及日志。使用说明见 [WEB_CONTROL.md](WEB_CONTROL.md)。

支持左右臂分别控制或双臂同步控制，输入关节角度或末端位姿，复用官方 FK/IK 与五次关节轨迹。独立控制面板和常驻仿真服务已支持 14 轴同步执行；夹爪固定。本项目不包含串口通信和真机执行代码。

MuJoCo 钢琴场景可用 `skills/paino_skill/start_piano.cmd` 启动。模型为 A0–C8 的 88 键钢琴（52 白键、36 黑键），可由 Aero 模型指尖接触发声，也可触屏试音。[钢琴场景](skills/paino_skill/README.md)

OpenArm 现场使用 C2–C7 的 61 键琴，双手食指原位为左 B3、右 C4。定位与点位数据见实机控制台说明。[实琴编号与标定表](../monitor-ui/README.md#实琴琴键与食指原位)

MuJoCo 默认采用理想位置跟随，不做重力补偿。双击 `start_mujoco.cmd` 打开常驻仿真端，再双击 `start_controller.cmd` 用独立控制面板发送关节或末端目标，无需按空格。详见 [CONTROL.md](CONTROL.md)。旧的 CSV 回放入口仍为 `verify_simulation.cmd`，见 [SIMULATION.md](SIMULATION.md)。

## Windows PowerShell

已在本机建立独立 `.venv`。进入本目录后无需激活环境：

```powershell
cd 'D:\桌面\open CyMon\弹钢琴\openarm'
.\.venv\Scripts\python.exe -m pytest -q --basetemp=.test-temp
.\.venv\Scripts\python.exe -m openarm_app.cli inspect
.\.venv\Scripts\python.exe -m openarm_app.cli demo --csv trajectory.csv
.\.venv\Scripts\python.exe -m openarm_app.cli fk --q 0 0 0 1.570796 0 0 0
.\.venv\Scripts\python.exe -m openarm_app.cli plan --start 0 0 0 1.570796 0 0 0 --goal 0.1 0.1 0 1.67 0 0 0 --velocity 0.2 0.2 0.2 0.2 0.2 0.2 0.2 --acceleration 0.4 0.4 0.4 0.4 0.4 0.4 0.4 --csv logs/joint.csv
```

新机器安装 Python 3.11 x64，并保留本项目的 vendor 模型副本：

```powershell
.\.python311\python.exe -m venv .venv
.\.venv\Scripts\python.exe -m pip install 'setuptools==80.9.0' wheel
# 固定构建工具并重新构建源码包，避免缓存中缺少模块的 wheel
.\.venv\Scripts\python.exe -m pip install --no-build-isolation --no-cache-dir -r requirements-lock.txt -e '.[test,aero-test]'
```

`ik --pose x y z qw qx qy qz --seed q1 q2 q3 q4 q5 q6 q7` 返回关节角。
`plan` 可用 `--pose` 替代 `--goal`，先解 IK 再导出轨迹。错误返回退出码 2。
`demo` 的起点取模型 home，目标由一组小幅关节变化的 FK 构造，运行 IK → 插补 → CSV 全流程。它是软件验证示例，不代表真机安全起点。

离线 CLI 默认右臂，可在子命令前加 `--arm left`（例如 `python -m openarm_app.cli --arm left inspect`）选择左臂。单臂 CSV 不记录臂侧，左臂导出的 CSV 不要交给旧右臂回放器；双臂仿真请使用 [CONTROL.md](CONTROL.md) 的控制面板或 API。

## MuJoCo 测试与窗口查看

可双击 `start_mujoco.cmd` 打开常驻理想位置仿真服务，默认阴影分辨率 2048；使用 `start_controller.cmd` 控制。
当前虚拟环境的基础解释器为本目录 `.python311\python.exe`，原负责人电脑的解释器路径已失效。
Windows 图形偏好需覆盖这个实际进程路径，而非只设置 `.venv\Scripts\python.exe`。
2026-11-02 已为两个路径设置高性能 GPU；原值备份位于 `logs/gpu-preferences-before-20261102.json`。
此偏好也作用于使用该 BrainLink Python 的其他新图形进程，已运行窗口需要关闭重开。

显卡及渲染速度自检：

```powershell
.\.venv\Scripts\python.exe tools\gpu_diagnostic.py
```

本机修复后报告 `NVIDIA GeForce RTX 2050/PCIe/SSE2`。隐藏窗口 1024×768 基准测试：
2048 阴影约 217 FPS，关闭阴影约 697 FPS；这不是交互窗口、全屏或复杂接触时的帧率保证。

先在 PowerShell 运行数值验收和轨迹生成：

```powershell
cd D:\桌面\open CyMon\弹钢琴\openarm
.\.venv\Scripts\python.exe -m pytest -q --basetemp=.test-temp
.\.venv\Scripts\python.exe -m openarm_app.cli demo --csv trajectory.csv
```

仅查看原模型的官方动力学窗口（不是默认理想位置回放）可运行：

```powershell
.\.venv\Scripts\python.exe -c "from openarm_app.kinematics import OpenArmIK; import mujoco.viewer; arm=OpenArmIK(); mujoco.viewer.launch(arm.setup.model, arm.setup.data)"
```

此原始窗口显示双臂模型；上面示例的 `OpenArmIK()` 默认选择右臂计算。可用空格切换暂停/运行，通过窗口的 Reset 重置；关闭窗口返回终端。官方窗口运行时使用模型的仿真执行器及物理参数，不代表达妙 PosVel 行为。

`demo` 本身只进行离线 FK/IK 与轨迹导出；理想位置回放请使用 `verify_simulation.cmd` 或 `python -m openarm_app.simulation`，可选 `--mode dynamic` 运行动力学对照。详见 SIMULATION.md。
查看器接口参考：https://mujoco.readthedocs.io/en/stable/python.html#interactive-viewer 。

## Python 接口

```python
from openarm_app.kinematics import OpenArmIK
from openarm_app.trajectory import JointTrajectoryPlanner

ik = OpenArmIK()
q_start = ik.home.copy()  # 仅用于离线示例
target_pose = ik.forward(q_start + [0.1, 0.1, 0, 0, 0, 0, 0])
q_goal = ik.inverse(target_pose, seed_q=q_start)
planner = JointTrajectoryPlanner(ik.limits[:, 0], ik.limits[:, 1])
trajectory = planner.plan(q_start, q_goal, [0.2]*7, [0.4]*7)
trajectory.save_csv('trajectory.csv')
```

左臂使用 `OpenArmIK(side="left")`，接口相同。双臂服务为两侧创建独立的官方求解器状态，共享 V1 模型网格；分别求解终点后以同一条 14 轴关节轨迹执行，不做双臂耦合任务或避碰求解。

全部角度 rad、速度 rad/s、加速度 rad/s²、时间 s、位置 m。
位姿为 `[x,y,z,qw,qx,qy,qz]`，相对于 `arm_origin`，不是世界原点。
输入四元数自动归一化，零四元数、NaN、Inf 和错误形状被拒绝。

## V1 适配核对

- 官方 `ArmSetup` 默认指向 V2；本应用显式读取 `vendor/openarm_mujoco/v1/scene.xml`，通过公开构造函数组装 `ArmSetup`。
- 本机 MuJoCo 原生文件接口无法打开含中文的绝对路径。适配层由 Python 读取原版 XML 和网格，使用 `MjModel.from_xml_string(..., assets)` 内存资源加载，避免修改工作目录或复制模型到额外路径；模型与 IK 源码均未修改。
- 模型 Python 包的资源打包仅包含 V2，不能假设 pip 安装后存在 V1；保留官方仓库中的完整 V1 XML、网格资源。可用全局 `--xml` 指定复制后的 V1 scene 路径。
- V1 scene 包含 `openarm_bimanual.xml`，`nq=18`。关节名 `openarm_right_joint1` … `7` 和 `openarm_left_joint1` … `7`；右臂 qpos 索引 9–15，左臂 0–6。所有索引和限位由模型名称解析，不使用电机偏移。
- 两侧 `home` 及夹爪值均从模型 keyframe 读取，夹爪保持该值。单臂控制时另一臂保持当前仿真位置。
- 末端 `right_ee_control_point`、`left_ee_control_point` 和原点 `arm_origin` 均为 site。FK 传 8 项；sync 传右 8 + 左 8；solve 返回 16 项，右臂取 `[0:7]`、左臂取 `[8:15]`。
- 官方 IK 核心保持原样。离线包装器关闭面向实时跟踪的误差整形、奇异点接近限速、零空间回归和动能代价；保留官方关节位置约束，重复 solve 并用 FK 复核，默认最多 300 次外层调用。
- 合格阈值 ≤2 mm、≤2°；失败抛 `IKConvergenceError`，不返回最后一次未收敛结果。限位违反同样失败。有限迭代失败不等于数学证明目标不可达。
- 对象使用有状态的官方求解器，按单线程使用。

## 插补及限制

七轴或十四轴共享 `s(u)=10u³−15u⁴+6u⁵`，零端点速度和加速度。规划器默认 7 轴，使用 `joint_count=14` 启用 14 轴（右七轴在前、左七轴在后）。使用精确峰值 `max s'=15/8`、`max |s''|=10/√3` 自动延长时长，保证连续时间速度和加速度约束。短于所需的用户时长会自动延长。零位移返回常值轨迹。

采样包含两个端点，实际间隔不超过 `sample_period`；默认 0.05 s 仅为离线导出采样参数，不是已测通信能力。单臂 CSV 为 22 列，双臂为 43 列：时间、q、dq、ddq；双臂各字段均按右七轴、左七轴排列。旧 CSV 回放器仅接受右臂 22 列格式。位置在端点之间单调，因此端点限位检查覆盖整段曲线。

这是关节空间路径，末端不保证直线。尚无碰撞检查、动力学/重力/扭矩限制、在线重规划或非零初速度衔接。演示速度、加速度仅为软件测试参数，未做硬件安全认证。

## 上游审查副本

| 项目 | 版本/提交 | 用途 |
|---|---|---|
| [openarm_control](https://github.com/enactic/openarm_control) | 0.4.0 / `2f1ee56ef2bf1272329b213a1f0b0768b9d07f2b` | 固定 Python 包运行；源码用于审查 |
| [openarm_mujoco](https://github.com/enactic/openarm_mujoco) | 仓库 `06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0`；Python 包 2.0.1 | 仓库提供 V1，包提供官方名称解析器 |
| [dora-openarm-kinematics](https://github.com/enactic/dora-openarm-kinematics) | `e40dd559d07e3defdc438f459b636a87ddd39684` | 接口参考，不安装 Dora |
| [motor-sdk](https://github.com/dmBots/motor-sdk) | `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a` | 保存审查，未导入、未执行 |

SDK 路径为 `Python例程/u2can/DM_CAN.py`，已查到 `control_Pos_Vel`；本阶段不推断其速度参数与轨迹导数的对应关系。
vendor 保留各自许可证。config YAML 是当前设计约定的记录，不是运行时可热加载配置。

## 后续阶段

已补充 MuJoCo 理想位置回放、可选动力学对照、暂停/停止和误差日志。未实现：独立虚拟电机接口、历史日志回放、硬件安全状态机、COM 只读接入、电机映射校准、PosVel 发送、异常受控停止、PySide6 界面。不得将 CSV 直接当作可上机运动许可。阶段 3 的完整验收仍未完成。

测试与实际验证记录见 `VALIDATION.md`。


钢琴按键定位调试：`skills/paino_skill/start_piano_controller.cmd`，详见 [使用说明](skills/paino_skill/DEBUG_CONTROLLER.md)。


