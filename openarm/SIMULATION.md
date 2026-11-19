# MuJoCo 理想位置仿真

本文描述旧的 CSV 回放。现在 `start_mujoco.cmd` 启动常驻仿真服务，配合独立 `start_controller.cmd` 控制面板使用，详见 CONTROL.md；不再自动回放示例或等待空格。

默认采用用户指定的理想假设：每个电机精确跟随轨迹目标位置。
CSV 目标 → 七轴理想位置跟随 → MuJoCo 正运动学与画面 → 日志。
每个采样时刻直接令实际关节角等于轨迹角度，调用 mj_forward 更新模型；不调用 mj_step，不做重力补偿。左臂和夹爪固定在 home。官方模型和 IK 保持不变。

## 开始

双击 `D:\桌面\open CyMon\弹钢琴\openarm\verify_simulation.cmd` 进入 CSV 理想位置回放。
脚本先生成独立的 simulation_demo.csv，不覆盖用户的 trajectory.csv。

1. 关闭之前的旧仿真窗口，重新运行脚本。
2. 点击新窗口后按空格开始；再按空格暂停/继续。
3. 前 1 秒保持起点，随后右臂执行约 1.22 秒的示例轨迹。
4. 轨迹完成后停留在最终姿态，立即保存结果并在终端显示；可以继续旋转、缩放观察模型。此时空格不会重新执行。
5. Q 或关闭窗口退出；执行中退出记为 STOPPED，完成后退出保留原验收结果。
6. 查看终端提示的 logs/simulation-日期-时间/ 下的 summary.json 和 tracking.csv，无需关闭画面才保存。

图形模式启动时还会显示 `OpenGL renderer` 和进程 PID，并立即保存 `graphics.json`。
双击启动脚本会先在实际启动用户环境内，为虚拟环境及基础 Python 写入高性能 GPU 偏好，然后启动新的仿真进程。已有其他显示选项保留；有改动时将原值备份到 logs/gpu-preferences-user-*.json。
这个偏好也影响使用同一基础 BrainLink Python 的其他新图形进程，不修改全局显卡设置。可通过 Windows 图形设置恢复；该脚本下次运行仍会请求高性能 GPU。
该记录来自查看器同一进程中的隐藏 GLFW 上下文探测，包含实际 Python 路径、用户、环境和显卡名称，不是帧率测量。
遇到双击启动与终端启动表现不同，请保持窗口暂停以便比较两次 graphics.json 及对应 PID 的 GPU 使用情况。
本机曾实测：工具启动环境可读到 GPU 偏好，但资源管理器启动环境读不到，导致前者使用 NVIDIA、后者使用 Intel。现由启动脚本在实际环境内配置，避免仅依赖工具环境写入结果。

```powershell
cd D:\桌面\open CyMon\弹钢琴\openarm
# 理想回放（默认）
.\verify_simulation.cmd
# 回放自己的轨迹
.\.venv\Scripts\python.exe -m openarm_app.simulation --csv trajectory.csv
# 不开窗口检查
.\.venv\Scripts\python.exe -m openarm_app.simulation --csv trajectory.csv --headless --output logs/my-ideal-check
```

## 验证范围

验证七轴动作顺序、关节路径、终点 FK、轨迹时间、同步到达、模型限位以及暂停/停止和日志。
关节跟踪误差为零是输入的理想假设，不是测出了真实电机精度。
日志中的末端误差相对于 CSV 最后一个关节目标的 FK；原始用户输入位姿的 IK 回代误差由阶段 1 单独检查。
此模式不评估电机扭矩、重力负载、惯性、接触响应或实际硬件跟踪误差，也不模拟达妙固件。

## 采样与结果

CSV 使用规划器的 22 列表头，时间从零开始且严格递增，关节角需有限且在模型限位内。
默认按模型 1 ms 时间步推进，CSV 位置点之间线性插值。理想速度取该分段直线的斜率，起点保持及终点保持时为零；CSV 的 dq/ddq 列不参与回放。
连续五次多项式的速度/加速度验收由轨迹规划器及其测试承担，回放本身是位置采样序列的展示。

- PASSED：走完轨迹并保持 0.25 秒；理想模式终点关节误差 ≤1e-12 rad，速度 ≤0.02 rad/s，终点 FK 误差 ≤2 mm / 2°，且未观察到关节越限。
- STOPPED：用户停止，不视为完成。
- FAULT：数值或 MuJoCo 状态异常。
- NOT_REACHED：终点等待超时，主要用于动力学对照。

tracking.csv 每步记录七轴目标、实际位置、误差、速度、末端位姿、终点误差和接触数。
summary.json 明确标注 mode: ideal、gravity_compensation: false 及验证范围。
接触数只作观察，当前没有危险碰撞分类和安全验收。显式指定 --output 会覆盖目录内同名结果文件。

## 可选的动力学对照

只有明确指定 --mode dynamic 才运行原先的物理模拟：

```powershell
.\verify_simulation.cmd --mode dynamic
.\verify_simulation.cmd --mode dynamic --gravity-compensation
```

原模型对照使用官方位置 PD 执行器，示例终点误差约 10.136 mm / 3.660°，未到达。
理想重力补偿的对照约 0.080 mm / 0.124°，通过。这些是此前模型实验，不是默认模式，也不是达妙 POS_VEL 的精度结论。
理想模式与 --gravity-compensation 同用会明确报错。

## 本机验证

2026-11-02：默认理想模式 PASSED，2466 个采样步，仿真时间 2.466 s。
七轴峰值跟踪误差均为 0，终点 FK 位置及姿态误差均为 0。结果见 logs/simulation-ideal-check。
新增理想模式测试确认：不调用动力学积分、全程目标与实际位置完全一致、左臂和夹爪不动、终点速度为零、停止冻结状态、错误模式参数被拒绝。

独立 MockMotorDriver、历史日志回放和硬件执行仍未实现，阶段 3 完整范围尚未验收。

