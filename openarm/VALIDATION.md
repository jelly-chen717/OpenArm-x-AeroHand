2026-11-15 当前版本验证与证据见 [统一核对记录](../docs/guides/仿真统一核对.md)。以下历史记录涉及的旧输出已包含在唯一原始备份中。

# 阶段 1、2 验收记录

## HTML 四区布局与实时 3D（2026-11-03）

按用户要求移除应用场景页，改为左上 3D、右上末端 IK、右下关节控制、左下 XYZ/Rx/Ry/Rz。末端输入采用固定轴 Rz·Ry·Rx 欧拉角并转换为官方 wxyz 接口。Three.js 读取真实 MuJoCo 编译网格与世界变换，不伪造机器人几何。

完整回归：70 passed in 20.74s，含新增网格字节、索引、材质及世界姿态导出验证。真实 Edge 浏览器分别在标准和钢琴临时端口完成联调，包含真实模型加载、四区布局检查、双臂执行、IK、暂停/继续、Home/停止、点动、限位及断线禁用。105 组欧拉角回代测试含 ±90° 奇异姿态。最终钢琴桌面与手机截图已检查。未向用户原有仿真发送运动。

以下为历史版本记录；旧版“没有内嵌 3D”和“应用场景页”说明已由本节替代。


## HTML 上位机（2026-11-03）

新增无需密码的本机 HTML 控制台，参考 G1 Web Control 的蓝白布局和左右臂点动交互，页面和后端均为本项目实现。保留独立 MuJoCo 渲染窗口，通过本机网页代理接入既有仿真，不修改仿真通信限制。

完整回归：`69 passed in 20.89s`，命令为 `.venv/Scripts/python.exe -m pytest -q --basetemp=.test-temp-web-final --tb=short`。包含新增网页静态资源、同源命令转发及跨源/非法请求拒绝测试，原双臂、GUI、IK、轨迹和钢琴场景测试继续通过。

真实 Edge 无头浏览器与独立 MuJoCo/网页进程联调通过：双臂运动、暂停/继续、IK 预览、末端目标、Home/停止、左臂点动、关节越限、场景信息页、断线禁用及 390px 宽布局。测试只向独立临时端口发送命令，未控制用户原有仿真。桌面与窄屏截图已检查，未发现横向溢出。首次尝试因 Playwright 自带 Chromium 未安装失败，改用已安装 Edge 后通过；未下载浏览器。

开发联调借用参考目录现有的 Playwright 测试依赖，未修改参考项目；上位机运行本身不依赖该目录、Node.js 或 Playwright。启动及范围见 WEB_CONTROL.md。

## 双臂仿真扩展（2026-11-02）

当前实现：左右臂各自的官方 FK/IK、14 轴同步五次规划、本机服务的 right/left/both 选择、双臂状态反馈与独立控制面板。单臂动作保持另一侧当前位置，两侧夹爪保持 home；任一侧目标失败时整个动作拒绝。

最终完整回归：`.venv/Scripts/python.exe -m pytest -q -rs --basetemp=.test-temp-dual-verified --tb=short`，**58 passed in 10.96s**，无跳过。使用真实 Windows Python 3.11、MuJoCo 与官方 IK 依赖。覆盖两侧 FK 坐标变换与 IK 回代、左右索引及夹爪隔离、14 轴限速同步与 43 列 CSV、双臂目标原子性、单臂保持另一侧、暂停/继续/停止、回 Home、HTTP 协议及旧右臂接口兼容。

界面测试使用隐藏 Tk 窗口，验证左右选择、双臂载荷、切换选择后 IK 回复仍填入正确侧、旧服务拒绝控制以及窗口布局尺寸；未进行人工视觉验收。早期测试在同一进程反复重建 Tk 解释器时出现一次 init.tcl 初始化错误；改为共用一个解释器、每项测试新建窗口后，完整测试全部通过。

`tools/check_remote_control.py` 两个真实独立进程联调通过（控制进程 33116、仿真进程 19116）：右臂兼容调用、左臂保持右臂、双臂关节目标、双臂暂停/继续、双臂 IK 预览与末端执行、双臂 Home、左臂 CLI、停止及服务持续运行。测试使用独立临时回环端口，未向用户原有仿真服务发送运动。

左臂离线 `--arm left demo` 生成 `logs/dual-validation/left-demo.csv`，26 个采样，1.2248666324 s；示例 FK 回代误差 2.9802322388e-08 m、1.6593230852e-07 rad。这是特定示例的数值结果，不代表全工作空间或真机精度。

未实现双臂避碰、相对位姿约束/协同搬运、工具与场景道具、夹爪控制或硬件功能。旧 CSV 回放入口仍只处理右臂。下文为此前各阶段的历史验收记录。

## 独立控制程序（2026-11-02）

新增 sim_server、sim_client 和 Tk 控制面板。回归 38 passed in 10.35s。
两个真实独立进程联调通过（控制进程 45196，仿真进程 13768）：查询、关节运动、暂停/继续、IK 预览、末端运动、回 Home、停止、动作完成后服务不退出。
Tk 窗口构造检查通过。受限执行环境不允许回环连接，网络测试在允许本机通信的环境重跑通过；未跳过测试。

## 资源管理器启动的 GPU 修复（2026-11-02）

用户双击进程 17080 的 graphics.json 报告 Intel Iris Xe，而工具环境对照报告 NVIDIA；资源管理器诊断读到两个 Python 路径的 GPU 偏好缺失。
启动脚本现先在实际调用用户环境内配置/核对这两个路径，再启动独立仿真进程。保留其他显示选项，改动前写入备份。
修复后用户自己双击的进程 38544（Console 会话）报告 NVIDIA GeForce RTX 2050，并已出现在 nvidia-smi 的 C+G 进程列表；我方额外复核窗口已关闭，保留用户窗口。
记录：logs/simulation-20261102-190535/graphics.json（修复前）、logs/simulation-20261102-190952/graphics.json（用户修复后）。
最终回归：29 passed in 12.68s。此前一次并行诊断时模型分配内存失败，关闭旧查看器后重跑全部通过。

## 最新默认：理想位置跟随

2026-11-02 按用户要求，将仿真默认设为 ideal：直接使用轨迹目标关节角更新模型运动学，无动力学积分，无重力补偿。
无窗口示例 PASSED：2466 步、2.466 s，七轴全程跟踪误差和终点 FK 误差均为 0（理想假设，不是电机性能测量）。
新增测试覆盖全程位置精确一致、不会调用 mj_step、另一臂保持、插值与停止及模式组合校验。
最终完整测试：29 passed in 11.63s。
下面的动力学及其误差数值仅为历史/可选 --mode dynamic 对照，不能作为达妙 POS_VEL 的结论。

## 后续补充：MuJoCo 动态轨迹验证

2026-11-02 新增 `openarm_app.simulation` 和 `verify_simulation.cmd`。
最新完整测试结果：`25 passed in 8.50s`，包含真实物理状态与 FK 数据隔离、CSV 拒绝异常、停止行为、原模型不达标检测、理想重力补偿达标及日志检查。
原模型动态示例结果为 NOT_REACHED（10.136 mm / 3.660°），理想重力补偿对照为 PASSED（0.080 mm / 0.124°）。
不应将“测试全部通过”与“原模型动态精度达标”混为一谈：前者包含验证系统正确识别未达标的测试。
交互窗口暂停/停止的实际操作尚未人工验收；阶段 3 完整范围尚未完成。详见 SIMULATION.md。

以下为此前阶段 1、2 的历史验收记录。

日期：2026-11-02（Asia/Shanghai）。本机 Windows x64、Python 3.11.17，独立 `.venv`。

同日已迁移至 `D:\openarm`，重新安装本地项目并修复环境路径。迁移后测试：`18 passed in 2.30s`；demo 再次成功生成 26 点 CSV，导入路径确认为 `D:\openarm\src\openarm_app\__init__.py`。本次未验收交互查看器窗口。

## 实际执行结果

```text
.venv/Scripts/python.exe -m pytest -q --basetemp=.test-temp-vfs
18 passed in 2.54s
```

没有跳过测试。覆盖真实 MuJoCo V1 模型加载、右臂索引核对、直接 MuJoCo 坐标变换与 FK 对照、三个非平凡 IK 回代、不可达目标失败、四元数等价性与输入校验、官方 solver 返回失败、左臂与夹爪保持、五次曲线导数数值核对、连续峰值约束、零位移、采样时间和 CSV 往返。

此外源码 compileall 通过；四份 vendor 副本的 git status 均为空，没有改动官方源码。

```text
.venv/Scripts/python.exe -m openarm_app.cli demo --csv trajectory.csv
duration_s = 1.21654795752721
samples = 26
position_error_m = 5.960464477539063e-08
orientation_error_rad = 5.16191365590357e-08
```

这是特定模型和示例目标的数值结果，不代表全工作空间精度或真实机械误差。
生成的 `trajectory.csv` 有 22 列，包含起终点及 q/dq/ddq。`inspect` 成功输出 home、限位、末端位姿与 qpos 索引。

## 固定依赖

实际安装：openarm-control 0.4.0、openarm-mujoco 2.0.1、DAQP 0.7.2、mink 1.1.0、MuJoCo 3.17.0、NumPy 2.4.6、SciPy 1.17.1、pytest 9.2.0。
完整版本清单在 `requirements-lock.txt`，上游提交在 `vendor/UPSTREAM.md` 和 README。
DAQP 的 Windows CPython 3.11 安装及真实求解调用均通过。

## 发现并处理的问题

1. 上游 control 主分支是 0.5.0；运行固定为需求中的 0.4.0。
2. openarm-mujoco Python 包只打包 V2 模型资源；V1 使用固定仓库副本。
3. V1 包含双臂，右臂 qpos 是 9–15；官方返回值是右 8 + 左 8，不能混用。
4. Windows 中文绝对路径使 MuJoCo 原生打开 XML 失败；现使用 Python 读取和 MuJoCo 内存资源加载，完整测试已验证修复。
5. 受限执行环境的默认临时目录不可写；测试通过 `--basetemp` 将临时文件放在项目内。

## 里程碑

- [x] V1.0 MJCF 加载成功
- [x] FK 返回 XYZ + wxyz 四元数
- [x] 右臂七轴 IK 通过 FK 回代，失败明确报错
- [x] 五次插补满足模型关节限位及速度/加速度约束
- [x] 生成 trajectory.csv
- [x] Windows Python 3.11 全部项目测试通过

阶段 1、2 已完成。阶段 3–6 尚未实现；本次未连接任何真实串口、未导入电机 SDK、未发送 CAN、未使能电机。未验证碰撞、机械零位、电机方向、真机运动或通信频率。


## 钢琴场景验证

钢琴相关验证记录已移至 [skills/paino_skill/VALIDATION.md](skills/paino_skill/VALIDATION.md)。
