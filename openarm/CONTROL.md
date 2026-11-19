# 两个独立程序控制双臂仿真

## 启动和操作

1. 双击 `D:\桌面\open CyMon\弹钢琴\openarm\start_mujoco.cmd`：打开常驻 MuJoCo 仿真端，保持 home 等待命令，不自动执行示例，不需要空格启动。
2. 双击 `D:\桌面\open CyMon\弹钢琴\openarm\start_controller.cmd`：打开独立控制面板，自动连接本机仿真端。
3. 选择“右臂”“左臂”或“双臂同步”。连接后自动填入两侧当前值；未选中的一侧输入框禁用，其机械臂保持当前位置。
4. 在关节页输入 rad，点击“执行所选关节目标”；或者在末端页输入 XYZ 和 wxyz 四元数，先点击“IK 预览”，再点击“执行所选末端目标”。面板同时显示两侧关节、末端 FK 和共同进度。
5. 运动完成后 MuJoCo 保持最终姿态并等待下一条目标，不退出。关闭 MuJoCo 窗口才结束服务。

更新后先关闭旧仿真窗口和旧控制面板，再重新双击两个脚本。新面板会拒绝控制旧版服务，避免臂侧指令被旧服务忽略。

最初测试可从模型 home 开始，选择“双臂同步”，将右臂 J1 改为 `0.1`、左臂 J1 改为 `-0.1`，其余角度保留当前值，执行关节目标。完成后选择“双臂同步”点击“所选臂回 Home”。这是仿真示例，不是硬件运动许可。

XYZ 单位 m，相对于 arm_origin；四元数顺序 qw qx qy qz。两侧夹爪固定 home，左右臂七轴均为理想位置跟随；无重力补偿、无动力学积分、无 COM/CAN 通信。

“所选臂回 Home”只重置选中的臂；“全部暂停”“继续”“全部停止并保持”作用于整个当前动作，切换界面选择不会改变正在执行的动作。

同一时间只接受一个动作：运动中或暂停时拒绝任何新目标，包括另一臂的目标；要同时运动，使用“双臂同步”一次提交两侧目标。暂停冻结共同轨迹时间；停止立即冻结两侧当前位置。下一次动作由当前位置、零初速度重新规划，这是理想仿真假设。
关闭控制面板不会终止已接受的动作；可以重开面板继续观察和控制。网络请求超时不自动重发运动，先查询状态以免重复执行。

## 程序结构

控制面板（独立进程，Tkinter） → 本机 HTTP JSON → MuJoCo 仿真端（独立进程）。

仿真端共享 V1 模型网格，为两侧分别创建官方 IK 状态，独立求解两侧终点，再以一条 14 轴五次轨迹同步起止。右臂在前、左臂在后；单侧运动时另一侧轨迹保持常值。任何一侧验证或 IK 失败，整个新动作被拒绝，不执行另一侧。按解析五次曲线更新理想位置，不依赖 CSV。
默认每轴速度上限 0.2 rad/s，加速度上限 0.4 rad/s²；规划时间会自动延长。图形循环以 60 Hz 推进仿真时间，系统繁忙时会慢于墙钟时间，不承诺实时控制。
状态是 IDLE、EXECUTING、PAUSED、STOPPED、COMPLETED；反馈的“实际角度”是理想仿真角度，不是硬件测量。
仿真端为运动学求解的共享数据加锁；IK 离线求解期间画面/状态请求可能短暂等待。

## 让你自己的程序控制

仿真启动后，其他 Python 程序可调用：

```python
from openarm_app.sim_client import SimClient

arm = SimClient()  # 127.0.0.1:8769
print(arm.state())
arm.command("move_joints", q=[0.1, 0.1, 0, 1.67, 0, 0, 0])
# 等待完成后再发送下一条目标，或先 arm.command("stop")
```

省略 `arm` 时仍默认右臂，旧调用兼容。控制左臂或双臂：

```python
state = arm.state()
right = state["arms"]["right"]["q"].copy()
left = state["arms"]["left"]["q"].copy()
right[0] += 0.1
left[0] -= 0.1
sent = arm.command("move_joints", arm="both", q={"right": right, "left": left})
motion_id = sent["motion_id"]
# 查询 state，确认 motion_id 相同且 state == "COMPLETED" 后再发下一条。
# 只控制左臂时使用 arm.command("move_joints", arm="left", q=left)。
```

末端目标：

```python
pose = [0.232704, -0.150961, -0.177241, 0.653241, 0.033270, -0.752708, 0.074834]
preview = arm.command("preview_pose", pose=pose)  # 不运动
print(preview["q_goal"])
arm.command("move_pose", pose=pose)
```

双臂末端目标使用 `arm="both", pose={"right": right_pose, "left": left_pose}`，每侧各 7 个数。`preview_pose` 同样接受此格式；成功返回的 `q_goal` 与 `fk_pose` 均按 `right`、`left` 分组。单臂返回格式保持为 7 元素列表，并增加 `arm` 标识。预览使用当前关节作初值，不改变仿真姿态。

这些示例应使用项目 .venv Python，或在调用程序中安装本项目。任意其他语言也可通过下面的协议接入。

### 本机接口

仅监听 `127.0.0.1:8769`，不监听局域网。没有认证，适用于可信本机程序；拒绝带 Origin 的浏览器命令和非 JSON 请求，不提供 CORS。

- `GET /state`：`{"ok":true,"result":{...}}`。`api_version=2`，`arms.right` 和 `arms.left` 各包含 q、dq、pose、goal、home、limits。顶层旧字段 q、dq、pose、goal、home、limits 始终指右臂；新增 q14（右七轴＋左七轴）、joint_order 和 active_arms（最近接受动作选择的臂）。state、progress、motion_id、elapsed_s、duration_s 为整个动作共享。
- `POST /command`，`Content-Type: application/json`：

```json
{"action":"move_joints","q":[0.1,0.1,0,1.67,0,0,0]}
```

支持 action：move_joints、move_pose、preview_pose、home、pause、resume、stop。
arm 可为 right、left、both，默认 right。move_joints 使用 q；move_pose/preview_pose 使用 pose。
单臂目标是 7 元素数组；双臂目标必须是包含 right 和 left 两个 7 元素数组的对象。home 根据 arm 选择回零范围，pause/resume/stop 总是作用于整个动作。
运动命令可选 velocity_limits、acceleration_limits（单臂各 7 数值、双臂各 14 数值，右在前）和 duration（秒）。过短 duration 会按两侧的限值统一延长。缺省速度 0.2 rad/s、加速度 0.4 rad/s²。
无效、越限、不可达或忙碌请求返回 HTTP 400 与 `{"ok":false,"error":"..."}`，不替换现有目标。
命令成功只表示接受/完成规划，**不表示动作已经完成**；请持续读取 state 和 motion_id。

命令行也可控制：

```powershell
cd D:\桌面\open CyMon\弹钢琴\openarm
.\.venv\Scripts\python.exe -m openarm_app.sim_client state
.\.venv\Scripts\python.exe -m openarm_app.sim_client move_joints 0.1 0.1 0 1.67 0 0 0
.\.venv\Scripts\python.exe -m openarm_app.sim_client --arm left home
.\.venv\Scripts\python.exe -m openarm_app.sim_client --arm both home
.\.venv\Scripts\python.exe -m openarm_app.sim_client stop
```

`--arm both move_joints`、`--arm both move_pose` 和 `--arm both preview_pose` 后面填 14 个数，先右臂七项、后左臂七项；CLI 会转换为分组 JSON。

端口冲突时：仿真端、控制面板和命令行客户端均可用 `--port 8770`，双方保持相同端口。
无窗口服务：`python -m openarm_app.sim_server --headless`。

## 验证及范围

双臂验收包括左右 FK/IK 回代、14 轴同步与导出、单臂动作保持另一侧、夹爪保持、无效双臂目标整体拒绝、暂停/继续/停止与 Home。`tools/check_remote_control.py` 用两个真实独立进程验证右臂兼容调用、左臂单独运动、双臂关节/末端运动、双臂预览、同步暂停/继续、双臂 Home、左臂 CLI 和服务持续运行。实际测试结果见 VALIDATION.md。

独显偏好沿用已修复的实际用户环境配置；仿真启动记录在 logs/server-日期-时间/graphics.json，退出时保存 final-state.json。
本入口目前没有完整逐帧历史轨迹记录。旧 CSV 回放仍使用 verify_simulation.cmd，见 SIMULATION.md。
未实现双臂碰撞检查或避碰规划、双臂共同搬运的相对位姿约束、力控制、夹爪控制或真机执行。两侧 IK 分别满足各自目标，不保证两臂之间或与道具之间无碰撞；理想位置更新也不会因接触自动阻止穿透。钢琴已实现独立场景（skills/paino_skill/start_piano.cmd），双臂换装 Aero 灵巧手，手指和琴键暂时固定，详见 skills/paino_skill/README.md；架子鼓和绘画仍是场景模板。

