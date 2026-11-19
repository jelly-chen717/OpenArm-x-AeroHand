# HTML 四区上位机

无需密码。先关闭旧的 MuJoCo 和网页服务，再启动 `start_mujoco.cmd`（或者 `skills/paino_skill/start_piano.cmd`），最后启动 `start_web_controller.cmd`。浏览器打开 http://127.0.0.1:8088 。本次新增了模型显示接口，需要重启两个服务并刷新网页。

## 页面布局

- 左上：当前 MuJoCo 场景的真实 3D 模型。鼠标拖动旋转、滚轮缩放、右键拖动平移；“复位视角”恢复相机。模型随关节反馈更新，钢琴场景包含双手和琴键。没有应用场景菜单。
- 右上：左右臂末端 IK 控制，输入 X/Y/Z（米）、Rx/Ry/Rz（度）；预览只填入关节目标，执行按钮才产生运动。
- 右下：左右臂关节角度与目标（度），± 按选定步长立即点动该臂；“执行关节目标”按所选控制对象提交目标。
- 左下：左右臂实时 X/Y/Z、Rx/Ry/Rz，附折叠操作日志。小屏幕自动纵向排列。

## 坐标与控制

XYZ 相对于 arm_origin，末端为对应机械臂的 ee_control_point，并非灵巧手指尖。
Rx/Ry/Rz 使用固定轴 roll/pitch/yaw，R = Rz(rz) × Ry(ry) × Rx(rx)。网页与仿真接口之间转换成 wxyz 四元数，官方 IK 核心不变。欧拉角在 ±90° 俯仰处不唯一；接近该姿态（cos(pitch) < 1e-5）时采用 Rx=0 的稳定表示，旋转误差小于 1e-5 rad。

选择左臂、右臂或双臂同步；另一侧单臂动作期间保持当前位置。速度倍率 100% 对应 0.2 rad/s、0.4 rad/s²，默认 50%，只作用于下一条命令。暂停、继续、停止作用于整个动作；Home 只作用于所选臂。请求处理中点击停止会排队到该请求返回之后发送。关闭网页不会停止已接受的运动。

断线后保留最后的 3D 姿态和数值，并显示反馈断开、禁用运动按钮。3D 是实际模型的反馈展示，不做额外动力学积分，不产生运动指令，不提供避碰保证。手指和琴键仍固定。

## 服务与资源

`python -m openarm_app.web_controller --port 8088 --sim-port 8769` 指定网页及仿真端口；`--no-browser` 禁用自动打开浏览器。两端仅监听回环地址，没有登录密码，网页服务仍校验同源请求。

页面使用本地 Three.js 0.179.1（MIT），许可证保存在 `src/openarm_app/web/lib/THREE-LICENSE.txt`；OrbitControls 仅调整了本地 import 路径。运行不需要 CDN、Node.js 或参考项目目录。模型来自实际仿真服务 `/visual/model`，包含编译后的网格、材质与几何尺寸；`/state` 的 visual 字段提供同序几何体的世界位置和旋转矩阵，以及服务实例标识。网页根据实例标识重新载入模型。网格用于显示，官方 V1 资产和 vendor 未修改。

## 验证

Python 回归：70 项通过。浏览器测试分别通过标准和钢琴仿真；覆盖 3D 加载、四区位置、双臂运动、暂停/继续、六维 IK、Home/停止、单臂点动、限位、断线禁用和手机布局。还检查 105 组欧拉角与四元数回代（含奇异与跨 ±90° 姿态）。截图位于 logs/web-controller-desktop.png 和 logs/web-controller-mobile.png。

开发浏览器测试使用 `tools/check_web_ui.py`，需要 Playwright、Node.js 和 Edge；`OPENARM_TEST_SCENE=piano` 可验证钢琴场景。上位机运行本身不需要这些测试工具。

