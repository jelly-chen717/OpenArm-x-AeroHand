# 双手并排并拢姿态（2026-11-03）

目标：双手掌面朝下、手指朝前、内侧外形边缘接触。拇指保持当前向下姿态，琴体位置不变。通过 openarm-control 0.4.0 官方 IK 求解，没有修改或重写 IK，也没有向运行中的仿真发送命令。

## 可见外形间距为零的候选

可见几何（网格凸包）最小距离约 0.000003 mm，可视为数值容差内接触。手腕中心间距约 93.829 mm。两臂末端控制点相对 arm_origin 为 x=0.32 m、z=-0.18 m、y=±0.04691444 m，保持 Home 的手腕朝向。最近关节限位余量约 1.879°。此姿态位于琴键上方，不是按键姿态。

| 关节 | 右臂 rad | 左臂 rad |
|---|---:|---:|
| J1 | 0.568683147 | -0.568683147 |
| J2 | -0.141745776 | 0.141745776 |
| J3 | -0.327924788 | 0.327924788 |
| J4 | 1.005718708 | 1.005718708 |
| J5 | 0.054817144 | -0.054817144 |
| J6 | -0.352165520 | 0.352165520 |
| J7 | -0.017339909 | 0.017339909 |

精确数值见 [posture.json](logs/adjacent-hands-visual/posture.json)。

![外形并拢预览](logs/adjacent-hands-visual/preview.png)

**限制：原模型的 palm_collision_8/10/11 和 th_mp_collision 等简化碰撞盒仍发生重叠，最大报告穿透约 17.5 mm。不能称为无碰撞姿态，也不能据此直接执行运动。** 外形距离使用 MuJoCo 网格凸包，不是实物测量；手臂到达该点的连续路径尚未检查。

## 保守对照

若要求当前碰撞盒也不穿透，保持同一水平手姿时，找到的手腕中心间距约为 150 mm；可见掌壳内侧还相隔约 56.17 mm，不能满足照片里的可见外形零间距。

该对照的静态模型无超过 1 微米的接触穿透，精确关节角见 [对照结果](logs/adjacent-hands-conservative/posture.json)。这不等于连续路径安全。

## 重现

从 D:\openarm 运行：

```powershell
.venv/Scripts/python.exe -m paino_skill.find_adjacent --metric visual
.venv/Scripts/python.exe -m paino_skill.find_adjacent --metric conservative
```

两种搜索均只做离线计算并保存结果，不改变默认启动姿态、不发送控制命令。
