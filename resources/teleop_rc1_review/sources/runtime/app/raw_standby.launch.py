# 项目中文注释（2026-10-29）：原资料包 ROS 待机启动。
# 调用与职责：加载本地双臂模型、唯一控制管理器和资料包桥适配器；can0 右臂、can1 左臂。
# 运行边界：仅构建未配置待机；不自动激活控制器或回零。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Original archive robot and bridge, without automatic hardware activation."""

import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# 注释：用同一套本地 xacro 构建双臂描述，把两个硬件组件显式置为 unconfigured，再启动原桥适配器。
def generate_launch_description():
    share = get_package_share_directory("openarm_description")
    sys.path.insert(0, os.path.join(share, "launch"))
    import openarm_common
    description = xacro.process_file(
        os.path.join(share, "assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro"),
        mappings={"arm_type": "openarm_v1.0", "bimanual": "true", "use_fake_hardware": "false",
                  "ros2_control": "true", "right_can_interface": "can0", "left_can_interface": "can1",
                  "return_to_zero": "false", "gravity_scale": "1.0", "teaching_mode": "false"},
    ).toprettyxml(indent="  ")
    controllers = os.path.join(get_package_share_directory("openarm_bringup"),
                               "config/controllers/openarm_bimanual_controllers.yaml")
    manager = Node(
        package="controller_manager", executable="ros2_control_node", output="both",
        parameters=[controllers, {"hardware_components_initial_state.unconfigured":
                                 ["openarm_right_hardware_interface", "openarm_left_hardware_interface"]}],
        remappings=[("~/robot_description", "robot_description")],
    )
    # Keep the archive bridge's original /joint_trajectory and XR Joy topics.
    # No controller spawner, cleanup command, configure, or activate action runs.
    bridge = ExecuteProcess(cmd=["/usr/bin/python3", "-B", "/test/app/archive_bridge_adapter.py"], output="screen")
    return LaunchDescription([openarm_common.robot_state_publisher_node(description), manager, bridge])
