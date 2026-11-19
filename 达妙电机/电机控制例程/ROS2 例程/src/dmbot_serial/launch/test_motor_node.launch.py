from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def int_list_param(value):
    return ParameterValue(
        PythonExpression(["list(", value, ")"]),
        value_type=List[int],
    )


def str_list_param(value):
    return ParameterValue(
        PythonExpression(["list(", value, ")"]),
        value_type=List[str],
    )


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    can_mode = LaunchConfiguration("can_mode")
    motor_types = LaunchConfiguration("motor_types")
    motor_can_ids = LaunchConfiguration("motor_can_ids")
    motor_master_ids = LaunchConfiguration("motor_master_ids")
    control_mode = LaunchConfiguration("control_mode")
    send_rate_hz = LaunchConfiguration("send_rate_hz")

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("can_mode", default_value="canfd"),
        DeclareLaunchArgument("motor_types", default_value="['DM4310', 'DM4310']"),
        DeclareLaunchArgument("motor_can_ids", default_value="[0x01, 0x02]"),
        DeclareLaunchArgument("motor_master_ids", default_value="[0x11, 0x12]"),
        DeclareLaunchArgument("control_mode", default_value="pos_vel"),
        DeclareLaunchArgument("send_rate_hz", default_value="1000.0"),
        Node(
            package="dmbot_serial",
            executable="test_motor_node",
            name="test_motor_node",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
                "can_mode": can_mode,
                "motor_types": str_list_param(motor_types),
                "motor_can_ids": int_list_param(motor_can_ids),
                "motor_master_ids": int_list_param(motor_master_ids),
                "control_mode": control_mode,
                "send_rate_hz": ParameterValue(send_rate_hz, value_type=float),
            }],
        ),
    ])
