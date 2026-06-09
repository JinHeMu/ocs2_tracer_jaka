"""
force_admittance_servo_launch.py

将恒力/导纳控制节点集成到现有的 jaka_servo_example.launch.py 中。
本 launch 文件仅启动力控节点本身，与 jaka_servo_example.launch.py 配合使用。

用法（在已有 jaka servo 环境运行后）：
  ros2 launch force_admittance_servo force_admittance_servo.launch.py

或合并进已有 launch 文件（见注释说明）。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("force_admittance_servo")

    # ── Launch 参数 ─────────────────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg, "config", "force_admittance_params.yaml"),
        description="Path to force_admittance_servo params YAML",
    )

    # ── 恒力/导纳控制节点 ──────────────────────────────────────────────────────
    force_admittance_node = Node(
        package="force_admittance_servo",
        executable="force_admittance_servo_node",
        name="force_admittance_servo",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        # 可选：重映射话题
        remappings=[
            # 如果力矩传感器话题名称不同，在此调整
            # ("/tcp_fts_sensor/wrench", "/your_actual_fts_topic"),
            #
            # MoveIt Servo 话题映射（需与 jaka_real_config.yaml 一致）
            # "/servo_node/delta_twist_cmds" → jaka_real_config.yaml: cartesian_command_in_topic
            ("/servo_node/delta_twist_cmds", "/servo_node/delta_twist_cmds"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        force_admittance_node,
    ])

    # ── 集成到现有 launch 文件的方法 ──────────────────────────────────────────
    # 在 jaka_servo_example.launch.py 的 generate_launch_description() 中：
    #
    # from launch.actions import IncludeLaunchDescription
    # from launch.launch_description_sources import PythonLaunchDescriptionSource
    #
    # force_ctrl_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("force_admittance_servo"),
    #             "launch",
    #             "force_admittance_servo.launch.py",
    #         )
    #     )
    # )
    #
    # 然后在 return LaunchDescription([...]) 中添加 force_ctrl_launch

