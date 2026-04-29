#!/usr/bin/env python3
# =============================================================================
# 仅 RViz 显示（joint_state_publisher_gui 拖滑块），不启动 Gazebo
# 用于先快速 sanity check URDF 是否正确解析
#
# 注意：这里通过 static_transform_publisher 把 odom 固定到 base_footprint，
#       这样 RViz 的 Fixed Frame=odom 仍然有效；上 Gazebo 时 diff_drive_controller
#       会替代这个静态 TF 发布动态 odom。
# =============================================================================

from launch import LaunchDescription
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('tracer_jaka_gazebo')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'tracer_jaka.urdf.xacro'])
    rviz_cfg   = PathJoinSubstitution([pkg_share, 'rviz', 'tracer_jaka.rviz'])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'sim_mode:=false'
    ])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([

        # 静态 TF：odom -> base_footprint （仅在没有 Gazebo / 仿真时使用）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen',
        ),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[robot_description],
             output='screen'),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             output='screen'),

        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', rviz_cfg],
             output='screen'),
    ])

