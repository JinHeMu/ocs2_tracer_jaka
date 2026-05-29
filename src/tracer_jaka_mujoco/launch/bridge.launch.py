#!/usr/bin/env python3
"""
方式 A（推荐，零第三方插件依赖）:
  MuJoCo 桥接节点 + robot_state_publisher + (可选) RViz

启动:
  ros2 launch tracer_jaka_mujoco bridge.launch.py
  ros2 launch tracer_jaka_mujoco bridge.launch.py rviz:=false
控制:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard       # 键盘控底盘 (/cmd_vel)
  ros2 topic pub /arm_command std_msgs/Float64MultiArray "{data: [0,0.5,1.0,0,0.5,0]}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "tracer_jaka_mujoco"
    share = get_package_share_directory(pkg)
    urdf = os.path.join(share, "urdf", "tracer_jaka_zu5.urdf")
    rviz = os.path.join(share, "rviz", "view.rviz")
    model = os.path.join(share, "models", "tracer_jaka_zu5_fixed.xml")

    with open(urdf, "r") as f:
        robot_description = {"robot_description": f.read()}

    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="whether to start RViz"),

        Node(package=pkg, executable="mujoco_bridge", output="screen",
             parameters=[{"model_path": model}]),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[robot_description]),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz], output="screen",
             condition=IfCondition(use_rviz)),
    ])
