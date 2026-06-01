#!/usr/bin/env python3
"""
bridge.launch.py —— 纯桥接方案（不依赖 mujoco_ros2_control / diff_drive_controller）

启动:
  ros2 launch tracer_jaka_mujoco bridge.launch.py
  ros2 launch tracer_jaka_mujoco bridge.launch.py rviz:=false viewer:=true

控制:
  底盘:  ros2 run teleop_twist_keyboard teleop_twist_keyboard            # 发 /cmd_vel
        ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear:{x:0.2}, angular:{z:0.3}}"
  机械臂(forward): ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray "{data:[0,0.5,1.0,0,0.5,0]}"
  机械臂(轨迹):    向 /arm_controller/joint_trajectory 发 trajectory_msgs/JointTrajectory
  机械臂(MoveIt):  通过 action /arm_controller/follow_joint_trajectory

要点:
  * 桥接节点自身 use_sim_time=false（它是 /clock 的产生者）。
  * 其它所有节点 use_sim_time=true，消费桥接节点发布的 /clock。
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
    model = os.path.join(share, "models", "tracer_jaka_zu5.xml")

    with open(urdf, "r") as f:
        robot_description = {"robot_description": f.read()}

    use_rviz = LaunchConfiguration("rviz")
    use_viewer = LaunchConfiguration("viewer")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="true",
                              description="是否打开 MuJoCo 自带可视化窗口"),

        # 桥接节点 —— 时间源，自身不用 sim_time
        Node(package=pkg, executable="mujoco_bridge", output="screen",
             parameters=[{
                 "model_path": model,
                 "use_sim_time": False,
                 "exact_base": True,          # 关键：消除底盘速度偏差
                 "wheel_separation": 0.34,
                 "wheel_radius": 0.065,
                 "publish_rate": 100.0,
                 "clock_rate": 250.0,
                 "odom_frame": "odom",
                 "base_frame": "base_footprint",
                 "publish_odom_tf": True,
                 "use_viewer": use_viewer,
             }]),

        # robot_state_publisher：消费 /clock + /joint_states
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[robot_description, {"use_sim_time": True}]),

        # Node(package="rviz2", executable="rviz2", name="rviz2",
        #      arguments=["-d", rviz], output="screen",
        #      parameters=[{"use_sim_time": True}],
        #      condition=IfCondition(use_rviz)),
    ])

