#!/usr/bin/env python3
# =============================================================================
# tracer_jaka_gazebo - Main bringup launch (ROS 2 Jazzy + Gazebo Harmonic)
# =============================================================================
#   流程：
#     1. 启动 Gazebo Harmonic (gz sim) + 加载 world
#     2. 解析 xacro -> /robot_description
#     3. robot_state_publisher 发布 TF
#     4. ros_gz_sim::create 把机器人 spawn 进 Gazebo
#     5. ros_gz_bridge 桥接 /clock 等 Gazebo 原生 topic
#     6. spawner 依次启动 controllers
#     7. （可选）启动 RViz2
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = 'tracer_jaka_gazebo'
    pkg_share = FindPackageShare(pkg_name)

    # ------------- launch 参数 -------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file   = LaunchConfiguration('world')
    rviz_cfg     = LaunchConfiguration('rviz_config')
    use_rviz     = LaunchConfiguration('use_rviz')
    x_pose       = LaunchConfiguration('x')
    y_pose       = LaunchConfiguration('y')
    z_pose       = LaunchConfiguration('z')
    yaw          = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world',
            default_value=PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])),
        DeclareLaunchArgument('rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'rviz', 'mobile_manipulator.rviz'])),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('x',   default_value='0.0'),
        DeclareLaunchArgument('y',   default_value='0.0'),
        DeclareLaunchArgument('z',   default_value='0.20'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # ------------- 1. 解析 xacro -------------
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'tracer_jaka.urdf.xacro'])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'sim_mode:=true',
    ])

    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time,
    }

    # ------------- 2. robot_state_publisher -------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ------------- 3. 启动 Gazebo Harmonic -------------
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ------------- 4. spawn 机器人 -------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',   'tracer_jaka',
            '-topic',  '/robot_description',
            '-x',      x_pose,
            '-y',      y_pose,
            '-z',      z_pose,
            '-Y',      yaw,
        ],
    )

    # ------------- 5. ros_gz_bridge：桥接 /clock -------------
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_share, 'config', 'gz_bridge.yaml']),
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------- 6. 控制器 spawner（顺序启动） -------------
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jaka_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 控制器有依赖关系：spawn 完机器人后等几秒再加载控制器
    delay_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[jsb_spawner])],
        )
    )
    delay_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[diff_drive_spawner])
    )
    delay_arm_after_diff = RegisterEventHandler(
        OnProcessExit(target_action=diff_drive_spawner, on_exit=[arm_traj_spawner])
    )

    # ------------- 7. RViz2（可选） -------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=None,  # 通过 use_rviz 控制
    )

    return LaunchDescription(declare_args + [
        rsp_node,
        gz_launch,
        bridge_node,
        spawn_entity,
        delay_jsb_after_spawn,
        delay_diff_after_jsb,
        delay_arm_after_diff,
        # rviz 延后 4s 启动，等 TF 就绪
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
