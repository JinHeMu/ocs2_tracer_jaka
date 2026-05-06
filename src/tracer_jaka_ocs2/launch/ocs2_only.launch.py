#!/usr/bin/env python3
# =============================================================================
#  ocs2_only.launch.py
#
#  当 tracer_jaka_gazebo/gazebo.launch.py 已经在另一个 terminal 跑着时,
#  用这个 launch 单独把 OCS2 三件套拉起来 (不重启 Gazebo).
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ensure_urdf(context, *args, **kwargs):
    xacro_file = context.perform_substitution(LaunchConfiguration('xacro_file'))
    urdf_out   = context.perform_substitution(LaunchConfiguration('urdf_file'))
    os.makedirs(os.path.dirname(urdf_out), exist_ok=True)
    return [
        ExecuteProcess(
            cmd=['xacro', xacro_file, 'sim_mode:=false', '-o', urdf_out],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_ocs2   = FindPackageShare('tracer_jaka_ocs2')
    pkg_gazebo = FindPackageShare('tracer_jaka_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    task_file    = LaunchConfiguration('task_file')
    xacro_file   = LaunchConfiguration('xacro_file')
    urdf_file    = LaunchConfiguration('urdf_file')
    lib_folder   = LaunchConfiguration('lib_folder')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('task_file',
            default_value=PathJoinSubstitution([pkg_ocs2, 'config', 'task.info'])),
        DeclareLaunchArgument('xacro_file',
            default_value=PathJoinSubstitution([pkg_gazebo, 'urdf', 'tracer_jaka.urdf.xacro'])),
        DeclareLaunchArgument('urdf_file',
            default_value='/tmp/ocs2_tracer_jaka/tracer_jaka.urdf'),
        DeclareLaunchArgument('lib_folder',
            default_value='/tmp/ocs2_tracer_jaka/auto_generated'),
    ]

    convert_urdf = OpaqueFunction(function=_ensure_urdf)

    mpc_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mpc_node',
        name='tracer_jaka_mpc_node',
        output='screen',
        parameters=[{
            'taskFile': task_file, 'urdfFile': urdf_file, 'libFolder': lib_folder,
            'use_sim_time': use_sim_time,
        }],
    )

    mrt_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mrt_node',
        name='tracer_jaka_mrt_node',
        output='screen',
        parameters=[{
            'taskFile': task_file, 'urdfFile': urdf_file, 'libFolder': lib_folder,
            'mrt_loop_rate': 100.0,
            'traj_horizon': 0.1, 'traj_num_points': 5,
            'odom_topic':        '/diff_drive_controller/odom',
            'joint_state_topic': '/joint_states',
            'base_cmd_topic':    '/diff_drive_controller/cmd_vel',
            'arm_cmd_topic':     '/jaka_arm_controller/joint_trajectory',
            'arm_joint_names':   ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
            'base_frame':        'base_footprint',
            'use_sim_time':      use_sim_time,
        }],
    )

    target_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_target_node',
        name='tracer_jaka_target_node',
        output='screen',
        parameters=[{
            'robot_name':        'mobile_manipulator',
            'target_pose_topic': '/target_pose',
            'use_sim_time':      use_sim_time,
        }],
    )

    return LaunchDescription(declare_args + [
        convert_urdf,
        mpc_node,
        mrt_node,
        target_node,
    ])
