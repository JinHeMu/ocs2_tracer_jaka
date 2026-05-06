#!/usr/bin/env python3
# =============================================================================
#  ocs2_sim.launch.py
#
#  完整集成 launch :
#    1. 调用 xacro 把 tracer_jaka.urdf.xacro 编译成 /tmp/.../tracer_jaka.urdf
#       (OCS2 需要 URDF 文件路径, 不能直接吃 xacro)
#    2. 启动 tracer_jaka_gazebo/gazebo.launch.py (Gazebo + 控制器)
#    3. 等控制器就绪后, 启动 OCS2 三件套:
#         - tracer_jaka_mpc_node    (求解器)
#         - tracer_jaka_mrt_node    (Gazebo 桥)
#         - tracer_jaka_target_node (目标位姿发布)
#    4. RViz2
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ensure_urdf(context, *args, **kwargs):
    """运行 xacro -> 写入临时 .urdf 给 OCS2 用"""
    xacro_file = context.perform_substitution(LaunchConfiguration('xacro_file'))
    urdf_out   = context.perform_substitution(LaunchConfiguration('urdf_file'))
    os.makedirs(os.path.dirname(urdf_out), exist_ok=True)
    return [
        ExecuteProcess(
            cmd=['xacro', xacro_file, 'sim_mode:=false', '-o', urdf_out],
            output='screen',
            shell=False,
        )
    ]


def generate_launch_description():
    pkg_ocs2   = FindPackageShare('tracer_jaka_ocs2')
    pkg_gazebo = FindPackageShare('tracer_jaka_gazebo')

    # --------- 参数 ---------
    use_sim_time = LaunchConfiguration('use_sim_time')
    task_file    = LaunchConfiguration('task_file')
    xacro_file   = LaunchConfiguration('xacro_file')
    urdf_file    = LaunchConfiguration('urdf_file')
    lib_folder   = LaunchConfiguration('lib_folder')
    use_rviz     = LaunchConfiguration('use_rviz')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument(
            'task_file',
            default_value=PathJoinSubstitution([pkg_ocs2, 'config', 'task.info'])),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=PathJoinSubstitution([pkg_gazebo, 'urdf', 'tracer_jaka.urdf.xacro'])),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='/tmp/ocs2_tracer_jaka/tracer_jaka.urdf'),
        DeclareLaunchArgument(
            'lib_folder',
            default_value='/tmp/ocs2_tracer_jaka/auto_generated'),
    ]

    # --------- 步骤 1: 把 xacro 转成 urdf 落盘 ---------
    convert_urdf = OpaqueFunction(function=_ensure_urdf)

    # --------- 步骤 2: 拉起原 gazebo.launch.py ---------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz':     'false',     # 这里我们用自己的 rviz
        }.items(),
    )

    # --------- 步骤 3: OCS2 三件套 ---------
    mpc_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mpc_node',
        name='tracer_jaka_mpc_node',
        output='screen',
        parameters=[{
            'taskFile':     task_file,
            'urdfFile':     urdf_file,
            'libFolder':    lib_folder,
            'use_sim_time': use_sim_time,
        }],
    )

    mrt_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mrt_node',
        name='tracer_jaka_mrt_node',
        output='screen',
        parameters=[{
            'taskFile':          task_file,
            'urdfFile':          urdf_file,
            'libFolder':         lib_folder,
            'mrt_loop_rate':     100.0,
            'traj_horizon':      0.1,
            'traj_num_points':   5,
            'odom_topic':        '/diff_drive_controller/odom',
            'joint_state_topic': '/joint_states',
            'base_cmd_topic':    '/diff_drive_controller/cmd_vel',
            'arm_cmd_topic':     '/jaka_arm_controller/joint_trajectory',
            'arm_joint_names':   ['joint_1', 'joint_2', 'joint_3',
                                  'joint_4', 'joint_5', 'joint_6'],
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
            'robot_name':         'mobile_manipulator',
            'target_pose_topic':  '/target_pose',
            'use_sim_time':       use_sim_time,
        }],
    )

    # --------- 步骤 4: RViz2 ---------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [pkg_ocs2, 'rviz', 'tracer_jaka_ocs2.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # --------- 时序 ---------
    # OCS2 编译 codegen 比较慢, 控制器也需要 ~5s 才能起来,
    # 所以延迟启动, 等 odom / joint_states 流出来后再起 OCS2
    ocs2_delayed = TimerAction(period=8.0,
                               actions=[mpc_node, mrt_node, target_node])
    rviz_delayed = TimerAction(period=4.0, actions=[rviz_node])

    return LaunchDescription(
        declare_args + [
            convert_urdf,
            gazebo_launch,
            rviz_delayed,
            ocs2_delayed,
        ]
    )
