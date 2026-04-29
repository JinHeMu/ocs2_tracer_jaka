# Real-robot bringup for Jaka ZU5 + Tracer with OCS2 mobile_manipulator MPC
#
# 启动顺序（建议）：
#   1. Tracer 底盘节点 → 提供 /odom + 接收 /cmd_vel
#   2. Jaka ros2_control(JTC + jaka_hardware_interface) → 提供 /joint_states +
#      接收 /jaka_arm_controller/joint_trajectory
#   3. OCS2 mobile_manipulator MPC 节点（不带 dummy）
#   4. jaka_tracer_ocs2_bridge_node
#   5. target publisher / RViz interactive marker

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------- arguments ----------
    declared = [
        DeclareLaunchArgument('robot_ip',  default_value='192.168.10.90',
                              description='Jaka controller IP'),
        DeclareLaunchArgument('local_ip',  default_value='192.168.10.100',
                              description='Onboard PC IP for EDG UDP'),
        DeclareLaunchArgument('can_port',  default_value='can0',
                              description='Tracer CAN bus'),
        DeclareLaunchArgument('use_rviz',  default_value='true'),
    ]

    # ---------- 1) Tracer base ----------
    tracer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tracer_base'),
                'launch', 'tracer_base.launch.py'])),
        launch_arguments={
            'port_name': LaunchConfiguration('can_port'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic_name': 'odom',
            'is_tracer_mini': 'false',
        }.items())

    # ---------- 2) Jaka ros2_control ----------
    # 假设你已有一个 jaka_bringup 包，里面 launch 文件里加载了 controller_manager
    # + jaka_hardware_interface + ros2_controllers.yaml
    jaka_controllers_yaml = PathJoinSubstitution([
        FindPackageShare('jaka_bringup'), 'config', 'ros2_controllers.yaml'])

    robot_description_xacro = PathJoinSubstitution([
        FindPackageShare('jaka_description'), 'urdf',
        'tracer_jaka_zu5.urdf.xacro'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description':
                # xacro will be expanded inside ros2_control_node
                {'value': '<robot_description_will_be_set_by_robot_state_publisher>'}},
            jaka_controllers_yaml,
            {'robot_ip':  LaunchConfiguration('robot_ip')},
            {'local_ip': LaunchConfiguration('local_ip')},
        ],
        output='screen',
        emulate_tty=True,
    )

    spawn_jsb = Node(package='controller_manager', executable='spawner',
                     arguments=['joint_state_broadcaster',
                                '--controller-manager', '/controller_manager'],
                     output='screen')

    spawn_arm = Node(package='controller_manager', executable='spawner',
                     arguments=['jaka_arm_controller',
                                '--controller-manager', '/controller_manager'],
                     output='screen')

    spawn_fts = Node(package='controller_manager', executable='spawner',
                     arguments=['jaka_fts_broadcaster',
                                '--controller-manager', '/controller_manager'],
                     output='screen')

    # ---------- 3) OCS2 mobile_manipulator MPC ----------
    # 直接复用 OCS2 自带的 MPC 节点（注意：这是 MPC，不是 dummy_mrt）
    # 在 legubiao/ocs2_ros2 仓库中，可执行文件名通常是 mobile_manipulator_mpc
    task_file = PathJoinSubstitution([
        FindPackageShare('ocs2_mobile_manipulator'),
        'config', 'tracer_jaka', 'task.info'])  # 你需要把 task.info 放到这里
    urdf_file = PathJoinSubstitution([
        FindPackageShare('ocs2_robotic_assets'),
        'resources', 'mobile_manipulator', 'tracer_jaka',
        'urdf', 'tracer_jaka_zu5.urdf'])

    ocs2_mpc_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_mpc',
        name='mobile_manipulator_mpc',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--ros-args',
            '-p', ['taskFile:=', task_file],
            '-p', ['urdfFile:=', urdf_file],
            '-p', 'libFolder:=/tmp/ocs2'
        ])

    # ---------- 4) Bridge ----------
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('jaka_tracer_ocs2'), 'config', 'bridge_params.yaml'])
    bridge_node = Node(
        package='jaka_tracer_ocs2',
        executable='jaka_tracer_ocs2_node',
        name='jaka_tracer_ocs2_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[bridge_yaml],
    )

    # ---------- 5) Target / RViz ----------
    # OCS2 自带的目标位姿发布节点 + RViz 交互式 marker
    target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        output='screen',
        arguments=['--ros-args', '-p', ['taskFile:=', task_file]])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=None,  # 用 IfCondition(LaunchConfiguration('use_rviz')) 替换
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('jaka_tracer_ocs2'), 'config', 'view.rviz'])])

    # ---------- 编排：MPC 早于 bridge，bridge 早于 target ----------
    delayed_spawners = TimerAction(period=2.0,
                                   actions=[spawn_jsb, spawn_arm, spawn_fts])
    delayed_mpc      = TimerAction(period=4.0,  actions=[ocs2_mpc_node])
    delayed_bridge   = TimerAction(period=6.0,  actions=[bridge_node])
    delayed_target   = TimerAction(period=8.0,  actions=[target_node, rviz_node])

    return LaunchDescription(declared + [
        tracer_launch,
        controller_manager,
        delayed_spawners,
        delayed_mpc,
        delayed_bridge,
        delayed_target,
    ])
