import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz')

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('jaka_tracer_ocs2'),
        'resource',
        'tracer_jaka',
        'urdf',
        'tracer_jaka_zu5_demo.urdf.xacro'
    ])

    # 如果你暂时还是纯 .urdf，也可以改成：
    # urdf_file = PathJoinSubstitution([... 'tracer_jaka_zu5_demo.urdf'])
    # robot_description = {'robot_description': Command(['cat ', urdf_file])}

    robot_description = {
        'robot_description': Command(['xacro ', urdf_xacro])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracer_jaka_zu5',
            '-topic', 'robot_description',
            '-z', '0.20'
        ],
        output='screen'
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'jaka_arm_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # OCS2 files
    task_file = PathJoinSubstitution([
        FindPackageShare('jaka_tracer_ocs2'),
        'config',
        'tracer_jaka',
        'task.info'
    ])

    urdf_file_for_ocs2 = PathJoinSubstitution([
        FindPackageShare('jaka_tracer_ocs2'),
        'resource',
        'tracer_jaka',
        'urdf',
        'tracer_jaka_zu5_demo.urdf'
    ])

    ocs2_mpc_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_mpc_node',
        name='mobile_manipulator_mpc',
        output='screen',
        parameters=[
            {
                'taskFile': task_file,
                'urdfFile': urdf_file_for_ocs2,
                'libFolder': '/tmp/ocs2_mobile_manipulator_auto_generated/tracer_jaka'
            }
        ]
    )

    # 你的桥接节点：接 OCS2 输出，然后发给 /cmd_vel 和 /jaka_arm_controller/joint_trajectory
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('jaka_tracer_ocs2'),
        'config',
        'bridge_params.yaml'
    ])

    bridge_node = Node(
        package='jaka_tracer_ocs2',
        executable='jaka_tracer_ocs2_node',
        name='jaka_tracer_ocs2_bridge',
        output='screen',
        parameters=[bridge_yaml],
    )

    target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        output='screen',
        parameters=[
            {'taskFile': task_file}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('jaka_tracer_ocs2'),
                'config',
                'view.rviz'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        gazebo,
        robot_state_publisher,
        spawn_robot,

        TimerAction(period=3.0, actions=[
            spawn_jsb,
            spawn_arm_controller,
            spawn_diff_drive,
        ]),

        TimerAction(period=5.0, actions=[
            ocs2_mpc_node,
        ]),

        TimerAction(period=7.0, actions=[
            bridge_node,
        ]),

        TimerAction(period=9.0, actions=[
            target_node,
            rviz_node,
        ]),
    ])
