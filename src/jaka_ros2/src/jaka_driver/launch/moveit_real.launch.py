import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition


def generate_launch_description():

    # -------------------------------
    # Launch arguments
    # -------------------------------
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz2"
    )

    declare_db = DeclareLaunchArgument(
        "db",
        default_value="false",
        description="Start MongoDB warehouse"
    )

    # -------------------------------
    # MoveIt config
    # -------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka",
            package_name="tracer_jaka_moveit_config",
        )
        .robot_description(
            file_path="config/tracer_jaka_zu5.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/tracer_jaka_zu5.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .to_moveit_configs()
    )

    # -------------------------------
    # Robot state publisher
    # -------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # -------------------------------
    # ros2_control node (REAL ROBOT)
    # -------------------------------
    ros2_controllers_path = os.path.join(
        get_package_share_directory("tracer_jaka_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # URDF/XACRO
            ros2_controllers_path,            # 控制器 YAML
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")  # 确保订阅 ROS2 Control 的 robot_description
        ],
        output="screen",
    )


    # -------------------------------
    # Controller spawners
    # -------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    jaka_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jaka_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    jaka_admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jaka_admittance_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    jaka_fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jaka_fts_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    #关键步骤：创建事件处理器
    # 当 admittance_spawner 进程退出时（spawner 成功发完指令就会退出），再启动 jtc_spawner
    delay_jtc_after_admittance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jaka_admittance_controller_spawner,
            on_exit=[jaka_arm_controller_spawner],
        )
    )
    # -------------------------------
    # Move group (MoveIt core)
    # -------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # -------------------------------
    # RViz
    # -------------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory("tracer_jaka_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        output="log",
    )

    # -------------------------------
    # Launch description
    # -------------------------------
    return LaunchDescription([
        declare_rviz,
        declare_db,

        robot_state_publisher,
        move_group_node,
        rviz_node,


        ros2_control_node,
        joint_state_broadcaster_spawner,
        
        jaka_fts_broadcaster_spawner, 
        #jaka_admittance_controller_spawner,
        #delay_jtc_after_admittance,
        jaka_arm_controller_spawner,
    ])