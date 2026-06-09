import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
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
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
    arguments=["--ros-args", "--log-level", "info"],
    )   

    # Get parameters for the Servo node
    servo_yaml = load_yaml("moveit_servo", "config/jaka_real_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
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


    # ros2_control using FakeSystem as hardware
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

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            move_group_node,

            ros2_control_node,
            joint_state_broadcaster_spawner,
            jaka_fts_broadcaster_spawner, 
            #jaka_admittance_controller_spawner,
            #delay_jtc_after_admittance,
            jaka_arm_controller_spawner,
            servo_node,
            container,
        ]
    )
