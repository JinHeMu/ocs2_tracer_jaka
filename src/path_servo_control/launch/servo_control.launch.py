import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件的绝对路径
    config_file = os.path.join(
        get_package_share_directory('path_servo_control'),
        'config',
        'params.yaml'
    )

    # 定义节点并加载参数
    servo_node = Node(
        package='path_servo_control',
        executable='path_to_servo_controller',  # setup.py 中定义的入口点名称
        name='path_to_servo_controller',        # 节点名称，需与 YAML 顶层名称一致
        output='screen',
        parameters=[config_file]                # 传入参数文件
    )

    return LaunchDescription([
        servo_node
    ])
