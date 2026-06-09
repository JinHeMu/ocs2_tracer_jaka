"""
visualizer_launch.py
=====================
仅启动可视化节点（VisualizerNode）和点云采集节点（PcdSaverNode）。
适合已有规划结果、仅需在 RViz2 中查看轨迹或实时采集点云的场景。

用法
----
# 默认参数
ros2 launch point_cloud_processor visualizer_launch.py

# 指定 CSV / PCD 路径（覆盖 params.yaml）
ros2 launch point_cloud_processor visualizer_launch.py \
    csv_path:=/abs/path/coverage_path.csv \
    pcd_path:=/abs/path/cropped_cloud.pcd

# 启动时自动开始采集
ros2 launch point_cloud_processor visualizer_launch.py \
    auto_start:=true save_dir:=saved_pcds

# 打开 RViz2（需已安装 rviz2）
ros2 launch point_cloud_processor visualizer_launch.py \
    rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('point_cloud_processor')

    # ── 参数声明 ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
            description='YAML 参数文件路径'),
        DeclareLaunchArgument(
            'csv_path',
            default_value='/home/a/tracer_jaka/outputs/coverage_path.csv',
            description='覆盖 visualizer.csv_path'
        ),
        DeclareLaunchArgument(
            'pcd_path',
            default_value='/home/a/tracer_jaka/data/cropped.pcd',
            description='覆盖 visualizer.pcd_path'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='saved_pcds',
            description='覆盖 saver.save_dir'
        ),


        DeclareLaunchArgument('auto_start', default_value='false',
            description='采集节点启动后是否立即开始保存'),

        DeclareLaunchArgument('rviz',      default_value='false',
            description='是否自动打开 RViz2'),
        DeclareLaunchArgument('log_level', default_value='INFO',
            description='日志级别'),
    ]

    # ── 可视化节点 ────────────────────────────────────────────
    vis_overrides = {}
    csv_val = LaunchConfiguration('csv_path')
    pcd_val = LaunchConfiguration('pcd_path')
    # 非空时才覆盖（空字符串视为"使用 YAML 默认值"）
    vis_overrides['visualizer.csv_path'] = csv_val
    vis_overrides['visualizer.pcd_path'] = pcd_val

    visualizer_node = Node(
        package='point_cloud_processor',
        executable='visualizer_node',
        name='coverage_visualizer',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), vis_overrides],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )

    # ── 点云采集节点 ──────────────────────────────────────────
    saver_overrides = {
        'saver.auto_start': LaunchConfiguration('auto_start'),
    }
    save_dir_val = LaunchConfiguration('save_dir')
    saver_overrides['saver.save_dir'] = save_dir_val

    saver_node = Node(
        package='point_cloud_processor',
        executable='pcd_saver_node',
        name='pcd_saver',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), saver_overrides],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )


    return LaunchDescription([
        *args,
        LogInfo(msg=">>> 启动 VisualizerNode + PcdSaverNode <<<"),
        visualizer_node,
        saver_node,
    ])
