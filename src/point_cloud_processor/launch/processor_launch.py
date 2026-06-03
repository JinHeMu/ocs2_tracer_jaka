"""
full_system_launch.py
======================
一键启动完整系统：
  1. point_cloud_processor_node  — 处理 + 规划
  2. pcd_saver_node               — 点云实时采集
  3. coverage_visualizer          — 轨迹与点云可视化
  4. rviz2（可选）

建议工作流
----------
  步骤 1 — 启动系统（含 RViz2）
    ros2 launch point_cloud_processor full_system_launch.py rviz:=true

  步骤 2 — 开始采集点云
    ros2 service call /pcd_saver/set_saving \
      point_cloud_processor/srv/SetSaving \
      "{enable: true, save_dir: 'data', max_frames: 0}"

  步骤 3 — 裁剪
    ros2 service call /crop_point_cloud \
      point_cloud_processor/srv/CropPointCloud \
      "{input_pcd_path: 'data/frame_00000.pcd', output_pcd_path: 'data/cropped_cloud.pcd',
        min_bound: [0.1,-1.0,0.5], max_bound: [0.5,2.0,0.8]}"

  步骤 4 — 预处理 + 重建（等待完成）
    ros2 action send_goal /process_point_cloud \
      point_cloud_processor/action/ProcessPointCloud \
      "{input_pcd_path: '', output_ply_path: ''}" --feedback

  步骤 5 — 路径规划（等待完成）
    ros2 action send_goal /plan_coverage_path \
      point_cloud_processor/action/PlanCoveragePath \
      "{input_ply_path: '', output_csv_path: '', output_gcode_path: ''}" --feedback

  步骤 6 — 热重载可视化
    ros2 service call /coverage_visualizer/reload std_srvs/srv/Trigger
"""



from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('point_cloud_processor')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
            description='统一 YAML 参数文件'),
        DeclareLaunchArgument('auto_pipeline', default_value='false',
            description='启动后自动执行完整处理流水线'),
        DeclareLaunchArgument('auto_start_save', default_value='false',
            description='启动后立即开始采集点云'),
        DeclareLaunchArgument('rviz',       default_value='false',
            description='是否打开 RViz2'),
        DeclareLaunchArgument('log_level',  default_value='INFO',
            description='日志级别'),
    ]

    common_params = [LaunchConfiguration('params_file')]
    common_args   = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    # ── 1. 处理 + 规划节点 ────────────────────────────────────
    processor_node = Node(
        package='point_cloud_processor',
        executable='point_cloud_processor_node',
        name='point_cloud_processor',
        output='screen',
        parameters=common_params + [
            {'node.auto_pipeline': LaunchConfiguration('auto_pipeline')}
        ],
        arguments=common_args,
        emulate_tty=True,
    )

    # ── 2. 点云采集节点 ───────────────────────────────────────
    saver_node = Node(
        package='point_cloud_processor',
        executable='pcd_saver_node',
        name='pcd_saver',
        output='screen',
        parameters=common_params + [
            {'saver.auto_start': LaunchConfiguration('auto_start_save')}
        ],
        arguments=common_args,
        emulate_tty=True,
    )

    # ── 3. 可视化节点 ─────────────────────────────────────────
    visualizer_node = Node(
        package='point_cloud_processor',
        executable='visualizer_node',
        name='coverage_visualizer',
        output='screen',
        parameters=common_params,
        arguments=common_args,
        emulate_tty=True,
    )



    return LaunchDescription([
        *args,
        LogInfo(msg="=" * 55),
        LogInfo(msg=">>> 点云全系统启动：采集 / 处理 / 规划 / 可视化 <<<"),
        LogInfo(msg="=" * 55),
        processor_node,
        saver_node,
        visualizer_node,
    ])