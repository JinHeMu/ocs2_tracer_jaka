import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def _ensure_urdf(context, *args, **kwargs):
    """
    Synchronously convert tracer_jaka xacro to a temporary URDF file for OCS2.
    OCS2 usually expects a concrete .urdf file, not a .xacro file.
    """
    xacro_file = context.perform_substitution(LaunchConfiguration('xacro_file'))
    urdf_file = context.perform_substitution(LaunchConfiguration('urdfFile'))
    sim_mode = context.perform_substitution(LaunchConfiguration('sim_mode'))

    os.makedirs(os.path.dirname(urdf_file), exist_ok=True)

    cmd = [
        'xacro',
        xacro_file,
        f'sim_mode:={sim_mode}',
        '-o',
        urdf_file,
    ]

    print('[ocs2_tracer_jaka] Generate URDF:')
    print(' '.join(cmd))

    subprocess.check_call(cmd)

    return []


def generate_launch_description():
    tracer_jaka_pkg = get_package_share_directory('tracer_jaka_gazebo')
    ocs2_mobile_manipulator_ros_pkg = get_package_share_directory(
        'ocs2_mobile_manipulator_ros'
    )

    default_xacro_file = os.path.join(
        tracer_jaka_pkg,
        'urdf',
        'tracer_jaka.urdf.xacro'
    )

    default_urdf_file = os.path.join(
        '/tmp',
        'ocs2_tracer_jaka',
        'generated',
        'tracer_jaka.urdf'
    )

    # 你需要自己准备这个 task.info
    default_task_file = os.path.join(
        tracer_jaka_pkg,
        'config',
        'tracer_jaka',
        'task.info'
    )

    default_lib_folder = os.path.join(
        '/tmp',
        'ocs2_tracer_jaka_auto_generated',
        'tracer_jaka'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz.'
        ),

        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Run OCS2 nodes in debug mode.'
        ),

        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Whether to generate URDF for simulation mode.'
        ),

        DeclareLaunchArgument(
            'xacro_file',
            default_value=default_xacro_file,
            description='Input tracer_jaka xacro file.'
        ),

        DeclareLaunchArgument(
            'urdfFile',
            default_value=default_urdf_file,
            description='Generated URDF file used by OCS2.'
        ),

        DeclareLaunchArgument(
            'taskFile',
            default_value=default_task_file,
            description='OCS2 task.info file for tracer_jaka.'
        ),

        DeclareLaunchArgument(
            'libFolder',
            default_value=default_lib_folder,
            description='Folder for OCS2 auto-generated code.'
        ),

        # 必须在 include OCS2 启动文件前生成 URDF
        OpaqueFunction(function=_ensure_urdf),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    ocs2_mobile_manipulator_ros_pkg,
                    'launch',
                    'include',
                    'mobile_manipulator.launch.py'
                )
            ),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'debug': LaunchConfiguration('debug'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'taskFile': LaunchConfiguration('taskFile'),
                'libFolder': LaunchConfiguration('libFolder'),
            }.items()
        ),
    ])
