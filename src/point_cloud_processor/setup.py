from setuptools import setup, find_packages

package_name = 'point_cloud_processor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}',       ['package.xml']),
        (f'share/{package_name}/config', ['config/params.yaml']),
        (f'share/{package_name}/launch', ['launch/processor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 点云处理与全覆盖路径规划节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'point_cloud_processor_node = '
            'point_cloud_processor.node:main',
            'pcp_client = '
            'point_cloud_processor.cli_client:main',
        ],
    },
)
