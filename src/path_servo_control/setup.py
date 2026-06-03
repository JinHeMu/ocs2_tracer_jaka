import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'path_servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ⬇️ 新增：安装 launch 文件夹中的所有 .py 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # ⬇️ 新增：安装 config 文件夹中的所有 .yaml 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Path to servo velocity controller for robotic arms.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'path_to_servo_controller = path_servo_control.path_to_servo_controller:main'
        ],
    },
)
