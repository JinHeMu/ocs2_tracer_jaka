from setuptools import setup
from glob import glob
import os

package_name = "tracer_jaka_mujoco"


def collect_data_files(src_dir, install_dir):
    """
    递归收集 src_dir 下的所有普通文件，
    并保持目录结构安装到 install_dir 下。
    """
    data_files = []

    for root, dirs, files in os.walk(src_dir):
        file_paths = [
            os.path.join(root, f)
            for f in files
            if not f.endswith("~")
        ]

        if file_paths:
            rel_dir = os.path.relpath(root, src_dir)
            if rel_dir == ".":
                dst_dir = install_dir
            else:
                dst_dir = os.path.join(install_dir, rel_dir)

            data_files.append((dst_dir, file_paths))

    return data_files


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    (
        os.path.join("share", package_name),
        ["package.xml"],
    ),
    (
        os.path.join("share", package_name, "launch"),
        glob("launch/*.py"),
    ),
    (
        os.path.join("share", package_name, "config"),
        [f for f in glob("config/*") if os.path.isfile(f)],
    ),
    (
        os.path.join("share", package_name, "rviz"),
        [f for f in glob("rviz/*") if os.path.isfile(f)],
    ),
]

# 安装 urdf 目录下的文件，但不把目录本身当成文件复制
data_files += collect_data_files(
    "urdf",
    os.path.join("share", package_name, "urdf"),
)

# 安装 models 目录，包括 xml 和 meshes 子目录，保留完整层级
data_files += collect_data_files(
    "models",
    os.path.join("share", package_name, "models"),
)


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Tracer + JAKA Zu5 移动机械臂的 MuJoCo 仿真与 ROS2 控制",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_bridge = tracer_jaka_mujoco.mujoco_bridge_node:main",
        ],
    },
)
