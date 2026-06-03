#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
import csv
import os

def visualize_trajectory_with_vectors(csv_file, pcd_file=None, vector_length=0.015):
    """
    可视化点云、轨迹、法向量和方向向量。
    - 绿色线 (Green): 机器人末端运动轨迹
    - 蓝色线 (Blue) : 法向量 (贴合表面的 Z 轴)
    - 红色线 (Red)  : 前进方向向量 (轨迹的切向)
    """
    geometries = []

    # ==========================================
    # 1. 加载并渲染点云/网格 (背景)
    # ==========================================
    if pcd_file and os.path.exists(pcd_file):
        print(f"加载点云/网格模型: {pcd_file}")
        # 如果是点云
        if pcd_file.endswith('.pcd'):
            pcd = o3d.io.read_point_cloud(pcd_file)
            pcd.paint_uniform_color([0.7, 0.7, 0.7])  # 浅灰色，避免喧宾夺主
            geometries.append(pcd)
        # 如果是网格
        elif pcd_file.endswith('.ply'):
            mesh = o3d.io.read_triangle_mesh(pcd_file)
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color([0.7, 0.7, 0.7])
            geometries.append(mesh)
    else:
        print("未提供点云/网格文件或文件不存在，仅显示轨迹。")

    # ==========================================
    # 2. 读取 CSV 轨迹数据
    # ==========================================
    print(f"读取轨迹文件: {csv_file}")
    positions, normals, directions = [], [], []
    
    if not os.path.exists(csv_file):
        print(f"错误: 找不到 CSV 文件 {csv_file}")
        return

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            positions.append([float(row['x']), float(row['y']), float(row['z'])])
            normals.append([float(row['nx']), float(row['ny']), float(row['nz'])])
            directions.append([float(row['dx']), float(row['dy']), float(row['dz'])])

    positions = np.array(positions)
    normals = np.array(normals)
    directions = np.array(directions)
    
    if len(positions) == 0:
        print("CSV 文件中没有轨迹点。")
        return

    # ==========================================
    # 3. 创建轨迹线 (连续的路径)
    # ==========================================
    trajectory_lines = o3d.geometry.LineSet()
    trajectory_lines.points = o3d.utility.Vector3dVector(positions)
    # 将相邻的两个点连接成线段
    lines = [[i, i + 1] for i in range(len(positions) - 1)]
    trajectory_lines.lines = o3d.utility.Vector2iVector(lines)
    # 绿色表示轨迹
    trajectory_lines.paint_uniform_color([0.0, 1.0, 0.0])  
    geometries.append(trajectory_lines)

    # ==========================================
    # 4. 创建法向量 (贴合表面的 Z 轴)
    # ==========================================
    normal_lines = o3d.geometry.LineSet()
    # 向量的终点 = 起点 + 方向 * 长度
    normal_endpoints = positions + normals * vector_length
    # 将起点和终点堆叠起来
    normal_pts = np.vstack((positions, normal_endpoints))
    # 起点索引是 i，终点索引是 i + len(positions)
    n_lines = [[i, i + len(positions)] for i in range(len(positions))]
    
    normal_lines.points = o3d.utility.Vector3dVector(normal_pts)
    normal_lines.lines = o3d.utility.Vector2iVector(n_lines)
    # 蓝色表示法向量
    normal_lines.paint_uniform_color([0.0, 0.0, 1.0])  
    geometries.append(normal_lines)

    # ==========================================
    # 5. 创建方向向量 (前进切向)
    # ==========================================
    direction_lines = o3d.geometry.LineSet()
    direction_endpoints = positions + directions * vector_length
    direction_pts = np.vstack((positions, direction_endpoints))
    d_lines = [[i, i + len(positions)] for i in range(len(positions))]
    
    direction_lines.points = o3d.utility.Vector3dVector(direction_pts)
    direction_lines.lines = o3d.utility.Vector2iVector(d_lines)
    # 红色表示前进方向
    direction_lines.paint_uniform_color([1.0, 0.0, 0.0])  
    geometries.append(direction_lines)

    # ==========================================
    # 6. 添加坐标系并显示
    # ==========================================
    # 在原点添加一个坐标系指示器 (RGB -> XYZ)
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    geometries.append(axis_frame)

    print("开始渲染可视化窗口...")
    print("- 绿色: 路径")
    print("- 蓝色: 法向 (Normal)")
    print("- 红色: 前进方向 (Direction)")
    
    # 开启可视化
    o3d.visualization.draw_geometries(geometries, window_name="Coverage Path Visualization",
                                      width=1280, height=720,
                                      point_show_normal=False)

if __name__ == "__main__":
    # 替换为你实际的输出路径
    CSV_PATH = '/home/ras/tracer_jaka/outputs/coverage_path.csv'
    
    # 你可以选择传入剪裁后的点云或者泊松重建后的网格
    # 如果不想显示点云，传入 None 即可
    PCD_PATH = '/home/ras/tracer_jaka/data/cropped.pcd' 
    
    visualize_trajectory_with_vectors(CSV_PATH, PCD_PATH, vector_length=0.015)
