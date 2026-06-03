#!/usr/bin/env python3
"""
pcd_saver_node.py
==================
ROS 2 节点：订阅 RGBD 相机点云话题，将带颜色的帧保存为 PCD 文件。

提供的接口
----------
服务（Service）
  ~/set_saving       [SetSaving]      开始 / 停止采集
  ~/get_status       [GetSaverStatus] 查询当前状态

订阅
----
  <topic>   [sensor_msgs/PointCloud2]  相机点云输入（可通过参数配置）

参数（config/params.yaml → saver 节）
--------------------------------------
  saver.topic_name          订阅话题
  saver.save_dir            保存目录
  saver.save_interval_sec   最小保存间隔（秒）
  saver.auto_start          启动后立即开始采集
  saver.max_frames          最大保存帧数（0 = 不限）
  saver.filename_prefix     文件名前缀
"""

import os
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from point_cloud_processor.srv import SetSaving, GetSaverStatus

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm   # PointCloud2 transform 支持
from rclpy.duration import Duration


class PcdSaverNode(Node):

    def __init__(self):
        super().__init__('pcd_saver')
        self._cb_group = ReentrantCallbackGroup()

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('saver.topic_name',
                               '/camera/camera/depth/color/points')
        self.declare_parameter('saver.save_dir',       'data')
        self.declare_parameter('saver.save_interval_sec', 0.5)
        self.declare_parameter('saver.auto_start',     False)
        self.declare_parameter('saver.max_frames',     0)
        self.declare_parameter('saver.filename_prefix', 'frame')

        # ── 内部状态 ──────────────────────────────────────────
        self._lock        = threading.Lock()
        self._is_saving   = False
        self._saved_count = 0
        self._max_frames  = self.get_parameter('saver.max_frames').value
        self._interval    = self.get_parameter('saver.save_interval_sec').value
        self._save_dir    = self.get_parameter('saver.save_dir').value
        self._prefix      = self.get_parameter('saver.filename_prefix').value
        self._topic       = self.get_parameter('saver.topic_name').value
        self._last_save_t = 0.0

        # ── 服务 ──────────────────────────────────────────────
        self.create_service(SetSaving,       '~/set_saving',
                            self._handle_set_saving,
                            callback_group=self._cb_group)
        self.create_service(GetSaverStatus,  '~/get_status',
                            self._handle_get_status,
                            callback_group=self._cb_group)

        # ── 订阅 ──────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2,
            self._topic,
            self._on_pointcloud,
            10,
            callback_group=self._cb_group
        )

        # ── 自动启动 ──────────────────────────────────────────
        if self.get_parameter('saver.auto_start').value:
            self._start_saving(self._save_dir, self._max_frames)

        self.get_logger().info(
            f"PcdSaverNode 已启动\n"
            f"  订阅话题 : {self._topic}\n"
            f"  保存目录 : {self._save_dir}\n"
            f"  服务     : ~/set_saving  ~/get_status"
        )

        # ── TF2 监听器 ─────────────────────────────────────────
        self.declare_parameter('saver.target_frame', 'world')   # 目标坐标系
        self._target_frame = self.get_parameter('saver.target_frame').value

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            f"  目标坐标系 : {self._target_frame}（点云将变换到此系下保存）"
        )


    # ─────────────────────────────────────────────────────────
    #  内部辅助
    # ─────────────────────────────────────────────────────────

    def _start_saving(self, save_dir: str, max_frames: int):
        with self._lock:
            self._save_dir    = save_dir or self.get_parameter('saver.save_dir').value
            self._max_frames  = max_frames
            self._saved_count = 0
            os.makedirs(self._save_dir, exist_ok=True)
            self._is_saving   = True
            self.get_logger().info(
                f"[Saver] ▶ 开始采集 → {self._save_dir}"
                + (f"，最多 {max_frames} 帧" if max_frames > 0 else "，不限帧数")
            )

    def _stop_saving(self):
        with self._lock:
            self._is_saving = False
            self.get_logger().info(
                f"[Saver] ⏹ 停止采集，共保存 {self._saved_count} 帧"
            )

    # ─────────────────────────────────────────────────────────
    #  点云回调
    # ─────────────────────────────────────────────────────────

    def _on_pointcloud(self, msg: PointCloud2):
        # 节流检查
        now = time.time()
        if (now - self._last_save_t) < self._interval:
            return

        with self._lock:
            if not self._is_saving:
                return
            if self._max_frames > 0 and self._saved_count >= self._max_frames:
                self._is_saving = False
                self.get_logger().info(
                    f"[Saver] 达到最大帧数 {self._max_frames}，自动停止"
                )
                return
            frame_idx = self._saved_count
            save_dir  = self._save_dir
            prefix    = self._prefix

        self._last_save_t = now

        # ── 1. TF 变换：将点云转到 world 坐标系 ────────────────
        try:
            # 等待变换可用（最多 0.3 s，避免阻塞）
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,              # 目标系：world
                msg.header.frame_id,             # 源系：camera_depth_optical_frame
                msg.header.stamp,
                timeout=Duration(seconds=0.3)
            )
            msg_world = tf2_sm.do_transform_cloud(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"[Saver] TF 变换失败，跳过本帧: {e}")
            return

        # ── 2. 解析变换后的 PointCloud2 ────────────────────────
        try:
            gen = pc2.read_points(msg_world,
                                field_names=("x", "y", "z", "rgb"),
                                skip_nans=True)
            points_data = np.array(list(gen))
        except Exception as e:
            self.get_logger().error(f"[Saver] 解析点云失败: {e}")
            return

        if points_data.shape[0] == 0:
            return

        # 兼容结构化数组 (ROS 2 Humble+) 和普通 2D 数组
        if points_data.ndim == 1 and points_data.dtype.names is not None:
            xyz       = np.column_stack((points_data['x'],
                                        points_data['y'],
                                        points_data['z']))
            rgb_float = points_data['rgb']
        else:
            xyz       = points_data[:, :3]
            rgb_float = points_data[:, 3]

        # 解包 RGB（与原逻辑相同）
        rgb_u32 = np.asarray(rgb_float, dtype=np.float32).view(np.uint32)
        r = np.bitwise_and(np.right_shift(rgb_u32, 16), 255)
        g = np.bitwise_and(np.right_shift(rgb_u32,  8), 255)
        b = np.bitwise_and(rgb_u32, 255)
        colors = np.vstack((r, g, b)).T / 255.0

        # ── 3. 保存（与原逻辑相同）──────────────────────────────
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors)

        filename = os.path.join(save_dir, f"{prefix}_{frame_idx:05d}.pcd")
        o3d.io.write_point_cloud(filename, pcd)

        with self._lock:
            self._saved_count += 1
            count = self._saved_count

        self.get_logger().info(
            f"[Saver] 已保存第 {count} 帧 → {filename}"
            f"（{len(xyz)} 个点，坐标系: {self._target_frame}）"
        )

    # ─────────────────────────────────────────────────────────
    #  服务回调
    # ─────────────────────────────────────────────────────────

    def _handle_set_saving(self,
                           request: SetSaving.Request,
                           response: SetSaving.Response):
        if request.enable:
            self._start_saving(request.save_dir, request.max_frames)
            response.success = True
            response.message = "开始采集"
        else:
            self._stop_saving()
            response.success = True
            response.message = "停止采集"

        with self._lock:
            response.saved_frames = self._saved_count
            response.save_dir     = self._save_dir
        return response

    def _handle_get_status(self,
                           _request: GetSaverStatus.Request,
                           response: GetSaverStatus.Response):
        with self._lock:
            response.is_saving         = self._is_saving
            response.saved_frames      = self._saved_count
            response.save_dir          = self._save_dir
            response.save_interval_sec = self._interval
            response.topic_name        = self._topic
        return response


# ─────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PcdSaverNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
