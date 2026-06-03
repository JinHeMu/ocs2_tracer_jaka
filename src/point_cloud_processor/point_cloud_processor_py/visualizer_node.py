#!/usr/bin/env python3
"""
visualizer_node.py
==================
ROS 2 节点：将规划结果（覆盖路径 CSV + PCD 点云）可视化发布到 RViz2。

发布的话题
----------
  /coverage_path         [nav_msgs/Path]          轨迹路径线
  /coverage_poses        [geometry_msgs/PoseArray] 6-DoF 末端位姿数组
  /coverage_normals      [visualization_msgs/Marker] 青色法向量段
  /coverage_directions   [visualization_msgs/Marker] 黄色前进方向段
  /filtered_cloud        [sensor_msgs/PointCloud2]   原始点云底图

服务（Service）
  ~/reload   [std_srvs/Trigger]  热重载 CSV 和 PCD（无需重启节点）

参数（config/params.yaml → visualizer 节）
------------------------------------------
  visualizer.csv_path             规划路径 CSV 文件
  visualizer.pcd_path             点云底图 PCD 文件
  visualizer.frame_id             坐标系 ID
  visualizer.publish_rate_hz      发布频率
  visualizer.downsample_step      CSV 轨迹点降采样步长（1 = 不降）
  visualizer.display_length_m     法向/方向向量显示长度
  visualizer.direction_scale      方向向量相对法向的比例 (0~1)
  visualizer.pcd_offset_x/y/z    点云平移偏移量
  visualizer.normal_line_width    法向线宽
  visualizer.direction_line_width 方向线宽
"""

import os
import csv
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R


class VisualizerNode(Node):

    def __init__(self):
        super().__init__('coverage_visualizer')
        self._cb_group = ReentrantCallbackGroup()
        self._lock     = threading.Lock()

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('visualizer.csv_path',
                               'outputs/coverage_path.csv')
        self.declare_parameter('visualizer.pcd_path',
                               'data/cropped_cloud.pcd')
        self.declare_parameter('visualizer.frame_id',       'world')
        self.declare_parameter('visualizer.publish_rate_hz', 1.0)
        self.declare_parameter('visualizer.downsample_step', 1)
        self.declare_parameter('visualizer.display_length_m', 0.02)
        self.declare_parameter('visualizer.direction_scale',  0.8)
        self.declare_parameter('visualizer.pcd_offset_x',    0.0)
        self.declare_parameter('visualizer.pcd_offset_y',    0.0)
        self.declare_parameter('visualizer.pcd_offset_z',    0.0)
        # y 轴偏移用于将点云与机器人坐标系对齐（对应原脚本的 offset_y=0.31）
        self.declare_parameter('visualizer.path_offset_y',   0.0)
        # 轨迹 y 轴偏移（对应原脚本 y + 0.4）
        self.declare_parameter('visualizer.normal_line_width',    0.0015)
        self.declare_parameter('visualizer.direction_line_width', 0.0015)

        # ── 发布者 ────────────────────────────────────────────
        qos = 10
        fid = self.get_parameter('visualizer.frame_id').value
        self._path_pub  = self.create_publisher(Path,      '/coverage_path',    qos)
        self._pose_pub  = self.create_publisher(PoseArray, '/coverage_poses',   qos)
        self._nrm_pub   = self.create_publisher(Marker,    '/coverage_normals', qos)
        self._dir_pub   = self.create_publisher(Marker,    '/coverage_directions', qos)
        self._pcd_pub   = self.create_publisher(PointCloud2, '/filtered_cloud', qos)

        # ── 服务：热重载 ──────────────────────────────────────
        self.create_service(Trigger, '~/reload',
                            self._handle_reload,
                            callback_group=self._cb_group)

        # ── 消息占位 ──────────────────────────────────────────
        self._frame_id  = fid
        self._path_msg  = None
        self._pose_msg  = None
        self._nrm_msg   = None
        self._dir_msg   = None
        self._pcd_msg   = None

        # ── 初次加载 ──────────────────────────────────────────
        self._load_all()

        # ── 定时发布 ──────────────────────────────────────────
        rate = self.get_parameter('visualizer.publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate, self._timer_cb,
                                        callback_group=self._cb_group)

        self.get_logger().info(
            "VisualizerNode 已启动\n"
            f"  CSV  : {self.get_parameter('visualizer.csv_path').value}\n"
            f"  PCD  : {self.get_parameter('visualizer.pcd_path').value}\n"
            f"  频率 : {rate} Hz\n"
            "  服务 : ~/reload（热重载数据）"
        )

    # ─────────────────────────────────────────────────────────
    #  数据加载
    # ─────────────────────────────────────────────────────────

    def _load_all(self):
        csv_path = self.get_parameter('visualizer.csv_path').value
        pcd_path = self.get_parameter('visualizer.pcd_path').value

        path_msg, pose_msg, nrm_msg, dir_msg = self._build_path_msgs(csv_path)
        pcd_msg = self._build_pcd_msg(pcd_path)

        with self._lock:
            self._path_msg = path_msg
            self._pose_msg = pose_msg
            self._nrm_msg  = nrm_msg
            self._dir_msg  = dir_msg
            self._pcd_msg  = pcd_msg

    def _build_path_msgs(self, csv_path: str):
        """解析 CSV，构建 Path / PoseArray / 两组 Marker"""
        frame_id   = self._frame_id
        step       = self.get_parameter('visualizer.downsample_step').value
        disp_len   = self.get_parameter('visualizer.display_length_m').value
        dir_scale  = self.get_parameter('visualizer.direction_scale').value
        nw         = self.get_parameter('visualizer.normal_line_width').value
        dw         = self.get_parameter('visualizer.direction_line_width').value
        y_offset   = self.get_parameter('visualizer.path_offset_y').value

        path_msg = Path()
        path_msg.header.frame_id = frame_id

        pose_msg = PoseArray()
        pose_msg.header.frame_id = frame_id

        # 法向 Marker（青色）
        nrm_msg = self._make_line_marker(
            frame_id, ns='normals', mid=0,
            r=0.0, g=1.0, b=1.0, line_width=nw
        )
        # 方向 Marker（黄色）
        dir_msg = self._make_line_marker(
            frame_id, ns='directions', mid=1,
            r=1.0, g=1.0, b=0.0, line_width=dw
        )

        if not os.path.isfile(csv_path):
            self.get_logger().warn(f"[Visualizer] CSV 不存在: {csv_path}")
            return path_msg, pose_msg, nrm_msg, dir_msg

        try:
            with open(csv_path, 'r') as f:
                rows = list(csv.DictReader(f))
        except Exception as e:
            self.get_logger().error(f"[Visualizer] 读取 CSV 失败: {e}")
            return path_msg, pose_msg, nrm_msg, dir_msg

        sampled = rows[::step]
        self.get_logger().info(
            f"[Visualizer] 加载 {len(rows)} 个路径点"
            f"（降采样步长 {step}，实际显示 {len(sampled)} 个）"
        )

        for row in sampled:
            x   = float(row['x'])
            y   = float(row['y']) + y_offset
            z   = float(row['z'])
            nx  = float(row['nx']); ny = float(row['ny']); nz = float(row['nz'])
            dx  = float(row['dx']); dy = float(row['dy']); dz = float(row['dz'])

            # Path 点
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

            # PoseArray（6-DoF）
            q = self._compute_6dof_quat(nx, ny, nz, dx, dy, dz)
            pm = Pose()
            pm.position.x = x; pm.position.y = y; pm.position.z = z
            pm.orientation.x = q[0]; pm.orientation.y = q[1]
            pm.orientation.z = q[2]; pm.orientation.w = q[3]
            pose_msg.poses.append(pm)

            p_start = Point(x=x, y=y, z=z)

            # 法向线段
            nrm_msg.points.extend([
                p_start,
                Point(x=x + nx * disp_len,
                      y=y + ny * disp_len,
                      z=z + nz * disp_len)
            ])

            # 方向线段
            dl = disp_len * dir_scale
            dir_msg.points.extend([
                p_start,
                Point(x=x + dx * dl,
                      y=y + dy * dl,
                      z=z + dz * dl)
            ])

        return path_msg, pose_msg, nrm_msg, dir_msg

    def _build_pcd_msg(self, pcd_path: str) -> PointCloud2:
        """用 Open3D 读取 PCD，应用偏移，转为 PointCloud2"""
        import open3d as o3d

        ox = self.get_parameter('visualizer.pcd_offset_x').value
        oy = self.get_parameter('visualizer.pcd_offset_y').value
        oz = self.get_parameter('visualizer.pcd_offset_z').value

        msg = PointCloud2()
        msg.header.frame_id = self._frame_id

        if not os.path.isfile(pcd_path):
            self.get_logger().warn(f"[Visualizer] PCD 不存在: {pcd_path}")
            return msg

        try:
            pcd = o3d.io.read_point_cloud(pcd_path)
            if pcd.is_empty():
                self.get_logger().warn(f"[Visualizer] PCD 为空: {pcd_path}")
                return msg

            tf = np.eye(4)
            tf[0, 3] = ox; tf[1, 3] = oy; tf[2, 3] = oz
            pcd.transform(tf)

            pts = np.asarray(pcd.points, dtype=np.float32)
            self.get_logger().info(
                f"[Visualizer] 加载点云 {len(pts)} 个点，"
                f"偏移 ({ox:.3f}, {oy:.3f}, {oz:.3f})"
            )

            msg.height      = 1
            msg.width       = len(pts)
            msg.fields      = [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step   = 12
            msg.row_step     = 12 * len(pts)
            msg.is_dense     = True
            msg.data         = pts.tobytes()
        except Exception as e:
            self.get_logger().error(f"[Visualizer] 加载 PCD 失败: {e}")

        return msg

    # ─────────────────────────────────────────────────────────
    #  服务回调：热重载
    # ─────────────────────────────────────────────────────────

    def _handle_reload(self, _req: Trigger.Request,
                       response: Trigger.Response):
        self.get_logger().info("[Visualizer] 热重载 CSV 和 PCD ...")
        try:
            self._load_all()
            response.success = True
            response.message = "重载成功"
        except Exception as e:
            response.success = False
            response.message = f"重载失败: {e}"
        return response

    # ─────────────────────────────────────────────────────────
    #  定时发布
    # ─────────────────────────────────────────────────────────

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        with self._lock:
            pm  = self._path_msg
            pom = self._pose_msg
            nm  = self._nrm_msg
            dm  = self._dir_msg
            pcm = self._pcd_msg

        if pm is not None:
            pm.header.stamp  = now
            pom.header.stamp = now
            nm.header.stamp  = now
            dm.header.stamp  = now
            self._path_pub.publish(pm)
            self._pose_pub.publish(pom)
            self._nrm_pub.publish(nm)
            self._dir_pub.publish(dm)

        if pcm is not None and len(pcm.data) > 0:
            pcm.header.stamp = now
            self._pcd_pub.publish(pcm)

    # ─────────────────────────────────────────────────────────
    #  工具方法
    # ─────────────────────────────────────────────────────────

    @staticmethod
    def _make_line_marker(frame_id, ns, mid, r, g, b, line_width) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.ns              = ns
        m.id              = mid
        m.type            = Marker.LINE_LIST
        m.action          = Marker.ADD
        m.scale.x         = line_width
        m.color.r         = float(r)
        m.color.g         = float(g)
        m.color.b         = float(b)
        m.color.a         = 1.0
        return m

    @staticmethod
    def _compute_6dof_quat(nx, ny, nz, dx, dy, dz) -> np.ndarray:
        """
        工具坐标系：Z 轴 = 表面法向，X 轴 = 轨迹前进方向。
        Returns: quaternion [x, y, z, w]
        """
        z_axis = np.array([nx, ny, nz], dtype=float)
        nz_len = np.linalg.norm(z_axis)
        z_axis = z_axis / nz_len if nz_len > 1e-6 else np.array([0., 0., 1.])

        x_init = np.array([dx, dy, dz], dtype=float)
        nx_len = np.linalg.norm(x_init)
        x_init = x_init / nx_len if nx_len > 1e-6 else np.array([1., 0., 0.])

        y_axis = np.cross(z_axis, x_init)
        ny_len = np.linalg.norm(y_axis)
        if ny_len < 1e-6:
            fb = np.array([0., 1., 0.]) if abs(z_axis[2]) < 0.9 \
                 else np.array([1., 0., 0.])
            y_axis = np.cross(z_axis, fb)
            y_axis /= np.linalg.norm(y_axis)
        else:
            y_axis /= ny_len

        x_axis = np.cross(y_axis, z_axis)
        rot    = np.column_stack((x_axis, y_axis, z_axis))
        return R.from_matrix(rot).as_quat()   # [x, y, z, w]


# ─────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
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

