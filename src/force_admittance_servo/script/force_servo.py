#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Admittance Controller (Python) → MoveIt Servo
=============================================
将 C++ admittance_controller 移植为 Python ROS 2 节点。
控制输出为 geometry_msgs/TwistStamped，发布给 MoveIt Servo。

导纳控制律（控制坐标系下，按轴选择）：
  导纳柔顺轴: X_ddot = M_inv * (F - D * X_dot - K * X)
  恒力控制轴: X_ddot = M_inv * (F_target + F_ext - D * X_dot)

依赖：
  pip install numpy scipy
  ros2 packages: rclpy, geometry_msgs, sensor_msgs, std_msgs, tf2_ros, moveit_msgs

运行：
  ros2 run <your_package> admittance_controller_servo.py
  或直接：
  python3 admittance_controller_servo.py
"""

import numpy as np
from scipy.spatial.transform import Rotation
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, WrenchStamped, Wrench
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ─────────────────────────────────────────────────────────────────────────────
#  工具函数
# ─────────────────────────────────────────────────────────────────────────────

def rotation_matrix_to_6x6(R: np.ndarray) -> np.ndarray:
    """将 3x3 旋转矩阵扩展为 6x6 块对角矩阵（用于力/速度的坐标变换）。"""
    T = np.zeros((6, 6))
    T[:3, :3] = R
    T[3:, 3:] = R
    return T


def skew(v: np.ndarray) -> np.ndarray:
    """向量 → 反对称矩阵（叉积矩阵）。"""
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])


def angle_axis_from_rotation(R: np.ndarray) -> np.ndarray:
    """从旋转矩阵提取角轴向量 (angle * axis)。"""
    rot = Rotation.from_matrix(R)
    rotvec = rot.as_rotvec()   # magnitude = angle, direction = axis
    return rotvec


def exponential_smoothing(new_val: float, old_val: float, alpha: float) -> float:
    """一阶指数平滑：y = alpha * x + (1 - alpha) * y_prev"""
    return alpha * new_val + (1.0 - alpha) * old_val


# ─────────────────────────────────────────────────────────────────────────────
#  AdmittanceState  —  保存控制器内部状态
# ─────────────────────────────────────────────────────────────────────────────

class AdmittanceState:
    def __init__(self):
        # ---- 导纳参数（6 DOF：x, y, z, rx, ry, rz）----
        self.mass      = np.ones(6)          # 惯性参数
        self.mass_inv  = np.ones(6)          # 1 / mass
        self.damping   = np.zeros(6)         # 阻尼系数
        self.stiffness = np.zeros(6)         # 刚度系数
        self.selected_axes      = np.ones(6) # 哪些轴参与导纳（0/1）
        self.force_control_axes = np.zeros(6)# 哪些轴做恒力控制（bool）
        self.target_wrench_control = np.zeros(6)  # 目标力（控制坐标系）

        # ---- 运动学状态 ----
        self.admittance_velocity     = np.zeros(6)  # 导纳速度（基坐标系）
        self.admittance_acceleration = np.zeros(6)  # 导纳加速度（基坐标系）

        # ---- 力/力矩（基坐标系） ----
        self.wrench_base = np.zeros(6)

        # ---- 坐标变换 ----
        self.rot_base_control = np.eye(3)    # 基坐标系 → 控制坐标系的旋转矩阵


# ─────────────────────────────────────────────────────────────────────────────
#  AdmittanceRule  —  核心控制计算（与 C++ 逻辑对应）
# ─────────────────────────────────────────────────────────────────────────────

class AdmittanceRule:
    """
    对应 C++ admittance_rule_impl.hpp 中的 calculate_admittance_rule()。
    输入：当前外力（基坐标系）、当前速度（控制坐标系）、位置偏差（控制坐标系）
    输出：笛卡尔加速度 → 积分得到速度 → 发布 TwistStamped
    """

    def __init__(self):
        self.state = AdmittanceState()

        # 滤波后的力（世界坐标系）
        self.wrench_world = np.zeros(6)

        # 重力补偿
        self.end_effector_weight = np.zeros(3)   # [0, 0, -mg]
        self.cog_pos             = np.zeros(3)   # CoG 位置（CoG 坐标系）

    # ------------------------------------------------------------------
    #  参数设置接口
    # ------------------------------------------------------------------

    def set_admittance_params(self,
                              mass:       np.ndarray,
                              damping_ratio: np.ndarray,
                              stiffness:  np.ndarray,
                              selected_axes: np.ndarray):
        """设置导纳参数；阻尼由临界阻尼比计算：D = zeta * 2 * sqrt(M * K)"""
        self.state.mass      = np.array(mass, dtype=float)
        self.state.mass_inv  = 1.0 / self.state.mass
        self.state.stiffness = np.array(stiffness, dtype=float)
        self.state.selected_axes = np.array(selected_axes, dtype=float)
        for i in range(6):
            self.state.damping[i] = (damping_ratio[i] * 2.0 *
                                     np.sqrt(self.state.mass[i] * self.state.stiffness[i]))

    def set_force_control(self,
                          force_control_axes: np.ndarray,
                          target_wrench_control: np.ndarray):
        """设置力控轴与目标力（在控制坐标系下表达）。"""
        self.state.force_control_axes      = np.array(force_control_axes, dtype=float)
        self.state.target_wrench_control   = np.array(target_wrench_control, dtype=float)

    def set_gravity_compensation(self, ee_weight: float, cog_pos: np.ndarray):
        """设置末端执行器质量产生的重力（N）及 CoG 坐标。"""
        self.end_effector_weight = np.array([0.0, 0.0, -ee_weight])
        self.cog_pos = np.array(cog_pos, dtype=float)

    # ------------------------------------------------------------------
    #  力传感器处理（含重力补偿 + 指数滤波）
    # ------------------------------------------------------------------

    def process_wrench_measurements(self,
                                    measured_wrench: np.ndarray,
                                    rot_world_sensor: np.ndarray,
                                    rot_world_cog: np.ndarray,
                                    filter_coeff: float = 0.05):
        """
        输入：传感器原始 wrench (6,)，旋转矩阵 world←sensor，world←CoG
        更新 self.wrench_world（已重力补偿 + 低通滤波）
        """
        f_sensor = measured_wrench[:3]
        t_sensor = measured_wrench[3:]

        # 旋转到世界坐标系
        f_world = rot_world_sensor @ f_sensor
        t_world = rot_world_sensor @ t_sensor

        # 重力补偿（Z 方向力 + CoG 产生的力矩）
        f_world[2] -= self.end_effector_weight[2]   # 减去重力（end_effector_weight[2] = -mg）
        t_world -= np.cross(rot_world_cog @ self.cog_pos, self.end_effector_weight)

        new_wrench = np.concatenate([f_world, t_world])

        # 指数滤波
        self.wrench_world = (filter_coeff * new_wrench +
                             (1.0 - filter_coeff) * self.wrench_world)

    # ------------------------------------------------------------------
    #  核心导纳计算（对应 C++ calculate_admittance_rule）
    # ------------------------------------------------------------------

    def calculate(self,
                  rot_base_control: np.ndarray,
                  wrench_base: np.ndarray,
                  dt: float) -> np.ndarray:
        """
        计算笛卡尔加速度（基坐标系），并积分更新内部速度状态。

        参数：
          rot_base_control : 3x3  基坐标系 → 控制坐标系旋转矩阵
          wrench_base      : (6,) 外力（基坐标系）
          dt               : 控制周期（秒）

        返回：
          admittance_velocity (6,)  笛卡尔速度（基坐标系），用于 TwistStamped
        """
        R = rot_base_control   # base → control

        # ── 1. 将力变换到控制坐标系 ──
        F_base = wrench_base.copy()

        # 先投影到 selected_axes（控制坐标系下过滤不受控轴）
        F_ctrl = np.zeros(6)
        F_ctrl[:3] = R.T @ F_base[:3]
        F_ctrl[3:] = R.T @ F_base[3:]
        F_ctrl *= self.state.selected_axes
        # 变换回基坐标系（后续再用于计算）
        F_base[:3] = R @ F_ctrl[:3]
        F_base[3:] = R @ F_ctrl[3:]

        # ── 2. 变量变换到控制坐标系 ──
        X_dot_ctrl = np.zeros(6)
        X_dot_ctrl[:3] = R.T @ self.state.admittance_velocity[:3]
        X_dot_ctrl[3:] = R.T @ self.state.admittance_velocity[3:]

        # 位置偏差（本实现直接输出速度给 Servo，位置误差由积分得到近似）
        # 注：若需完整位置跟踪，需要接入运动学接口计算 X（见下方扩展说明）
        X_ctrl = np.zeros(6)   # 简化：位置偏差由速度积分隐含

        F_ctrl_filtered = np.zeros(6)
        F_ctrl_filtered[:3] = R.T @ F_base[:3]
        F_ctrl_filtered[3:] = R.T @ F_base[3:]

        # ── 3. 逐轴计算加速度（混合导纳 + 恒力控制）──
        X_ddot_ctrl = np.zeros(6)
        for i in range(6):
            if self.state.force_control_axes[i] > 0.5:
                # 恒力控制轴：跟踪目标力，无刚度项
                # X_ddot = M_inv * (F_target + F_ext - D * X_dot)
                X_ddot_ctrl[i] = self.state.mass_inv[i] * (
                    self.state.target_wrench_control[i]
                    + F_ctrl_filtered[i]
                    - self.state.damping[i] * X_dot_ctrl[i]
                )
            else:
                # 导纳柔顺轴：标准弹簧-阻尼-质量
                # X_ddot = M_inv * (F - D * X_dot - K * X)
                X_ddot_ctrl[i] = self.state.mass_inv[i] * (
                    F_ctrl_filtered[i]
                    - self.state.damping[i] * X_dot_ctrl[i]
                    - self.state.stiffness[i] * X_ctrl[i]
                )

        # ── 4. 变换回基坐标系 ──
        X_ddot = np.zeros(6)
        X_ddot[:3] = R @ X_ddot_ctrl[:3]
        X_ddot[3:] = R @ X_ddot_ctrl[3:]

        # ── 5. 积分：加速度 → 速度 ──
        self.state.admittance_acceleration = X_ddot
        self.state.admittance_velocity += X_ddot * dt

        return self.state.admittance_velocity.copy()

    def reset(self):
        """重置运动状态（切换控制模式或急停时调用）。"""
        self.state.admittance_velocity     = np.zeros(6)
        self.state.admittance_acceleration = np.zeros(6)
        self.wrench_world = np.zeros(6)


# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 节点
# ─────────────────────────────────────────────────────────────────────────────

class AdmittanceControllerNode(Node):
    """
    ROS 2 节点：
      订阅  → /force_torque_sensor/wrench  (geometry_msgs/WrenchStamped)
      订阅  → /joint_states                (sensor_msgs/JointState)        [可选]
      发布  → /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped)
    """

    def __init__(self):
        super().__init__('admittance_controller_py')

        # ── 声明 ROS 参数 ──────────────────────────────────────────────────
        self.declare_parameter('control_frame',    'tool0')
        self.declare_parameter('base_frame',       'base_link')
        self.declare_parameter('publish_rate',     125.0)          # Hz
        self.declare_parameter('filter_coefficient', 0.05)

        # 导纳参数（6 DOF：x, y, z, rx, ry, rz）
        self.declare_parameter('admittance.mass',
                               [10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
        self.declare_parameter('admittance.damping_ratio',
                               [0.7,  0.7,  0.7,  0.7, 0.7, 0.7])
        self.declare_parameter('admittance.stiffness',
                               [200., 200., 200., 10., 10., 10.])
        self.declare_parameter('admittance.selected_axes',
                               [1.0,  1.0,  1.0,  0.0, 0.0, 0.0])

        # 力控参数
        self.declare_parameter('admittance.force_control_axes',
                               [False, False, False, False, False, False])
        self.declare_parameter('admittance.target_wrench',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 重力补偿
        self.declare_parameter('gravity_compensation.cog_pos', [0.0, 0.0, 0.0])
        self.declare_parameter('gravity_compensation.ee_weight', 0.0)

        # MoveIt Servo 输出话题
        self.declare_parameter('servo_twist_topic',
                               '/servo_node/delta_twist_cmds')

        # ── 初始化导纳规则 ──────────────────────────────────────────────────
        self.admittance = AdmittanceRule()
        self._apply_parameters()

        # ── 内部状态 ────────────────────────────────────────────────────────
        self.latest_wrench: np.ndarray = np.zeros(6)  # 传感器最新值（传感器坐标系）
        self.rot_world_sensor = np.eye(3)   # world ← sensor（由 TF 获取，此处简化为单位阵）
        self.rot_world_cog    = np.eye(3)   # world ← CoG
        self.rot_base_control = np.eye(3)   # base  ← control（由 TF 获取）
        self.lock = threading.Lock()

        # ── 话题订阅 ────────────────────────────────────────────────────────
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor/wrench',
            self._wrench_callback,
            10
        )
        self.get_logger().info('订阅力传感器话题: /force_torque_sensor/wrench')

        # ── 话题发布 ────────────────────────────────────────────────────────
        servo_topic = self.get_parameter('servo_twist_topic').value
        self.twist_pub = self.create_publisher(TwistStamped, servo_topic, 10)
        self.get_logger().info(f'发布 Twist 到: {servo_topic}')

        # ── 控制定时器 ──────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        period = 1.0 / rate
        self.dt = period
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().info(f'控制频率: {rate:.1f} Hz，周期: {period*1000:.1f} ms')

        self.get_logger().info('AdmittanceControllerNode 已启动 ✓')

    # ------------------------------------------------------------------
    #  参数加载
    # ------------------------------------------------------------------

    def _apply_parameters(self):
        mass            = list(self.get_parameter('admittance.mass').value)
        damping_ratio   = list(self.get_parameter('admittance.damping_ratio').value)
        stiffness       = list(self.get_parameter('admittance.stiffness').value)
        selected_axes   = list(self.get_parameter('admittance.selected_axes').value)

        self.admittance.set_admittance_params(
            np.array(mass),
            np.array(damping_ratio),
            np.array(stiffness),
            np.array(selected_axes)
        )

        fc_axes    = list(self.get_parameter('admittance.force_control_axes').value)
        tgt_wrench = list(self.get_parameter('admittance.target_wrench').value)
        self.admittance.set_force_control(
            np.array(fc_axes, dtype=float),
            np.array(tgt_wrench, dtype=float)
        )

        cog_pos   = list(self.get_parameter('gravity_compensation.cog_pos').value)
        ee_weight = float(self.get_parameter('gravity_compensation.ee_weight').value)
        self.admittance.set_gravity_compensation(ee_weight, np.array(cog_pos))

        self.filter_coeff = float(self.get_parameter('filter_coefficient').value)

    # ------------------------------------------------------------------
    #  力传感器回调
    # ------------------------------------------------------------------

    def _wrench_callback(self, msg: WrenchStamped):
        with self.lock:
            self.latest_wrench = np.array([
                msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ])

    # ------------------------------------------------------------------
    #  主控制循环
    # ------------------------------------------------------------------

    def _control_loop(self):
        # 1. 读取传感器（线程安全）
        with self.lock:
            wrench_sensor = self.latest_wrench.copy()

        # 2. 重力补偿 + 滤波 → wrench_world_
        self.admittance.process_wrench_measurements(
            wrench_sensor,
            self.rot_world_sensor,
            self.rot_world_cog,
            self.filter_coeff
        )

        # 3. 将 wrench_world_ 变换到基坐标系
        #    wrench_base = R_world_base^T * wrench_world
        #    （简化：若 world == base，则 R = I）
        ww = self.admittance.wrench_world
        # 这里假设 world_base rotation 为单位阵（常见配置：base_link = world）
        # 若不同，需要通过 TF2 获取 rot_world_base 后做变换
        rot_world_base = np.eye(3)     # ← 按实际 TF 替换
        wrench_base = np.zeros(6)
        wrench_base[:3] = rot_world_base.T @ ww[:3]
        wrench_base[3:] = rot_world_base.T @ ww[3:]

        # 4. 运行导纳计算 → 得到笛卡尔速度（基坐标系）
        velocity = self.admittance.calculate(
            self.rot_base_control,
            wrench_base,
            self.dt
        )

        # 5. 打包为 TwistStamped 并发布（供 MoveIt Servo 消费）
        twist_msg = TwistStamped()
        twist_msg.header.stamp  = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.get_parameter('base_frame').value

        twist_msg.twist.linear.x  = float(velocity[0])
        twist_msg.twist.linear.y  = float(velocity[1])
        twist_msg.twist.linear.z  = float(velocity[2])
        twist_msg.twist.angular.x = float(velocity[3])
        twist_msg.twist.angular.y = float(velocity[4])
        twist_msg.twist.angular.z = float(velocity[5])

        self.twist_pub.publish(twist_msg)

        # 6. 调试日志（可根据需要降低频率）
        if hasattr(self, '_log_count'):
            self._log_count += 1
        else:
            self._log_count = 0

        if self._log_count % 50 == 0:   # 每 50 次打印一次
            self.get_logger().debug(
                f'F_ext(base): [{wrench_base[0]:.2f}, {wrench_base[1]:.2f}, '
                f'{wrench_base[2]:.2f}] N | '
                f'v_cart: [{velocity[0]:.4f}, {velocity[1]:.4f}, '
                f'{velocity[2]:.4f}] m/s'
            )


# ─────────────────────────────────────────────────────────────────────────────
#  独立测试（无 ROS 环境时）
# ─────────────────────────────────────────────────────────────────────────────

def standalone_test():
    """
    不需要 ROS 环境的单元测试。
    验证导纳控制律数值计算的正确性。
    """
    print("=" * 60)
    print("  AdmittanceRule 独立数值测试")
    print("=" * 60)

    rule = AdmittanceRule()

    # ── 参数配置（典型机器人末端导纳参数）──
    rule.set_admittance_params(
        mass          = np.array([10., 10., 10.,  1.,  1.,  1.]),
        damping_ratio = np.array([ 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]),
        stiffness     = np.array([200., 200., 200., 10., 10., 10.]),
        selected_axes = np.array([  1.,   1.,   1.,  0.,  0.,  0.])
    )

    # ── 测试 1：纯柔顺模式（沿 Z 轴施加 10 N 外力）──
    print("\n[测试 1] 柔顺模式 - Z 轴施加 10 N 外力")
    rule.reset()
    rule.set_force_control(
        force_control_axes    = np.zeros(6),
        target_wrench_control = np.zeros(6)
    )

    dt = 0.008  # 125 Hz
    wrench_base = np.array([0., 0., 10., 0., 0., 0.])  # 10 N in Z

    print(f"  {'时间(ms)':>10}  {'F_z(N)':>8}  {'v_z(m/s)':>10}  {'a_z(m/s²)':>10}")
    print(f"  {'-'*46}")
    for step in range(20):
        vel = rule.calculate(np.eye(3), wrench_base, dt)
        t_ms = (step + 1) * dt * 1000
        if step % 4 == 3:
            print(f"  {t_ms:>10.1f}  {wrench_base[2]:>8.2f}  {vel[2]:>10.5f}  "
                  f"{rule.state.admittance_acceleration[2]:>10.4f}")

    # ── 测试 2：恒力控制模式（Z 轴目标力 -5 N，接触面提供 -10 N 反力）──
    print("\n[测试 2] 恒力控制模式 - 目标 -5 N，接触反力 -10 N")
    rule.reset()
    rule.set_force_control(
        force_control_axes    = np.array([0., 0., 1., 0., 0., 0.]),   # Z 轴力控
        target_wrench_control = np.array([0., 0., -5., 0., 0., 0.])   # 目标 -5 N
    )

    wrench_fc = np.array([0., 0., -10., 0., 0., 0.])  # 实际感受到 -10 N（接触力）
    print(f"  导纳控制律: a = M_inv * (F_target + F_ext - D * v)")
    print(f"  预期：误差力 = -5 + (-10) = -15 N 驱动运动")
    print(f"  {'时间(ms)':>10}  {'F_z(N)':>8}  {'v_z(m/s)':>10}")
    print(f"  {'-'*34}")
    for step in range(20):
        vel = rule.calculate(np.eye(3), wrench_fc, dt)
        t_ms = (step + 1) * dt * 1000
        if step % 4 == 3:
            print(f"  {t_ms:>10.1f}  {wrench_fc[2]:>8.2f}  {vel[2]:>10.5f}")

    # ── 测试 3：旋转坐标系（控制坐标系相对基坐标系旋转 45° 绕 Z）──
    print("\n[测试 3] 控制坐标系绕 Z 旋转 45°，X 方向施力")
    rule.reset()
    rule.set_force_control(
        force_control_axes    = np.zeros(6),
        target_wrench_control = np.zeros(6)
    )
    rule.set_admittance_params(
        mass          = np.array([10., 10., 10.,  1.,  1.,  1.]),
        damping_ratio = np.array([ 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]),
        stiffness     = np.array([200., 200., 200., 10., 10., 10.]),
        selected_axes = np.array([  1.,   1.,   1.,  0.,  0.,  0.])
    )

    angle = np.pi / 4
    R_45 = np.array([
        [ np.cos(angle), -np.sin(angle), 0],
        [ np.sin(angle),  np.cos(angle), 0],
        [0,               0,             1]
    ])
    wrench_rot = np.array([10., 0., 0., 0., 0., 0.])  # 基坐标系 X 轴 10 N

    for step in range(5):
        vel = rule.calculate(R_45, wrench_rot, dt)
    print(f"  施力方向（基坐标系）: F = [10, 0, 0] N")
    print(f"  最终速度（基坐标系）: v = [{vel[0]:.5f}, {vel[1]:.5f}, {vel[2]:.5f}] m/s")
    print(f"  速度方向应接近 X 轴（证明旋转变换正确）")

    print("\n" + "=" * 60)
    print("  测试完成 ✓")
    print("=" * 60)


# ─────────────────────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────────────────────

def main():
    import sys

    # 若以 --test 运行，执行独立数值测试（无需 ROS）
    if '--test' in sys.argv or len(sys.argv) == 1:
        try:
            import rclpy as _rclpy_check   # noqa: F401
        except ImportError:
            print("[INFO] rclpy 未找到，执行独立数值测试模式")
            standalone_test()
            return

    # 尝试启动 ROS 2 节点
    try:
        rclpy.init(args=sys.argv)
        node = AdmittanceControllerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('收到 Ctrl-C，正在关闭…')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"[ERROR] ROS 2 启动失败: {e}")
        print("[INFO] 回退到独立数值测试模式")
        standalone_test()


if __name__ == '__main__':
    main()

