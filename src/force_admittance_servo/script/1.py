#!/usr/bin/env python3
# Copyright (c) 2024
# 将 admittance_controller 的恒力/导纳混合控制律移植为 Python ROS2 节点，
# 订阅六维力传感器，输出笛卡尔速度指令给 MoveIt Servo。

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, WrenchStamped


# ---------------------------------------------------------------------------
# 参数说明（与 admittance_controller YAML 字段对应）
# ---------------------------------------------------------------------------
# mass              : 6D 虚拟质量 [kg / (kg·m²)]
# damping_ratio     : 阻尼比（自动计算临界阻尼）
# stiffness         : 6D 刚度 [N/m 或 Nm/rad]（恒力轴置 0）
# selected_axes     : 哪些轴参与控制 (0/1)
# force_control_axes: 哪些轴用恒力控制 (0=导纳柔顺, 1=恒力跟踪)
# target_wrench     : 恒力轴的目标力/力矩（控制坐标系下，与传感器反力符号约定相同）
# joint_damping     : 关节阻尼附加项（此处在笛卡尔速度上等效施加）
# filter_coefficient: 力传感器指数平滑系数 [0,1]，越大越平滑
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    # --- 虚拟惯性 ---
    "mass":               [10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
    "damping_ratio":      [0.9,  0.9,  0.9,  0.9, 0.9, 0.9],
    "stiffness":          [0.0,  0.0,  200.0, 5.0, 5.0, 5.0],

    # --- 轴选择 ---
    "selected_axes":      [1, 1, 1, 1, 1, 1],   # 全部轴参与
    "force_control_axes": [0, 0, 1, 0, 0, 0],   # 仅 Z 轴做恒力控制

    # --- 恒力目标（控制坐标系下）---
    # 符号约定：机器人向 -Z 压紧工件，环境反力 +Z；
    # 平衡时 target + F_measured = 0，故 target = -5.0 N
    "target_wrench":      [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],

    # --- 传感器滤波 ---
    "filter_coefficient": 0.1,   # 指数平滑系数（越小越平滑，延迟越大）

    # --- 速度饱和 ---
    "max_linear_vel":     0.05,  # m/s
    "max_angular_vel":    0.2,   # rad/s

    # --- 控制周期 ---
    "control_hz":         100.0,

    # --- 坐标系 ---
    "command_frame_id":   "tool0",
}


class AdmittanceServoNode(Node):
    """
    实现与 admittance_controller::AdmittanceRule 等效的混合控制律：
    - 导纳柔顺轴：M·ẍ + D·ẋ + K·x = F_ext
    - 恒力控制轴：M·ẍ + D·ẋ      = F_target + F_ext
    积分得到笛卡尔速度后发布给 MoveIt Servo。
    """

    def __init__(self):
        super().__init__('admittance_servo_node')

        # ---- 声明 & 读取 ROS 参数 ----
        for key, val in DEFAULT_PARAMS.items():
            self.declare_parameter(key, val)

        p = DEFAULT_PARAMS   # 简写，实际使用时改为 self.get_parameter(key).value

        mass_vec       = np.array(p["mass"],               dtype=float)
        dr_vec         = np.array(p["damping_ratio"],      dtype=float)
        stiff_vec      = np.array(p["stiffness"],          dtype=float)
        self.sel_axes  = np.array(p["selected_axes"],      dtype=float)
        self.fc_axes   = np.array(p["force_control_axes"], dtype=float)
        self.F_target  = np.array(p["target_wrench"],      dtype=float)
        self.alpha     = float(p["filter_coefficient"])
        self.max_lin   = float(p["max_linear_vel"])
        self.max_ang   = float(p["max_angular_vel"])
        self.frame_id  = str(p["command_frame_id"])

        # 虚拟质量倒数
        self.mass_inv = 1.0 / mass_vec

        # 临界阻尼：D = ratio * 2 * sqrt(M * K)
        # 对恒力轴（K=0），使用纯阻尼：D = ratio * 2 * sqrt(M * 1)（避免除零）
        K_eff = np.where(stiff_vec > 0, stiff_vec, 1.0)
        self.D = dr_vec * 2.0 * np.sqrt(mass_vec * K_eff)
        self.K = stiff_vec

        self.get_logger().info(
            f"[Admittance] mass_inv={np.round(self.mass_inv,4)}\n"
            f"             D       ={np.round(self.D,4)}\n"
            f"             K       ={np.round(self.K,4)}\n"
            f"             fc_axes ={self.fc_axes}"
        )

        # ---- 内部状态 ----
        self.X_dot  = np.zeros(6)   # 导纳速度（基/控制坐标系）
        self.X      = np.zeros(6)   # 导纳位置偏差（仅导纳轴有意义）
        self.wrench_filtered = np.zeros(6)   # 滤波后的外力（控制坐标系）

        self.dt = 1.0 / float(p["control_hz"])

        # ---- ROS 接口 ----
        self.sub_wrench = self.create_subscription(
            WrenchStamped,
            '/jaka_fts_broadcaster/wrench',
            self._wrench_cb,
            10
        )
        self.pub_twist = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info("AdmittanceServoNode 已启动，等待力传感器数据……")

    # ------------------------------------------------------------------
    # 回调：力传感器数据接收 + 指数平滑滤波
    # ------------------------------------------------------------------
    def _wrench_cb(self, msg: WrenchStamped):
        """
        将传感器数据存入内部变量，并做指数平滑。
        这里假设传感器坐标系与控制坐标系对齐；
        若不对齐，需在此处加入旋转变换 R_ctrl_sensor。
        """
        raw = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ])
        # 指数平滑：y_k = alpha * x_k + (1 - alpha) * y_{k-1}
        self.wrench_filtered = (
            self.alpha * raw + (1.0 - self.alpha) * self.wrench_filtered
        )

    # ------------------------------------------------------------------
    # 主控制循环（100 Hz）
    # ------------------------------------------------------------------
    def _control_loop(self):
        dt = self.dt

        # 1. 只对已选轴施加力
        F = self.wrench_filtered * self.sel_axes

        # 2. 逐轴计算笛卡尔加速度（控制坐标系下对角运算，等效于 C++ 的逐轴循环）
        #
        #   恒力轴 (fc_axes == 1):
        #     ẍ_i = mass_inv_i * (F_target_i + F_i - D_i * ẋ_i)
        #     力误差 = F_target + F_measured；当二者抵消时加速度趋零
        #
        #   导纳柔顺轴 (fc_axes == 0):
        #     ẍ_i = mass_inv_i * (F_i - D_i * ẋ_i - K_i * x_i)
        #
        force_ctrl_acc = self.mass_inv * (
            self.F_target + F - self.D * self.X_dot
        )
        admit_ctrl_acc = self.mass_inv * (
            F - self.D * self.X_dot - self.K * self.X
        )

        # 按轴选择控制律
        X_ddot = np.where(self.fc_axes > 0.5, force_ctrl_acc, admit_ctrl_acc)

        # 3. 欧拉积分
        self.X_dot += X_ddot * dt
        self.X     += self.X_dot * dt

        # 4. 速度饱和
        lin_speed = np.linalg.norm(self.X_dot[:3])
        ang_speed = np.linalg.norm(self.X_dot[3:])
        if lin_speed > self.max_lin and lin_speed > 1e-9:
            self.X_dot[:3] *= self.max_lin / lin_speed
        if ang_speed > self.max_ang and ang_speed > 1e-9:
            self.X_dot[3:] *= self.max_ang / ang_speed

        # 5. 发布 TwistStamped 给 MoveIt Servo
        twist_msg = TwistStamped()
        twist_msg.header.stamp    = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id

        twist_msg.twist.linear.x  = float(self.X_dot[0])
        twist_msg.twist.linear.y  = float(self.X_dot[1])
        twist_msg.twist.linear.z  = float(self.X_dot[2])
        twist_msg.twist.angular.x = float(self.X_dot[3])
        twist_msg.twist.angular.y = float(self.X_dot[4])
        twist_msg.twist.angular.z = float(self.X_dot[5])

        self.pub_twist.publish(twist_msg)

        # 6. 调试日志（每 100 拍打印一次）
        if not hasattr(self, '_log_cnt'):
            self._log_cnt = 0
        self._log_cnt += 1
        if self._log_cnt % 100 == 0:
            self.get_logger().info(
                f"F_filtered={np.round(F,3)} | "
                f"X_dot={np.round(self.X_dot,4)} | "
                f"X_ddot={np.round(X_ddot,4)}"
            )


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

