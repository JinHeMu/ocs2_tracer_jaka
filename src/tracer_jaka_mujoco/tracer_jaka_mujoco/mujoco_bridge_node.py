#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mujoco_bridge_node.py  ——  ROS2 <-> MuJoCo 自包含桥接节点
（不依赖 mujoco_ros2_control / diff_drive_controller；底盘为 3 自由度平面关节）

底盘模型假设（与你的 XML 一致）:
    <body name="base_footprint">
        <joint name="base_x"   type="slide" axis="1 0 0"/>
        <joint name="base_y"   type="slide" axis="0 1 0"/>
        <joint name="base_yaw" type="hinge" axis="0 0 1"/>
    => 世界系 x/y 平移 + 绕 z 偏航，三个独立单自由度关节，机器人只在平面运动。

设计目标
--------
1. 本节点是仿真唯一的“执行者 + 时间源”：加载 MuJoCo、步进物理、发布 /clock。
2. 机械臂：兼容 ros2_control 的两种用法
     - JTC 风格: 订阅 /arm_controller/joint_trajectory，并提供 FollowJointTrajectory action（MoveIt）
     - Forward 风格: 订阅 /arm_controller/commands (Float64MultiArray)
3. 底盘：仅用一个 cmd_vel（Twist）控制，不用差速控制器。
     - 订阅 /cmd_vel 与 /base_controller/cmd_vel
     - exact_base=True（默认）: 把 cmd_vel 积分成 (x,y,yaw) 直接写回 base_x/base_y/base_yaw，
       纯运动学跟随，彻底消除速度伺服 + 编码里程带来的稳态偏差。
4. 输出 OCS2 / MoveIt / RViz 需要的话题:
     /joint_states  /base_controller/odom  /clock  TF(odom->base_footprint)
"""

import math
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Time as TimeMsg
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from tf2_ros import TransformBroadcaster


ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
WHEELS = ["left_wheel", "right_wheel"]


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


def secs_to_time_msg(t):
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    m = TimeMsg()
    m.sec = sec
    m.nanosec = nsec
    return m


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        # ---------------- 参数 ----------------
        self.declare_parameter("model_path", "")
        # 平面底盘三个关节名（按你的 XML）
        self.declare_parameter("base_x_joint", "base_x")
        self.declare_parameter("base_y_joint", "base_y")
        self.declare_parameter("base_yaw_joint", "base_yaw")
        self.declare_parameter("wheel_separation", 0.34)
        self.declare_parameter("wheel_radius", 0.065)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("clock_rate", 250.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("exact_base", True)   # True: 平面运动学底盘，零偏差
        self.declare_parameter("spin_wheels", True)   # 仅视觉：让轮子转
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("arm_actuator_suffix", "_servo")

        gp = lambda n: self.get_parameter(n).value
        self.model_path = gp("model_path")
        self.bx_name = gp("base_x_joint")
        self.by_name = gp("base_y_joint")
        self.byaw_name = gp("base_yaw_joint")
        self.wheel_sep = float(gp("wheel_separation"))
        self.wheel_rad = float(gp("wheel_radius"))
        self.publish_rate = float(gp("publish_rate"))
        self.clock_rate = float(gp("clock_rate"))
        self.odom_frame = gp("odom_frame")
        self.base_frame = gp("base_frame")
        self.publish_odom_tf = bool(gp("publish_odom_tf"))
        self.exact_base = bool(gp("exact_base"))
        self.spin_wheels = bool(gp("spin_wheels"))
        self.use_viewer = bool(gp("use_viewer"))
        arm_suffix = gp("arm_actuator_suffix")

        if not self.model_path:
            raise RuntimeError("必须通过参数 model_path 指定 MuJoCo XML 路径")

        # ---------------- 加载模型 ----------------
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id < 0:
            raise RuntimeError("找不到 keyframe: home")

        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)


        nid = lambda t, n: mujoco.mj_name2id(self.model, t, n)

        # 臂关节 + 轮关节 + 三个底盘关节的 qpos/qvel 地址
        self.qadr, self.dadr = {}, {}
        for j in ARM_JOINTS + WHEELS + [self.bx_name, self.by_name, self.byaw_name]:
            i = nid(mujoco.mjtObj.mjOBJ_JOINT, j)
            if i < 0:
                self.get_logger().warn(f"模型中找不到关节 {j}")
                continue
            self.qadr[j] = self.model.jnt_qposadr[i]
            self.dadr[j] = self.model.jnt_dofadr[i]

        for j in (self.bx_name, self.by_name, self.byaw_name):
            if j not in self.qadr:
                raise RuntimeError(f"找不到底盘关节: {j}")

        # 臂位置执行器（轮子默认无执行器，纯运动学转动）
        self.a_arm = [nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}{arm_suffix}")
                      for j in ARM_JOINTS]

        # ---------------- 状态 ----------------
        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.arm_target = np.array(
            [self.data.qpos[self.qadr[j]] for j in ARM_JOINTS], dtype=float)
        self.arm_traj = None
        self.traj_start = 0.0

        # 底盘里程（从模型初值读起）
        self.base_x = float(self.data.qpos[self.qadr[self.bx_name]])
        self.base_y = float(self.data.qpos[self.qadr[self.by_name]])
        self.base_yaw = float(self.data.qpos[self.qadr[self.byaw_name]])
        # 轮子显示角度
        self.wheel_pos = {w: (float(self.data.qpos[self.qadr[w]]) if w in self.qadr else 0.0)
                          for w in WHEELS}

        self.sim_time = 0.0

        # ---------------- ROS 接口 ----------------
        cb = ReentrantCallbackGroup()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10, callback_group=cb)
        self.create_subscription(Twist, "/base_controller/cmd_vel", self.on_cmd_vel, 10,
                                 callback_group=cb)
        self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                 self.on_arm_traj, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "/arm_controller/commands",
                                 self.on_arm_forward, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "/arm_command",
                                 self.on_arm_forward, 10, callback_group=cb)

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "/base_controller/odom", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_bc = TransformBroadcaster(self)

        self._action = ActionServer(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_traj_action,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f"Loaded {self.model_path}  dt={self.dt:.4f}  exact_base={self.exact_base}")

    # ============================================================
    #  订阅回调
    # ============================================================
    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.cmd_v = float(msg.linear.x)
            self.cmd_w = float(msg.angular.z)

    def on_arm_forward(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            with self.lock:
                self.arm_traj = None
                self.arm_target = np.array(msg.data[:6], dtype=float)

    def _parse_trajectory(self, traj: JointTrajectory):
        if not traj.points:
            return None
        idx = {name: i for i, name in enumerate(traj.joint_names)}
        cols = [idx.get(j, None) for j in ARM_JOINTS]
        out = []
        for p in traj.points:
            q = self.arm_target.copy()
            for k, c in enumerate(cols):
                if c is not None and c < len(p.positions):
                    q[k] = float(p.positions[c])
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            out.append((t, q))
        out.sort(key=lambda x: x[0])
        return out

    def on_arm_traj(self, msg: JointTrajectory):
        parsed = self._parse_trajectory(msg)
        if parsed is None:
            return
        with self.lock:
            self.arm_traj = parsed
            self.traj_start = self.sim_time

    # ============================================================
    #  FollowJointTrajectory action（MoveIt）
    # ============================================================
    def execute_traj_action(self, goal_handle):
        traj = goal_handle.request.trajectory
        parsed = self._parse_trajectory(traj)
        result = FollowJointTrajectory.Result()
        if parsed is None:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        with self.lock:
            self.arm_traj = parsed
            self.traj_start = self.sim_time
        duration = parsed[-1][0]

        while rclpy.ok():
            with self.lock:
                elapsed = self.sim_time - self.traj_start
                if self.arm_traj is not parsed:
                    break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            if elapsed >= duration:
                break
            time.sleep(0.02)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ============================================================
    #  控制 / 物理步进
    # ============================================================
    def _interp_arm(self):
        traj = self.arm_traj
        if traj is None:
            return self.arm_target
        elapsed = self.sim_time - self.traj_start
        if elapsed <= traj[0][0]:
            return traj[0][1]
        if elapsed >= traj[-1][0]:
            return traj[-1][1]
        for i in range(1, len(traj)):
            t0, q0 = traj[i - 1]
            t1, q1 = traj[i]
            if elapsed <= t1:
                a = (elapsed - t0) / max(1e-6, (t1 - t0))
                return q0 + a * (q1 - q0)
        return traj[-1][1]

    def apply_arm_ctrl(self):
        with self.lock:
            self.arm_target = self._interp_arm()
            arm = self.arm_target.copy()
        for k, aid in enumerate(self.a_arm):
            if aid >= 0:
                self.data.ctrl[aid] = arm[k]

    def _integrate_planar_base(self):
        """
        平面运动学底盘：用 cmd_vel 积分位姿，直接写回 base_x/base_y/base_yaw。
        采用中点偏航以减小转弯时的积分误差。
        """
        with self.lock:
            v, w = self.cmd_v, self.cmd_w
        dt = self.dt
        yaw_mid = self.base_yaw + 0.5 * w * dt
        self.base_x += v * math.cos(yaw_mid) * dt
        self.base_y += v * math.sin(yaw_mid) * dt
        self.base_yaw = wrap_pi(self.base_yaw + w * dt)

        # 写回位姿（覆盖物理积分结果 -> 纯运动学跟随）
        self.data.qpos[self.qadr[self.bx_name]] = self.base_x
        self.data.qpos[self.qadr[self.by_name]] = self.base_y
        self.data.qpos[self.qadr[self.byaw_name]] = self.base_yaw
        # 写回速度（世界系线速度 + yaw 速率），使上报 twist 精确
        self.data.qvel[self.dadr[self.bx_name]] = v * math.cos(self.base_yaw)
        self.data.qvel[self.dadr[self.by_name]] = v * math.sin(self.base_yaw)
        self.data.qvel[self.dadr[self.byaw_name]] = w

        # 轮子视觉旋转（纯运动学）
        if self.spin_wheels:
            wl = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
            wr = (v + w * self.wheel_sep / 2.0) / self.wheel_rad
            for name, omega in (("left_wheel", wl), ("right_wheel", wr)):
                if name in self.qadr:
                    self.wheel_pos[name] += omega * dt
                    self.data.qpos[self.qadr[name]] = self.wheel_pos[name]
                    self.data.qvel[self.dadr[name]] = omega

    def step(self):
        self.apply_arm_ctrl()
        mujoco.mj_step(self.model, self.data)
        if self.exact_base:
            self._integrate_planar_base()
            mujoco.mj_forward(self.model, self.data)  # 刷新运动学供 viewer / 读取
        self.sim_time += self.dt

    # ============================================================
    #  发布
    # ============================================================
    def publish_clock(self):
        c = Clock()
        c.clock = secs_to_time_msg(self.sim_time)
        self.clock_pub.publish(c)

    def publish_state(self):
        now = secs_to_time_msg(self.sim_time)

        # joint_states
        js = JointState()
        js.header.stamp = now
        names = [j for j in ARM_JOINTS + WHEELS if j in self.qadr]
        js.name = names
        js.position = [float(self.data.qpos[self.qadr[j]]) for j in names]
        js.velocity = [float(self.data.qvel[self.dadr[j]]) for j in names]
        self.js_pub.publish(js)

        # 位姿（平面三关节直接读）
        x = float(self.data.qpos[self.qadr[self.bx_name]])
        y = float(self.data.qpos[self.qadr[self.by_name]])
        yaw = float(self.data.qpos[self.qadr[self.byaw_name]])
        vx_w = float(self.data.qvel[self.dadr[self.bx_name]])
        vy_w = float(self.data.qvel[self.dadr[self.by_name]])
        wz = float(self.data.qvel[self.dadr[self.byaw_name]])
        c, s = math.cos(yaw), math.sin(yaw)
        v_body = vx_w * c + vy_w * s
        vy_body = -vx_w * s + vy_w * c
        oq_w, _, _, oq_z = yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = oq_z
        odom.pose.pose.orientation.w = oq_w
        odom.twist.twist.linear.x = v_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = oq_z
            t.transform.rotation.w = oq_w
            self.tf_bc.sendTransform(t)


def _run_loop(node, with_viewer):
    pub_decim = max(1, int(round((1.0 / node.publish_rate) / node.dt)))
    clk_decim = max(1, int(round((1.0 / node.clock_rate) / node.dt)))

    def body(viewer=None):
        step = 0
        next_wall = time.perf_counter()
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            node.step()
            if step % clk_decim == 0:
                node.publish_clock()
            if step % pub_decim == 0:
                node.publish_state()
            if viewer is not None:
                viewer.sync()
            step += 1
            next_wall += node.dt
            sleep = next_wall - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_wall = time.perf_counter()

    if with_viewer:
        with mujoco.viewer.launch_passive(
            node.model,
            node.data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            body(viewer)

    else:
        body(None)


def main():
    rclpy.init()
    node = MujocoBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        _run_loop(node, node.use_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
