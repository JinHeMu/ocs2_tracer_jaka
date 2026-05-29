#!/usr/bin/env python3
"""
mujoco_bridge_node.py  ——  ROS2 <-> MuJoCo 桥接节点
直接加载 tracer_jaka_zu5_fixed.xml，用其内部已调好的执行器驱动整机。

订阅:
  /cmd_vel      geometry_msgs/Twist          差速底盘
  /arm_command  std_msgs/Float64MultiArray   6 个机械臂关节目标角(rad)
发布:
  /joint_states sensor_msgs/JointState
  TF: odom -> base_footprint （底盘里程，由 freejoint 位姿投影到地面得到）
"""
import math
import threading
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from ament_index_python.packages import get_package_share_directory
import os

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
WHEEL_SEPARATION = 0.34
WHEEL_RADIUS = 0.065


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        default_model = os.path.join(
            get_package_share_directory("tracer_jaka_mujoco"),
            "models", "tracer_jaka_zu5_fixed.xml")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("realtime", True)
        path = self.get_parameter("model_path").value

        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        nid = lambda t, n: mujoco.mj_name2id(self.model, t, n)
        self.a_left = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_servo")
        self.a_right = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_servo")
        self.a_arm = [nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_servo") for j in ARM_JOINTS]

        self.jid = {j: nid(mujoco.mjtObj.mjOBJ_JOINT, j)
                    for j in ARM_JOINTS + ["left_wheel", "right_wheel"]}
        self.qadr = {j: self.model.jnt_qposadr[i] for j, i in self.jid.items()}
        self.dadr = {j: self.model.jnt_dofadr[i] for j, i in self.jid.items()}
        # freejoint（底盘自由度）的 qpos 起始地址
        self.base_qadr = self.model.jnt_qposadr[
            nid(mujoco.mjtObj.mjOBJ_JOINT, "base_free")]

        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.arm_target = np.array([self.data.qpos[self.qadr[j]] for j in ARM_JOINTS])

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Float64MultiArray, "/arm_command", self.on_arm, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_bc = TransformBroadcaster(self)
        self.get_logger().info(f"Loaded {path}  dt={self.dt}")

    def on_cmd_vel(self, msg):
        with self.lock:
            self.cmd_v, self.cmd_w = msg.linear.x, msg.angular.z

    def on_arm(self, msg):
        if len(msg.data) >= 6:
            with self.lock:
                self.arm_target = np.array(msg.data[:6], dtype=float)

    def apply_ctrl(self):
        with self.lock:
            v, w, arm = self.cmd_v, self.cmd_w, self.arm_target.copy()
        self.data.ctrl[self.a_left] = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        self.data.ctrl[self.a_right] = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        for k, aid in enumerate(self.a_arm):
            self.data.ctrl[aid] = arm[k]

    def publish(self):
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        names = ARM_JOINTS + ["left_wheel", "right_wheel"]
        js.name = names
        js.position = [float(self.data.qpos[self.qadr[j]]) for j in names]
        js.velocity = [float(self.data.qvel[self.dadr[j]]) for j in names]
        self.js_pub.publish(js)

        # odom -> base_footprint（取 freejoint 的 x,y,yaw，z 投影到地面）
        b = self.base_qadr
        x, y = self.data.qpos[b], self.data.qpos[b + 1]
        qw, qx, qy, qz = (self.data.qpos[b + 3], self.data.qpos[b + 4],
                          self.data.qpos[b + 5], self.data.qpos[b + 6])
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_bc.sendTransform(t)


def main():
    rclpy.init()
    node = MujocoBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    decim = max(1, int(0.02 / node.dt))
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        step = 0
        while rclpy.ok() and viewer.is_running():
            node.apply_ctrl()
            mujoco.mj_step(node.model, node.data)
            if step % decim == 0:
                node.publish()
            viewer.sync()
            step += 1
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
