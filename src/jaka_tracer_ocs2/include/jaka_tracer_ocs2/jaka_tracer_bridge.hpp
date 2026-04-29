// Copyright (c) 2026 - Jaka + Tracer OCS2 Real-Robot Bridge
//
// 把 OCS2 mobile_manipulator MPC 节点的输出转换为：
//   * /cmd_vel              -> Tracer 底盘
//   * /jaka_arm_controller/joint_trajectory -> Jaka 机械臂 (ros2_control JTC)
// 同时把真实机器人的 /joint_states + /odom 组合成 OCS2 SystemObservation
// 发布回 mpc_observation。

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

namespace jaka_tracer_ocs2
{

class JakaTracerBridge : public rclcpp::Node
{
public:
  explicit JakaTracerBridge(const rclcpp::NodeOptions & options);

private:
  // ---------- 订阅回调 ----------
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---------- 主循环 ----------
  // 25 Hz: 推送 observation 给 MPC
  void publishObservation();
  // 200 Hz: 取 MPC policy → 解出 [v,ω] + 6 关节位置目标 → 下发
  void controlLoop();

  // ---------- 工具函数 ----------
  // 把 /joint_states 里的关节按 task.info 顺序重排
  bool extractArmJoints(const sensor_msgs::msg::JointState & msg,
                        ocs2::vector_t & arm_q,
                        ocs2::vector_t & arm_dq) const;

  // 由 quaternion 提取 yaw
  static double yawFromQuat(double x, double y, double z, double w);

  // ---------- ROS2 接口 ----------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;

  rclcpp::TimerBase::SharedPtr observation_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---------- OCS2 ----------
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;
  ocs2::SystemObservation observation_;
  std::string robot_name_{"mobile_manipulator"};

  // ---------- 状态缓存 ----------
  mutable std::mutex mutex_;
  ocs2::vector_t arm_q_;          // 6 维
  ocs2::vector_t arm_dq_;         // 6 维
  double base_x_{0.0}, base_y_{0.0}, base_yaw_{0.0};
  double base_v_meas_{0.0}, base_w_meas_{0.0};
  bool   joint_state_ready_{false};
  bool   odom_ready_{false};
  bool   policy_ready_{false};
  bool   mrt_initialized_{false};

  // ---------- 参数 ----------
  std::vector<std::string> arm_joint_names_;
  double observation_rate_hz_{25.0};
  double control_rate_hz_{200.0};
  double traj_lookahead_s_{0.04};   // 给 JTC 的目标点放在多少秒之后
  double base_max_v_{0.5};
  double base_max_w_{1.0};
  bool   send_base_cmd_{true};
  bool   send_arm_cmd_{true};
};

}  // namespace jaka_tracer_ocs2
