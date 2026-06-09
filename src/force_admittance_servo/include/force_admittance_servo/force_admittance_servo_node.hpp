// Copyright 2024, Force Admittance Servo Node
// SPDX-License-Identifier: Apache-2.0
//
// 恒力/导纳控制节点：按轴可选控制模式，输出末端速度给 MoveIt Servo
//
// 控制律（控制坐标系下，第 i 轴）：
//   导纳模式：M_i * x_ddot_i = F_i - D_i * x_dot_i - K_i * x_i
//   恒力模式：M_i * x_ddot_i = F_target_i + F_i - D_i * x_dot_i
//
// 符号约定：
//   F_i        — 传感器测量到的外力（环境反力，接触时方向与运动相反）
//   F_target_i — 期望的施加力（恒力模式），正值表示正轴方向
//   平衡条件：F_target + F_reaction = 0

#pragma once

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

namespace force_admittance_servo
{

/// @brief 单轴控制模式
enum class AxisMode : int
{
  DISABLED       = 0,  ///< 该轴不控制，速度输出为 0
  ADMITTANCE     = 1,  ///< 导纳柔顺控制（跟随外力）
  FORCE_CONTROL  = 2,  ///< 恒力控制（维持目标接触力）
};

/// @brief 每轴控制状态（积分量）
struct AxisState
{
  double position   = 0.0;  ///< 相对位移 x   [m 或 rad]
  double velocity   = 0.0;  ///< 速度      x_dot   [m/s 或 rad/s]
  double accel      = 0.0;  ///< 加速度    x_ddot  [m/s² 或 rad/s²]
};

/// @brief 恒力/导纳控制参数（运行时可动态更新）
struct ControlParams
{
  std::array<AxisMode, 6> mode        = {AxisMode::DISABLED};   ///< 每轴模式
  std::array<double, 6>   mass        = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  std::array<double, 6>   damping     = {50.0, 50.0, 50.0, 5.0, 5.0, 5.0};
  std::array<double, 6>   stiffness   = {200.0, 200.0, 200.0, 20.0, 20.0, 20.0};
  std::array<double, 6>   target_wrench = {0.0};  ///< 恒力目标 [N 或 Nm]
  double max_linear_vel   = 0.1;    ///< 线速度限幅 [m/s]
  double max_angular_vel  = 0.5;    ///< 角速度限幅 [rad/s]
  double max_linear_accel = 0.5;    ///< 线加速度限幅 [m/s²]
  double max_angular_accel = 1.0;  ///< 角加速度限幅 [rad/s²]
};

// ─────────────────────────────────────────────────────────────────────────────

class ForceAdmittanceServoNode : public rclcpp::Node
{
public:
  explicit ForceAdmittanceServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:

      // 在 ForceAdmittanceServoNode 类私有成员中添加
  Eigen::Matrix<double, 6, 1> prev_vel_out_ = Eigen::Matrix<double, 6, 1>::Zero();


  bool getToolRotation(Eigen::Matrix3d & R_base_tool) const;

  // ── 初始化 ──────────────────────────────────────────────────────────────────
  void declareParameters();
  void loadParameters();
  void setupTopics();

  // ── 回调 ────────────────────────────────────────────────────────────────────
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  // 新增：手柄话题回调函数声明
  void joyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void trackerTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();

  // ── 控制核心 ─────────────────────────────────────────────────────────────────
  /// @brief 对单轴执行控制律，返回加速度
  double computeAxisAccel(int axis, double force_ctrl, const AxisState & state,
                          const ControlParams & p) const;

  /// @brief 将速度向量限幅后发布为 TwistStamped
  void publishTwist(const Eigen::Matrix<double, 6, 1> & vel);

  /// @brief 重置所有积分状态
  void resetState();

  // ── 参数动态更新 ──────────────────────────────────────────────────────────────
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // ── 成员变量 ──────────────────────────────────────────────────────────────────
  ControlParams params_;
  std::array<AxisState, 6> axis_state_;

  // 传感器数据（mutex 保护）
  std::mutex wrench_mutex_;
  std::mutex joy_mutex_;
  std::mutex tracker_mutex_;
  geometry_msgs::msg::Wrench latest_wrench_{};
  geometry_msgs::msg::Twist latest_joy_twist_;
  geometry_msgs::msg::Twist latest_tracker_twist_;
  bool joy_received_ = false;
  bool wrench_received_ = false;
  bool tracker_received_ = false;

  // 控制使能
  bool enabled_ = true;

  // 时间步长
  double dt_        = 0.008;   // 默认 125 Hz
  double ctrl_rate_ = 125.0;
  
  // 坐标系名称（twist 发布时的 frame_id）
  std::string control_frame_id_ = "base_link";
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string ee_frame_id_{"tool0"};  // 与你的URDF一致

  // ROS 通信
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr tracker_sub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;



  // 动态参数回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace force_admittance_servo

