// Copyright (c) 2026
// Bridge between OCS2 mobile_manipulator MPC and Jaka+Tracer real robot.

#include "jaka_tracer_ocs2/jaka_tracer_bridge.hpp"

#include <algorithm>
#include <cmath>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MRT_BASE.h>

namespace jaka_tracer_ocs2
{

using std::placeholders::_1;

JakaTracerBridge::JakaTracerBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("jaka_tracer_ocs2_bridge", options)
{
  // ---------- 参数 ----------
  arm_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

  robot_name_           = this->declare_parameter<std::string>("robot_name", "mobile_manipulator");
  observation_rate_hz_  = this->declare_parameter<double>("observation_rate_hz", 25.0);
  control_rate_hz_      = this->declare_parameter<double>("control_rate_hz",     200.0);
  traj_lookahead_s_     = this->declare_parameter<double>("traj_lookahead_s",    0.04);
  base_max_v_           = this->declare_parameter<double>("base_max_v",          0.5);
  base_max_w_           = this->declare_parameter<double>("base_max_w",          1.0);
  send_base_cmd_        = this->declare_parameter<bool>("send_base_cmd",         true);
  send_arm_cmd_         = this->declare_parameter<bool>("send_arm_cmd",          true);

  arm_q_  = ocs2::vector_t::Zero(arm_joint_names_.size());
  arm_dq_ = ocs2::vector_t::Zero(arm_joint_names_.size());

  // ---------- OCS2 MRT ----------
  // MRT_ROS_Interface 内部会创建 publisher /<robot_name>_mpc_observation
  // 和 subscriber  /<robot_name>_mpc_policy。
  mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>(robot_name_);
  mrt_->launchNodes(this->shared_from_this());

  // OCS2 SystemObservation 维度：state 9 维, input 8 维（wheelBased）
  observation_.time = 0.0;
  observation_.state = ocs2::vector_t::Zero(9);
  observation_.input = ocs2::vector_t::Zero(8);
  observation_.mode  = 0;

  // ---------- ROS2 订阅 ----------
  rclcpp::QoS qos_state(20);
  qos_state.best_effort();

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos_state,
      std::bind(&JakaTracerBridge::jointStateCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos_state,
      std::bind(&JakaTracerBridge::odomCallback, this, _1));

  // ---------- ROS2 发布 ----------
  cmd_vel_pub_  = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  arm_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/jaka_arm_controller/joint_trajectory", 10);

  // ---------- 定时器 ----------
  observation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / observation_rate_hz_),
      std::bind(&JakaTracerBridge::publishObservation, this));

  control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&JakaTracerBridge::controlLoop, this));

  RCLCPP_INFO(get_logger(),
              "JakaTracerBridge started. obs=%.0f Hz, ctrl=%.0f Hz, lookahead=%.3f s",
              observation_rate_hz_, control_rate_hz_, traj_lookahead_s_);
}

// ============================================================
//                    Subscription Callbacks
// ============================================================

bool JakaTracerBridge::extractArmJoints(const sensor_msgs::msg::JointState & msg,
                                        ocs2::vector_t & arm_q,
                                        ocs2::vector_t & arm_dq) const
{
  arm_q.resize(arm_joint_names_.size());
  arm_dq.resize(arm_joint_names_.size());

  for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), arm_joint_names_[i]);
    if (it == msg.name.end()) {
      return false;
    }
    const auto idx = std::distance(msg.name.begin(), it);
    arm_q[i]  = msg.position[idx];
    arm_dq[i] = (idx < static_cast<long>(msg.velocity.size())) ? msg.velocity[idx] : 0.0;
  }
  return true;
}

void JakaTracerBridge::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  ocs2::vector_t q, dq;
  if (!extractArmJoints(*msg, q, dq)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Some arm joints missing in /joint_states");
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  arm_q_  = q;
  arm_dq_ = dq;
  joint_state_ready_ = true;
}

double JakaTracerBridge::yawFromQuat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void JakaTracerBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  base_x_   = msg->pose.pose.position.x;
  base_y_   = msg->pose.pose.position.y;
  base_yaw_ = yawFromQuat(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);
  base_v_meas_ = msg->twist.twist.linear.x;
  base_w_meas_ = msg->twist.twist.angular.z;
  odom_ready_ = true;
}

// ============================================================
//                    Observation -> MPC
// ============================================================
void JakaTracerBridge::publishObservation()
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (!joint_state_ready_ || !odom_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for /joint_states (%d) and /odom (%d) ...",
                         joint_state_ready_ ? 1 : 0, odom_ready_ ? 1 : 0);
    return;
  }

  observation_.time = this->now().seconds();

  // 状态: [x_b, y_b, yaw_b, q_1..q_6]
  observation_.state.resize(9);
  observation_.state(0) = base_x_;
  observation_.state(1) = base_y_;
  observation_.state(2) = base_yaw_;
  observation_.state.tail(6) = arm_q_;

  // 输入: [v, w, dq_1..dq_6]，反馈最近测量值便于 MPC warmstart
  observation_.input.resize(8);
  observation_.input(0) = base_v_meas_;
  observation_.input(1) = base_w_meas_;
  observation_.input.tail(6) = arm_dq_;

  observation_.mode = 0;

  if (!mrt_initialized_) {
    mrt_->resetMpcNode(ocs2::TargetTrajectories({0.0}, {observation_.state}, {observation_.input}));
    mrt_initialized_ = true;
    RCLCPP_INFO(get_logger(), "MRT reset done. Sending observation to MPC.");
  }

  mrt_->setCurrentObservation(observation_);
}

// ============================================================
//                    Policy -> Robot Commands
// ============================================================
void JakaTracerBridge::controlLoop()
{
  if (!mrt_initialized_) {return;}

  // 1. 等 MPC 出第一条 policy
  if (!policy_ready_) {
    if (mrt_->initialPolicyReceived()) {
      policy_ready_ = true;
      RCLCPP_INFO(get_logger(), "Initial MPC policy received.");
    } else {
      mrt_->spinMRT();
      return;
    }
  }

  // 2. 拉取最新 policy
  mrt_->spinMRT();
  mrt_->updatePolicy();

  // 3. 在当前时间 + lookahead 处插值 policy
  const double t_now    = this->now().seconds();
  const double t_target = t_now + traj_lookahead_s_;

  ocs2::vector_t state_des, input_des;
  size_t mode_des = 0;
  mrt_->evaluatePolicy(t_target, observation_.state, state_des, input_des, mode_des);

  if (state_des.size() < 9 || input_des.size() < 8) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Policy size mismatch: state=%zu input=%zu",
                         (size_t)state_des.size(), (size_t)input_des.size());
    return;
  }

  // 4. 拆分: base 用速度、arm 用位置
  // input_des = [v, w, dq_1..dq_6]
  // state_des = [x_b, y_b, yaw_b, q_1..q_6]
  if (send_base_cmd_) {
    geometry_msgs::msg::Twist tw;
    tw.linear.x  = std::clamp(input_des(0), -base_max_v_, base_max_v_);
    tw.angular.z = std::clamp(input_des(1), -base_max_w_, base_max_w_);
    cmd_vel_pub_->publish(tw);
  }

  if (send_arm_cmd_) {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.header.stamp = this->now();
    jt.joint_names  = arm_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(6);
    pt.velocities.resize(6);
    for (int i = 0; i < 6; ++i) {
      pt.positions[i]  = state_des(3 + i);
      pt.velocities[i] = input_des(2 + i);
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_lookahead_s_);
    jt.points.push_back(pt);
    arm_traj_pub_->publish(jt);
  }
}

}  // namespace jaka_tracer_ocs2

// ============================================================
//                          main()
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jaka_tracer_ocs2::JakaTracerBridge>(rclcpp::NodeOptions{});

  // 单线程执行器即可，MRT 内部已是异步
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
