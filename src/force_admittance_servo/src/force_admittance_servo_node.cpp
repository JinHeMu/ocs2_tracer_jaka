// Copyright 2024, Force Admittance Servo Node
// SPDX-License-Identifier: Apache-2.0

#include "force_admittance_servo/force_admittance_servo_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace force_admittance_servo
{

// ─── 辅助：从参数名字中解析轴索引的字符串向量 ───────────────────────────────────
static std::array<AxisMode, 6> parseModeArray(const std::vector<int64_t> & v)
{
  if (v.size() != 6) {
    throw std::runtime_error("mode array must have exactly 6 elements");
  }
  std::array<AxisMode, 6> out;
  for (int i = 0; i < 6; ++i) {
    switch (v[i]) {
      case 0: out[i] = AxisMode::DISABLED;      break;
      case 1: out[i] = AxisMode::ADMITTANCE;    break;
      case 2: out[i] = AxisMode::FORCE_CONTROL; break;
      default:
        throw std::runtime_error("invalid axis mode: " + std::to_string(v[i]));
    }
  }
  return out;
}

static std::array<double, 6> toArray6(const std::vector<double> & v)
{
  if (v.size() != 6) {
    throw std::runtime_error("expected array of length 6, got " + std::to_string(v.size()));
  }
  std::array<double, 6> out;
  std::copy(v.begin(), v.end(), out.begin());
  return out;
}

bool ForceAdmittanceServoNode::getToolRotation(Eigen::Matrix3d & R_base_tool) const
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      control_frame_id_, ee_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.02));

    Eigen::Quaterniond q(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    R_base_tool = q.toRotationMatrix();  // base → tool 的旋转
    return true;
  } catch (...) {
    return false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  构造 & 初始化
// ─────────────────────────────────────────────────────────────────────────────

ForceAdmittanceServoNode::ForceAdmittanceServoNode(const rclcpp::NodeOptions & options)
: Node("force_admittance_servo", options)
{
  declareParameters();
  loadParameters();
  setupTopics();
  resetState();

  // 注册动态参数回调
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ForceAdmittanceServoNode::onParameterChange, this, std::placeholders::_1));

  // 控制定时器
  dt_ = 1.0 / ctrl_rate_;
  ctrl_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_),
    std::bind(&ForceAdmittanceServoNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "ForceAdmittanceServoNode 启动，控制频率 %.1f Hz，坐标系: %s",
    ctrl_rate_, control_frame_id_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  参数声明
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::declareParameters()
{
  // ── 话题 ──
  this->declare_parameter<std::string>(
    "wrench_topic", "/tcp_fts_sensor/wrench");
  this->declare_parameter<std::string>(
    "servo_twist_topic", "/servo_node/delta_twist_cmds");
     
  this->declare_parameter<std::string>(
    "enable_topic", "~/enable");
  this->declare_parameter<std::string>(
    "control_frame_id", "base_link");

  // ── 频率 ──
  this->declare_parameter<double>("control_rate", 125.0);

  // ── 每轴模式
  //   0 = DISABLED, 1 = ADMITTANCE, 2 = FORCE_CONTROL
  //   顺序: [x, y, z, rx, ry, rz]（控制坐标系）
  this->declare_parameter<std::vector<int64_t>>(
    "axis_mode", {1, 1, 2, 0, 0, 0});

  // ── 动力学参数（每轴）──
  this->declare_parameter<std::vector<double>>(
    "mass",       {1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
  this->declare_parameter<std::vector<double>>(
    "damping",    {50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
  this->declare_parameter<std::vector<double>>(
    "stiffness",  {200.0, 200.0, 200.0, 20.0, 20.0, 20.0});

  // ── 恒力目标（仅 FORCE_CONTROL 轴生效）──
  this->declare_parameter<std::vector<double>>(
    "target_wrench", {0.0, 0.0, -5.0, 0.0, 0.0, 0.0});

  // ── 速度限幅 ──
  this->declare_parameter<double>("max_linear_vel",  0.1);
  this->declare_parameter<double>("max_angular_vel", 0.5);
  
  // 在 declareParameters() 中添加
  this->declare_parameter<double>("max_linear_accel",  0.5);  // m/s^2
  this->declare_parameter<double>("max_angular_accel", 1.0);  // rad/s^2


}

// ─────────────────────────────────────────────────────────────────────────────
//  参数加载
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::loadParameters()
{
  ctrl_rate_        = this->get_parameter("control_rate").as_double();
  control_frame_id_ = this->get_parameter("control_frame_id").as_string();

  auto mode_vec = this->get_parameter("axis_mode").as_integer_array();
  params_.mode = parseModeArray(mode_vec);

  params_.mass          = toArray6(this->get_parameter("mass").as_double_array());
  params_.damping       = toArray6(this->get_parameter("damping").as_double_array());
  params_.stiffness     = toArray6(this->get_parameter("stiffness").as_double_array());
  params_.target_wrench = toArray6(this->get_parameter("target_wrench").as_double_array());
  // 在 loadParameters() 中添加
  params_.max_linear_accel  = this->get_parameter("max_linear_accel").as_double();
  params_.max_angular_accel = this->get_parameter("max_angular_accel").as_double();

  params_.max_linear_vel  = this->get_parameter("max_linear_vel").as_double();
  params_.max_angular_vel = this->get_parameter("max_angular_vel").as_double();

}

// ─────────────────────────────────────────────────────────────────────────────
//  话题注册
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::setupTopics()
{
  const auto wrench_topic = this->get_parameter("wrench_topic").as_string();
  const auto twist_topic  = this->get_parameter("servo_twist_topic").as_string();
  const auto enable_topic = this->get_parameter("enable_topic").as_string();

  wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    wrench_topic, rclcpp::SensorDataQoS(),
    std::bind(&ForceAdmittanceServoNode::wrenchCallback, this, std::placeholders::_1));

  enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    enable_topic, 10,
    std::bind(&ForceAdmittanceServoNode::enableCallback, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    twist_topic, rclcpp::SystemDefaultsQoS());

  // 新增：订阅手柄参考速度
  joy_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/joy_reference_twist", rclcpp::SystemDefaultsQoS(),
    std::bind(&ForceAdmittanceServoNode::joyTwistCallback, this, std::placeholders::_1));

  tracker_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/tracker_reference_twist", rclcpp::SystemDefaultsQoS(),
    std::bind(&ForceAdmittanceServoNode::trackerTwistCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "订阅手柄参考速度: /joy_reference_twist");
  RCLCPP_INFO(this->get_logger(), "订阅跟踪参考速度: /tracker_reference_twist");
  RCLCPP_INFO(this->get_logger(), "订阅力矩话题: %s", wrench_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "发布 Servo 速度话题: %s", twist_topic.c_str());

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  状态重置
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::resetState()
{
  for (auto & s : axis_state_) {
    s.position = 0.0;
    s.velocity = 0.0;
    s.accel    = 0.0;
  }
  prev_vel_out_.setZero(); // 重置平滑器的状态
}

// ─────────────────────────────────────────────────────────────────────────────
//  传感器回调
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::wrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  latest_wrench_  = msg->wrench;
  wrench_received_ = true;
}


void ForceAdmittanceServoNode::joyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joy_mutex_);
  latest_joy_twist_ = msg->twist;
  joy_received_ = true;
}

void ForceAdmittanceServoNode::trackerTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tracker_mutex_);
  latest_tracker_twist_ = msg->twist;
  tracker_received_ = true;
}



// ─────────────────────────────────────────────────────────────────────────────
//  使能话题回调
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data && enabled_) {
    RCLCPP_INFO(this->get_logger(), "控制器已禁用，重置积分状态");
    resetState();
  } else if (msg->data && !enabled_) {
    RCLCPP_INFO(this->get_logger(), "控制器已启用");
    resetState();
  }
  enabled_ = msg->data;
}

// ─────────────────────────────────────────────────────────────────────────────
//  单轴控制律
//
//  参数说明：
//   axis       — 轴索引 [0,5]
//   force_ctrl — 该轴在控制坐标系下的测量力（已补偿，纯净值）
//   state      — 该轴当前积分状态
//   p          — 控制参数
//
//  返回：该轴加速度 x_ddot
// ─────────────────────────────────────────────────────────────────────────────



double ForceAdmittanceServoNode::computeAxisAccel(
  int axis, double force_ctrl, const AxisState & state, const ControlParams & p) const
{
  const double m_inv = (p.mass[axis] > 1e-9) ? (1.0 / p.mass[axis]) : 0.0;
  const double d     = p.damping[axis];
  const double k     = p.stiffness[axis];


  switch (p.mode[axis]) {
    case AxisMode::DISABLED:
      return 0.0;

    case AxisMode::ADMITTANCE:
      // 导纳模式：暂保持原样。
      // 注意：如果你发现在导纳模式下力矩的响应也是反的，请将这里的 force_ctrl 也改为 -force_ctrl
      return m_inv * (force_ctrl - d * state.velocity - k * state.position);

    case AxisMode::FORCE_CONTROL:
        return m_inv * (p.target_wrench[axis] + force_ctrl - d * state.velocity);

    default:
      return 0.0;
  }
}


// ─────────────────────────────────────────────────────────────────────────────
//  主控制循环
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::controlLoop()
{
  if (!enabled_) {
    // 使能关闭时持续发布零速，防止 Servo 超时报警
    Eigen::Matrix<double, 6, 1> zero_vel = Eigen::Matrix<double, 6, 1>::Zero();
    publishTwist(zero_vel);
    return;
  }

  if (!wrench_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "尚未收到力矩传感器数据，等待中...");
    return;
  }

  // ── 1. 获取当前 base→tool 旋转矩阵 ──────────────────────
  Eigen::Matrix3d R_base_tool;
  if (!getToolRotation(R_base_tool)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF 查询失败，跳过本周期");
    return;
  }

  Eigen::Matrix3d R_tool_base = R_base_tool.transpose();

 // ── 2. 取力矩数据（已在工具坐标系下）──────────────────────
  geometry_msgs::msg::Wrench wrench;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    wrench = latest_wrench_;
  }

  const std::array<double, 6> F = {
    wrench.force.x,  wrench.force.y,  wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z
  };

  // const std::array<double, 6> F = {
  //   0,  0,  0,
  //   0, 0, 0
  // };
  // 新增：提取最新的手柄速度（线程安全）
  geometry_msgs::msg::Twist joy_twist;
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    if (joy_received_) {
      joy_twist = latest_joy_twist_;
    } else {
      // 如果还没收到手柄数据，默认全为0
      joy_twist.linear.x = 0; joy_twist.linear.y = 0; joy_twist.linear.z = 0;
      joy_twist.angular.x = 0; joy_twist.angular.y = 0; joy_twist.angular.z = 0;
    }
  }
  // joy base_link
  Eigen::Matrix<double, 6, 1> joy_ref_tool;
  joy_ref_tool << joy_twist.linear.x, joy_twist.linear.y, joy_twist.linear.z,
                joy_twist.angular.x, joy_twist.angular.y, joy_twist.angular.z;

 // ── 3. 取路径跟踪器的参考速度（base_link 坐标系）─────────  
  geometry_msgs::msg::Twist tracker_twist;
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    if (tracker_received_) {
      tracker_twist = latest_tracker_twist_;
    } else {
      tracker_twist.linear.x = 0; tracker_twist.linear.y = 0; tracker_twist.linear.z = 0;
      tracker_twist.angular.x = 0; tracker_twist.angular.y = 0; tracker_twist.angular.z = 0;
    }
  }

  // ── 4. 将路径参考速度从 base_link 变换到工具坐标系 ───────
  Eigen::Vector3d v_path_base(tracker_twist.linear.x, tracker_twist.linear.y, tracker_twist.linear.z);
  Eigen::Vector3d w_path_base(tracker_twist.angular.x, tracker_twist.angular.y, tracker_twist.angular.z);

  Eigen::Vector3d v_path_tool = R_tool_base * v_path_base;
  Eigen::Vector3d w_path_tool = R_tool_base * w_path_base;

  Eigen::Matrix<double, 6, 1> path_ref_tool;
  path_ref_tool << v_path_tool, w_path_tool;


  // ── 5. 轴权责分离：力控轴清零路径参考 ─────────────────────
  //    只有 DISABLED 和 ADMITTANCE 轴接受路径参考
  //    FORCE_CONTROL 轴的路径参考强制置零，防止闭环冲突
  for (int i = 0; i < 6; ++i) {
    if (params_.mode[i] == AxisMode::FORCE_CONTROL) {
      path_ref_tool[i] = 0.0;  // 力控轴：路径跟踪器无权干预
    }
  }

  // ── 6. 逐轴力控/导纳计算（工具坐标系下）──────────────────
  Eigen::Matrix<double, 6, 1> vel_force_tool;
  for (int i = 0; i < 6; ++i) {
    if (params_.mode[i] == AxisMode::DISABLED) {
      axis_state_[i] = {0.0, 0.0, 0.0};
      vel_force_tool[i] = 0.0;
      continue;
    }
    const double accel = computeAxisAccel(i, F[i], axis_state_[i], params_);
    axis_state_[i].velocity += accel * dt_;
    axis_state_[i].position += axis_state_[i].velocity * dt_;
    axis_state_[i].accel = accel;
    vel_force_tool[i] = axis_state_[i].velocity;
  }

 

    // ── 7. 在工具坐标系下叠加 ─────────────────────────────────
  Eigen::Matrix<double, 6, 1> vel_total_tool = vel_force_tool + path_ref_tool;

  // ── 8. 变换回 base_link 发布 ──────────────────────────────
  Eigen::Vector3d v_total_base = R_base_tool * vel_total_tool.head<3>() + joy_ref_tool.head<3>();
  Eigen::Vector3d w_total_base = R_base_tool * vel_total_tool.tail<3>() + joy_ref_tool.tail<3>();


  Eigen::Matrix<double, 6, 1> vel_out;
  vel_out << v_total_base, w_total_base;
  
  publishTwist(vel_out);
}

// ─────────────────────────────────────────────────────────────────────────────
//  发布 TwistStamped → MoveIt Servo
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::publishTwist(const Eigen::Matrix<double, 6, 1> & vel_target)
{
  Eigen::Matrix<double, 6, 1> vel_limited;

  for (int i = 0; i < 6; ++i) {
    // 1. 确定当前轴使用的加速度和速度限幅值
    double max_accel = (i < 3) ? params_.max_linear_accel : params_.max_angular_accel;
    double max_vel   = (i < 3) ? params_.max_linear_vel   : params_.max_angular_vel;

    // 2. 加速度限幅：计算本周期允许的最大速度增量 (dv = a * dt)
    double max_dv = max_accel * dt_;
    double desired_dv = vel_target[i] - prev_vel_out_[i];
    
    // 限制 dv 的范围在 [-max_dv, max_dv]
    double limited_dv = std::clamp(desired_dv, -max_dv, max_dv);
    
    // 3. 更新速度
    vel_limited[i] = prev_vel_out_[i] + limited_dv;

    // 4. 速度限幅：确保最终输出不超过最大允许速度
    vel_limited[i] = std::clamp(vel_limited[i], -max_vel, max_vel);
  }

  // 更新上一周期速度记录
  prev_vel_out_ = vel_limited;

  // 5. 构造并发布消息
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = control_frame_id_;
  msg.twist.linear.x  = vel_limited[0];
  msg.twist.linear.y  = vel_limited[1];
  msg.twist.linear.z  = vel_limited[2];
  msg.twist.angular.x = vel_limited[3];
  msg.twist.angular.y = vel_limited[4];
  msg.twist.angular.z = vel_limited[5];

  twist_pub_->publish(msg);
}


// ─────────────────────────────────────────────────────────────────────────────
//  动态参数回调（运行时修改 target_wrench / mode 等，无需重启）
// ─────────────────────────────────────────────────────────────────────────────

rcl_interfaces::msg::SetParametersResult
ForceAdmittanceServoNode::onParameterChange(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    try {
      if (p.get_name() == "axis_mode") {
        params_.mode = parseModeArray(p.as_integer_array());
        RCLCPP_INFO(this->get_logger(), "axis_mode 已更新");
        resetState();
      } else if (p.get_name() == "mass") {
        params_.mass = toArray6(p.as_double_array());
      } else if (p.get_name() == "damping") {
        params_.damping = toArray6(p.as_double_array());
      } else if (p.get_name() == "stiffness") {
        params_.stiffness = toArray6(p.as_double_array());
      } else if (p.get_name() == "target_wrench") {
        params_.target_wrench = toArray6(p.as_double_array());
        RCLCPP_INFO(this->get_logger(), "target_wrench 已更新: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
          params_.target_wrench[0], params_.target_wrench[1], params_.target_wrench[2],
          params_.target_wrench[3], params_.target_wrench[4], params_.target_wrench[5]);
      } else if (p.get_name() == "max_linear_vel") {
        params_.max_linear_vel = p.as_double();
      } else if (p.get_name() == "max_angular_vel") {
        params_.max_angular_vel = p.as_double();
      } else if (p.get_name() == "max_linear_accel") {
        params_.max_linear_accel = p.as_double();
      } else if (p.get_name() == "max_angular_accel") {
        params_.max_angular_accel = p.as_double();
      }
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason     = e.what();
      RCLCPP_ERROR(this->get_logger(), "参数更新失败: %s", e.what());
    }
  }
  return result;
}

}  // namespace force_admittance_servo

