// =============================================================================
//  TracerJakaMrtNode.cpp
//
//  OCS2 MRT 节点 + Gazebo 控制器桥 (ROS 2)。
//
//  关键设计点 (相对上一版的修复):
//   * 初始 target 不再写零, 而是通过 TF 查 odom -> gripper_center_link 的当前
//     位姿, 让 MPC 一开始就保持在原位.
//   * 给 JTC 发轨迹时 *只发位置*, 不发速度. 这样不会触发 JTC 的
//     "Velocity of last trajectory point is not zero" 拒收逻辑.
//   * 单点轨迹 (1 个 point @ traj_horizon ahead). 避免反复查询 MPC policy
//     超出 plan 末端 ("currentTime > plan_end" 警告).
//   * 给 MRT 单独一个 internal node, 主 node 留给我们自己 spin (避免
//     双重 add_node 冲突).
//
//  状态向量 (wheelBasedMobileManipulator, 已 removeJoints 轮子):
//    state(0..2) : base x, base y, base yaw
//    state(3..8) : joint_1 .. joint_6
//    input(0..1) : forward velocity, yaw rate
//    input(2..7) : joint_1_dot .. joint_6_dot
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

using namespace std::chrono_literals;

class TracerJakaMrtBridge : public rclcpp::Node {
 public:
  TracerJakaMrtBridge() : Node("tracer_jaka_mrt_node") {
    // ----- params -----
    declare_parameter<std::string>("taskFile",         "");
    declare_parameter<std::string>("libFolder",        "");
    declare_parameter<std::string>("urdfFile",         "");
    declare_parameter<double>("mrt_loop_rate",         100.0);
    declare_parameter<double>("traj_horizon",          0.05);
    declare_parameter<std::string>("base_cmd_topic",   "/diff_drive_controller/cmd_vel");
    declare_parameter<std::string>("arm_cmd_topic",    "/jaka_arm_controller/joint_trajectory");
    declare_parameter<std::string>("odom_topic",       "/diff_drive_controller/odom");
    declare_parameter<std::string>("joint_state_topic","/joint_states");
    declare_parameter<std::vector<std::string>>(
        "arm_joint_names",
        std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"});
    declare_parameter<std::string>("base_frame",       "base_footprint");
    declare_parameter<std::string>("world_frame",      "odom");
    declare_parameter<std::string>("ee_frame",         "gripper_center_link");

    taskFile_       = get_parameter("taskFile").as_string();
    libFolder_      = get_parameter("libFolder").as_string();
    urdfFile_       = get_parameter("urdfFile").as_string();
    mrtRate_        = get_parameter("mrt_loop_rate").as_double();
    trajHorizon_    = get_parameter("traj_horizon").as_double();
    armJointNames_  = get_parameter("arm_joint_names").as_string_array();
    baseFrame_      = get_parameter("base_frame").as_string();
    worldFrame_     = get_parameter("world_frame").as_string();
    eeFrame_        = get_parameter("ee_frame").as_string();

    armQ_.assign(armJointNames_.size(), 0.0);

    if (taskFile_.empty() || libFolder_.empty() || urdfFile_.empty()) {
      throw std::runtime_error(
          "taskFile / libFolder / urdfFile parameters must all be set.");
    }

    // ----- robot interface -----
    interface_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorInterface>(
        taskFile_, libFolder_, urdfFile_);

    const auto& info = interface_->getManipulatorModelInfo();
    stateDim_ = info.stateDim;
    inputDim_ = info.inputDim;
    armDim_   = info.armDim;

    RCLCPP_INFO(get_logger(),
                "OCS2 model dims  state=%zu  input=%zu  arm=%zu",
                stateDim_, inputDim_, armDim_);
    if (armDim_ != armJointNames_.size()) {
      RCLCPP_FATAL(get_logger(),
                   "armDim (%zu) != arm_joint_names size (%zu). "
                   "Check task.info -> removeJoints.",
                   armDim_, armJointNames_.size());
      throw std::runtime_error("armDim mismatch");
    }

    // ----- TF -----
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ----- pubs -----
    base_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        get_parameter("base_cmd_topic").as_string(), 10);
    arm_cmd_pub_  = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        get_parameter("arm_cmd_topic").as_string(), 10);

    // ----- subs (在主 node 上, 由 main() 里的 executor spin) -----
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaMrtBridge::odomCallback, this, std::placeholders::_1));
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_state_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&TracerJakaMrtBridge::jsCallback, this, std::placeholders::_1));
  }

  /// 给 MRT 一个单独的内部 node, 这样 MRT 自己的 executor 不会跟我们抢主 node.
  void initMrt() {
    ocs2InternalNode_ = std::make_shared<rclcpp::Node>(
        std::string(get_name()) + "_ocs2_internal");
    mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>("mobile_manipulator");
    mrt_->initRollout(&interface_->getRollout());
    mrt_->launchNodes(ocs2InternalNode_);
  }

  /// MRT 主循环 (阻塞)
  void run() {
    RCLCPP_INFO(get_logger(), "Waiting for /odom and /joint_states ...");
    rclcpp::Rate wait(10);
    while (rclcpp::ok() && (!gotOdom_ || !gotJs_)) wait.sleep();
    if (!rclcpp::ok()) return;

    // ---- 初始观测 ----
    ocs2::SystemObservation initObs;
    initObs.state.setZero(stateDim_);
    initObs.input.setZero(inputDim_);
    initObs.time = 0.0;
    initObs.mode = 0;
    {
      std::lock_guard<std::mutex> lk(stateMutex_);
      fillStateLocked(initObs.state);
    }

    // ---- 初始 target = 当前末端位姿 (通过 TF), 让 MPC "保持原位" ----
    const ocs2::vector_t initEEPose = lookupCurrentEEPose();
    RCLCPP_INFO(get_logger(),
                "Initial EE target  pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
                initEEPose(0), initEEPose(1), initEEPose(2),
                initEEPose(3), initEEPose(4), initEEPose(5), initEEPose(6));

    ocs2::TargetTrajectories initTargetTrajectories(
        {0.0}, {initEEPose}, {ocs2::vector_t::Zero(inputDim_)});
    mrt_->resetMpcNode(initTargetTrajectories);

    RCLCPP_INFO(get_logger(), "Waiting for first MPC policy ...");
    while (rclcpp::ok() && !mrt_->initialPolicyReceived()) {
      mrt_->setCurrentObservation(initObs);
      mrt_->spinMRT();
      std::this_thread::sleep_for(50ms);
    }
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(get_logger(),
                "Got first policy. Entering MRT control loop @ %.0f Hz",
                mrtRate_);

    // ---- 主循环 ----
    rclcpp::Rate rate(mrtRate_);
    const auto t0 = now();
    while (rclcpp::ok()) {
      mrt_->spinMRT();

      ocs2::SystemObservation obs;
      obs.state.setZero(stateDim_);
      obs.input.setZero(inputDim_);
      obs.time = (now() - t0).seconds();
      obs.mode = 0;
      {
        std::lock_guard<std::mutex> lk(stateMutex_);
        fillStateLocked(obs.state);
      }

      mrt_->setCurrentObservation(obs);
      mrt_->updatePolicy();

      // 当前时刻的 input 直接给底盘
      ocs2::vector_t optState, optInput;
      size_t mode;
      mrt_->evaluatePolicy(obs.time, obs.state, optState, optInput, mode);
      publishBaseCommand(optInput);

      // traj_horizon 之后的 state 给机械臂 (单点, 仅 position)
      publishArmCommand(obs.time, obs.state);

      rate.sleep();
    }
  }

  /// 显式释放 OCS2 资源 (析构顺序敏感, main 退出前调用)
  void shutdownOcs2() {
    mrt_.reset();
    ocs2InternalNode_.reset();
  }

 private:
  // -------- callbacks --------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(stateMutex_);
    baseX_   = msg->pose.pose.position.x;
    baseY_   = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    baseYaw_ = y;
    gotOdom_ = true;
  }

  void jsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(stateMutex_);
    for (size_t i = 0; i < armJointNames_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), armJointNames_[i]);
      if (it != msg->name.end()) {
        const size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) armQ_[i] = msg->position[idx];
      }
    }
    gotJs_ = true;
  }

  // -------- helpers --------
  void fillStateLocked(ocs2::vector_t& state) const {
    state.resize(stateDim_);
    state.setZero();
    state(0) = baseX_;
    state(1) = baseY_;
    state(2) = baseYaw_;
    for (size_t i = 0; i < armDim_ && i < armQ_.size(); ++i)
      state(3 + i) = armQ_[i];
  }

  /// 通过 TF 查 world_frame -> ee_frame, 返回 [pos(3); quat-xyzw(4)]
  ocs2::vector_t lookupCurrentEEPose() {
    // 等 TF tree 就绪 - robot_state_publisher 可能要花 ~1s
    rclcpp::Rate r(10);
    int retry = 30;
    while (rclcpp::ok() && retry-- > 0) {
      if (tf_buffer_->canTransform(worldFrame_, eeFrame_, tf2::TimePointZero,
                                    tf2::durationFromSec(0.1))) {
        break;
      }
      r.sleep();
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
          worldFrame_, eeFrame_, tf2::TimePointZero,
          tf2::durationFromSec(1.0));
      ocs2::vector_t pose(7);
      pose(0) = tf.transform.translation.x;
      pose(1) = tf.transform.translation.y;
      pose(2) = tf.transform.translation.z;
      pose(3) = tf.transform.rotation.x;
      pose(4) = tf.transform.rotation.y;
      pose(5) = tf.transform.rotation.z;
      pose(6) = tf.transform.rotation.w;
      return pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(),
                  "TF lookup %s -> %s failed: %s. Using fallback pose.",
                  worldFrame_.c_str(), eeFrame_.c_str(), ex.what());
      ocs2::vector_t pose(7);
      double bx, by;
      {
        std::lock_guard<std::mutex> lk(stateMutex_);
        bx = baseX_; by = baseY_;
      }
      pose << bx + 0.5, by, 0.5, 0.0, 0.0, 0.0, 1.0;
      return pose;
    }
  }

  void publishBaseCommand(const ocs2::vector_t& input) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = baseFrame_;
    if (input.size() >= 2) {
      msg.twist.linear.x  = input(0);
      msg.twist.angular.z = input(1);
    }
    base_cmd_pub_->publish(msg);
  }

  /// 单点 trajectory, *只* 写 position, 不写 velocity.
  /// 这样 JTC 不会触发 "Velocity of last point is not zero" 检查,
  /// 而是用样条自己插值出速度.
  void publishArmCommand(double currentTime, const ocs2::vector_t& currentState) {
    ocs2::vector_t optState, optInput;
    size_t mode;
    mrt_->evaluatePolicy(currentTime + trajHorizon_, currentState,
                         optState, optInput, mode);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = armJointNames_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(armJointNames_.size());
    for (size_t i = 0; i < armJointNames_.size(); ++i)
      pt.positions[i] = optState(3 + i);
    // velocities 故意留空 -> JTC 自己插值
    pt.time_from_start.sec     = static_cast<int32_t>(trajHorizon_);
    pt.time_from_start.nanosec = static_cast<uint32_t>(
        (trajHorizon_ - static_cast<int>(trajHorizon_)) * 1e9);

    traj.points.push_back(std::move(pt));
    arm_cmd_pub_->publish(traj);
  }

  // -------- members --------
  std::string taskFile_, libFolder_, urdfFile_;
  std::string baseFrame_, worldFrame_, eeFrame_;
  double mrtRate_{100.0}, trajHorizon_{0.05};
  std::vector<std::string> armJointNames_;

  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  rclcpp::Node::SharedPtr ocs2InternalNode_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;

  size_t stateDim_{0}, inputDim_{0}, armDim_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  std::mutex stateMutex_;
  std::atomic<bool> gotOdom_{false}, gotJs_{false};
  double baseX_{0.0}, baseY_{0.0}, baseYaw_{0.0};
  std::vector<double> armQ_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto bridge = std::make_shared<TracerJakaMrtBridge>();

  try {
    bridge->initMrt();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(bridge->get_logger(), "initMrt failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  std::thread spinner([&exec]() {
    try { exec.spin(); } catch (...) {}
  });

  try {
    bridge->run();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(bridge->get_logger(), "MRT loop exception: %s", e.what());
  }

  // 析构顺序: 先停 ROS spinner, 再关 MRT(它会停自己的内部线程), 再 shutdown
  exec.cancel();
  if (spinner.joinable()) spinner.join();
  bridge->shutdownOcs2();
  rclcpp::shutdown();
  return 0;
}

