// =============================================================================
//  TracerJakaTargetNode.cpp
//
//  把 RViz 的 PoseStamped (e.g. /goal_pose 或自定义 /target_pose)
//  转成 OCS2 TargetTrajectories 发布到 /mobile_manipulator_mpc_target.
//
//  TargetTrajectories.desiredStateTrajectory 编码 (size=7):
//      [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
//  TargetTrajectories.desiredInputTrajectory (size=inputDim):
//      *必须* 是 inputDim 维零向量, 否则 MPC 端算 quadratic input cost
//      会触发 Eigen 尺寸不匹配 assertion.
//      inputDim 不需要硬编码, 我们从 MpcObservation 里读出来.
// =============================================================================

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

class TracerJakaTargetNode : public rclcpp::Node {
 public:
  TracerJakaTargetNode() : Node("tracer_jaka_target_node") {
    declare_parameter<std::string>("robot_name",        "mobile_manipulator");
    declare_parameter<std::string>("target_pose_topic", "/target_pose");
    declare_parameter<double>("default_target_height",  0.5);

    robotName_      = get_parameter("robot_name").as_string();
    defaultHeight_  = get_parameter("default_target_height").as_double();
    const auto topic = get_parameter("target_pose_topic").as_string();

    // 订阅 MPC 的 observation, 顺便记下 inputDim
    obs_sub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        "/" + robotName_ + "_mpc_observation",
        rclcpp::SensorDataQoS(),
        [this](ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          lastObsTime_ = msg->time;
          inputDim_    = msg->input.value.size();
          gotObs_      = true;
        });

    // 订阅用户的目标位姿
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10,
        std::bind(&TracerJakaTargetNode::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Target node ready. Listening on %s, publishing to /%s_mpc_target",
                topic.c_str(), robotName_.c_str());
    RCLCPP_INFO(get_logger(),
                "RViz '2D Goal Pose' z=0; will be lifted to default_target_height=%.2f m",
                defaultHeight_);
  }

  /// 必须在 std::make_shared 之后调用
  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);
  }

 private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double t;
    size_t inputDim;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!gotObs_ || inputDim_ == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "No MPC observation yet (inputDim unknown), dropping target.");
        return;
      }
      t        = lastObsTime_;
      inputDim = inputDim_;
    }

    ocs2::vector_t desiredState(7);
    desiredState(0) = msg->pose.position.x;
    desiredState(1) = msg->pose.position.y;
    // RViz 的 2D Goal Pose 工具把 z 设为 0, 这对末端来说是地面 → 不合理
    // 如果上游 z<=0 就用一个默认抬升高度, 让机械臂去够上面而不是地面
    desiredState(2) = (msg->pose.position.z > 1e-3)
                          ? msg->pose.position.z
                          : defaultHeight_;
    desiredState(3) = msg->pose.orientation.x;
    desiredState(4) = msg->pose.orientation.y;
    desiredState(5) = msg->pose.orientation.z;
    desiredState(6) = msg->pose.orientation.w;

    // ★ 关键: desiredInput 必须是 inputDim 维零向量, *不能* 留空
    ocs2::vector_t desiredInput = ocs2::vector_t::Zero(inputDim);

    ocs2::TargetTrajectories targetTrajectories(
        {t}, {desiredState}, {desiredInput});

    if (targetPub_) {
      targetPub_->publishTargetTrajectories(targetTrajectories);
      RCLCPP_INFO(get_logger(),
                  "Published target  pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
                  desiredState(0), desiredState(1), desiredState(2),
                  desiredState(3), desiredState(4), desiredState(5), desiredState(6));
    }
  }

  std::string robotName_;
  double defaultHeight_{1.0};
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obs_sub_;

  std::mutex mu_;
  double lastObsTime_{0.0};
  size_t inputDim_{0};
  bool   gotObs_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracerJakaTargetNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

