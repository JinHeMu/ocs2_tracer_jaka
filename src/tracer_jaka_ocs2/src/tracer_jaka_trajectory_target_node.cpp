// =============================================================================
//  path_trajectory_target_node.cpp
//
//  把一条已经规划好的 CSV 路径 (x, y, z, nx, ny, nz) 转换成
//  OCS2 的 TargetTrajectories 并一次性发布出去，
//  让 OCS2 (MPC + 全身控制 WBC) 自己完成移动机械臂的末端轨迹跟踪。
//
//  本版本的关键修改
//  --------------------------------------------------------------------------
//   - 不再使用“沿轨迹方向”的切线向量构造姿态。
//   - 姿态只依赖法向量 nx, ny, nz。
//   - 工具 Z 轴对齐法向量。
//   - 工具 X 轴通过“上一帧 X 轴投影到当前法平面”保持连续。
//   - 第一个点使用固定参考轴 reference_x_axis 初始化。
//   - 因此 CSV 只需要 x,y,z,nx,ny,nz，不需要 dx,dy,dz，也不需要路径切线。
//
//  姿态构造说明
//  --------------------------------------------------------------------------
//   - OCS2 target 需要完整四元数 [qx, qy, qz, qw]。
//   - 但法向量只定义了工具 Z 轴，绕 Z 的自旋是欠定的。
//   - 本版本不使用路径切线来决定这个自旋，而是使用连续投影法：
//
//       第一个点:
//         X = reference_x_axis 在法平面上的投影
//
//       后续点:
//         X = 上一个点的 X 轴在当前法平面上的投影
//
//       Z = normal
//       Y = Z × X
//
//   - 这样可以避免姿态绕 Z 轴乱跳，同时不依赖轨迹方向。
//
//  与示例 joy 节点保持一致
//  --------------------------------------------------------------------------
//     robot_name = "mobile_manipulator"
//     target     = [x, y, z, qx, qy, qz, qw]   (7 维)
//     input_dim  = 8  (2 底盘 + 6 臂)
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

// OCS2 MPC 观测消息，用来对齐参考轨迹的时间基准
#include <ocs2_msgs/msg/mpc_observation.hpp>


class PathTrajectoryTargetNode : public rclcpp::Node {
 public:
  PathTrajectoryTargetNode() : Node("path_trajectory_target_node") {
    // ── 基础参数 ──────────────────────────────────────────
    declare_parameter<std::string>("csv_file",
        "/home/a/ocs2_tracer_jaka/outputs/coverage_path.csv");
    declare_parameter<std::string>("robot_name",   "mobile_manipulator");
    declare_parameter<std::string>("marker_frame", "odom");   // 参考轨迹所在坐标系
    declare_parameter<std::string>("ee_frame",     "tool0");  // 末端 TF 帧
    declare_parameter<int>("input_dim",            8);        // 2 底盘 + 6 臂

    // ── 姿态参数 ──────────────────────────────────────────
    // 不再使用轨迹切线方向。
    // 第一个点的工具 X 轴由 reference_x_axis 投影到法平面得到。
    // 后续点使用上一帧 X 轴投影到当前法平面，保证绕法向量方向连续。
    declare_parameter<std::vector<double>>(
        "reference_x_axis", std::vector<double>{1.0, 0.0, 0.0});

    // 接触类工具，例如打磨 / 喷涂 / 擦拭，常常希望工具 Z 轴朝向表面内侧，
    // 即与外法向量相反，这时把 flip_normal 设为 true。
    declare_parameter<bool>("flip_normal", false);

    // ── 时间参数化 ─────────────────────────────────────────
    declare_parameter<double>("linear_speed",  0.10);  // [m/s]   末端沿路径线速度
    declare_parameter<double>("angular_speed", 0.50);  // [rad/s] 姿态调整角速度
    declare_parameter<double>("min_dt",        0.05);  // [s]     相邻点最小时间间隔
    declare_parameter<double>("start_lead",    0.0);   // [s]     第一个点的提前量

    // ── 行为参数 ──────────────────────────────────────────
    // true: 收到 MPC 观测后自动发布一次
    // false: 等待 /path_trajectory/publish 服务触发
    declare_parameter<bool>("auto_publish", true);

    // 是否在轨迹最前面插入“当前末端位姿”作为起点，
    // 让 MPC 从当前位置平滑过渡到第一个路点。
    declare_parameter<bool>("prepend_current_pose", true);

    // ── 可视化 ────────────────────────────────────────────
    declare_parameter<std::string>("pose_array_topic", "path_target_poses");

    // ── 读取参数 ──────────────────────────────────────────
    csvFile_       = get_parameter("csv_file").as_string();
    robotName_     = get_parameter("robot_name").as_string();
    markerFrame_   = get_parameter("marker_frame").as_string();
    eeFrame_       = get_parameter("ee_frame").as_string();
    inputDim_      = get_parameter("input_dim").as_int();

    vLin_          = get_parameter("linear_speed").as_double();
    wAng_          = get_parameter("angular_speed").as_double();
    minDt_         = get_parameter("min_dt").as_double();
    startLead_     = get_parameter("start_lead").as_double();
    flipNormal_    = get_parameter("flip_normal").as_bool();

    autoPublish_    = get_parameter("auto_publish").as_bool();
    prependCurrent_ = get_parameter("prepend_current_pose").as_bool();

    poseArrayTopic_ = get_parameter("pose_array_topic").as_string();

    const auto refAxisParam = get_parameter("reference_x_axis").as_double_array();
    if (refAxisParam.size() == 3) {
      referenceXAxis_ = Eigen::Vector3d(
          refAxisParam[0], refAxisParam[1], refAxisParam[2]);
      if (referenceXAxis_.norm() < 1e-6) {
        RCLCPP_WARN(get_logger(),
            "reference_x_axis 接近零向量，自动改为 [1, 0, 0]");
        referenceXAxis_ = Eigen::Vector3d::UnitX();
      } else {
        referenceXAxis_.normalize();
      }
    } else {
      RCLCPP_WARN(get_logger(),
          "reference_x_axis 参数长度不是 3，自动改为 [1, 0, 0]");
      referenceXAxis_ = Eigen::Vector3d::UnitX();
    }

    if (vLin_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "linear_speed <= 0，强制设为 0.1 m/s");
      vLin_ = 0.1;
    }

    if (wAng_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "angular_speed <= 0，强制设为 0.5 rad/s");
      wAng_ = 0.5;
    }

    if (minDt_ <= 1e-6) {
      RCLCPP_WARN(get_logger(), "min_dt <= 0，强制设为 0.05 s");
      minDt_ = 0.05;
    }

    // ── TF ────────────────────────────────────────────────
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 加载并预处理路径 ──────────────────────────────────
    if (!loadCsv(csvFile_)) {
      RCLCPP_ERROR(get_logger(), "读取 CSV 失败或路径为空，节点无法工作。");
    } else {
      buildPosesWithoutTrajectoryDirection();
      RCLCPP_INFO(get_logger(),
          "成功加载并构造 %zu 个目标位姿。", positions_.size());
    }

    // ── 订阅 MPC 观测，获取参考轨迹的时间基准 ──────────────
    const std::string obsTopic = robotName_ + "_mpc_observation";
    obsSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        obsTopic,
        rclcpp::QoS(1),
        [this](ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          latestObsTime_   = msg->time;
          haveObservation_ = true;
        });

    RCLCPP_INFO(get_logger(),
        "订阅 MPC 观测话题以对齐时间基准: %s", obsTopic.c_str());

    // ── 可视化发布者，transient_local 方便 RViz 后启动也能收到 ──
    const auto viz_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    poseArrayPub_ =
        create_publisher<geometry_msgs::msg::PoseArray>(poseArrayTopic_, viz_qos);
  }

  // TargetTrajectoriesRosPublisher 需要 shared_from_this()，放到 init() 里
  void init() {
    targetPub_ = std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
        shared_from_this(), robotName_);

    // 触发发布的服务
    publishSrv_ = create_service<std_srvs::srv::Trigger>(
        "/path_trajectory/publish",
        std::bind(&PathTrajectoryTargetNode::onPublishService, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 先把可视化发出去，方便在 RViz 里检查路径
    publishPoseArrayViz();

    if (autoPublish_) {
      // 等到收到 MPC 观测后自动发布一次
      autoTimer_ = create_wall_timer(
          std::chrono::milliseconds(200),
          std::bind(&PathTrajectoryTargetNode::onAutoTimer, this));
    }

    RCLCPP_INFO(get_logger(),
        "节点就绪。robot=%s frame=%s 路点数=%zu v=%.3f m/s w=%.3f rad/s",
        robotName_.c_str(), markerFrame_.c_str(),
        positions_.size(), vLin_, wAng_);

    RCLCPP_INFO(get_logger(),
        "姿态构造方式: Z 轴对齐法向量，X 轴连续投影，不使用路径切线方向。");

    RCLCPP_INFO(get_logger(),
        "调用服务 /path_trajectory/publish 发布轨迹，或将 auto_publish 设为 true。");
  }

 private:
  // ========================================================================
  // 1) 读取 CSV
  // ========================================================================
  bool loadCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "无法打开 CSV 文件: %s", path.c_str());
      return false;
    }

    std::string line;
    if (!std::getline(f, line)) {
      RCLCPP_ERROR(get_logger(), "CSV 文件为空: %s", path.c_str());
      return false;
    }

    // 解析表头，找到 x,y,z,nx,ny,nz 的列索引。
    // 允许 CSV 有多余列，例如 dx,dy,dz，但本版本不会使用它们。
    auto header = splitCsv(line);

    auto col = [&](const std::string& name) -> int {
      for (size_t i = 0; i < header.size(); ++i) {
        std::string h = header[i];

        // 去掉可能的空白
        h.erase(std::remove_if(h.begin(), h.end(),
                               [](unsigned char c) { return std::isspace(c); }),
                h.end());

        if (h == name) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };

    const int ix  = col("x");
    const int iy  = col("y");
    const int iz  = col("z");
    const int inx = col("nx");
    const int iny = col("ny");
    const int inz = col("nz");

    if (ix < 0 || iy < 0 || iz < 0 || inx < 0 || iny < 0 || inz < 0) {
      RCLCPP_ERROR(get_logger(),
          "CSV 表头缺少必要列，需包含 x,y,z,nx,ny,nz。");
      return false;
    }

    positions_.clear();
    normals_.clear();

    while (std::getline(f, line)) {
      if (line.empty()) {
        continue;
      }

      auto cells = splitCsv(line);
      const int maxIdx = std::max({ix, iy, iz, inx, iny, inz});

      if (static_cast<int>(cells.size()) <= maxIdx) {
        RCLCPP_WARN(get_logger(), "跳过列数不足的一行。");
        continue;
      }

      try {
        Eigen::Vector3d p(
            std::stod(cells[ix]),
            std::stod(cells[iy]),
            std::stod(cells[iz]));

        Eigen::Vector3d n(
            std::stod(cells[inx]),
            std::stod(cells[iny]),
            std::stod(cells[inz]));

        const double nn = n.norm();
        if (nn > 1e-6) {
          n /= nn;
        } else {
          RCLCPP_WARN(get_logger(), "检测到零法向量，默认使用 [0,0,1]。");
          n = Eigen::Vector3d::UnitZ();
        }

        if (flipNormal_) {
          n = -n;
        }

        positions_.push_back(p);
        normals_.push_back(n);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "跳过无法解析的一行: %s", e.what());
      }
    }

    return !positions_.empty();
  }

  static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;

    while (std::getline(ss, cell, ',')) {
      out.push_back(cell);
    }

    return out;
  }

  // ========================================================================
  // 2) 工具函数：把 preferred_x 投影到法平面，得到合法的工具 X 轴
  // ========================================================================
  Eigen::Vector3d projectXAxisToNormalPlane(
      const Eigen::Vector3d& preferred_x,
      const Eigen::Vector3d& z) const {
    Eigen::Vector3d x = preferred_x - preferred_x.dot(z) * z;

    if (x.norm() < 1e-6) {
      // preferred_x 恰好和 z 平行时，换一个固定参考轴。
      Eigen::Vector3d ref =
          (std::abs(z.x()) < 0.9) ? Eigen::Vector3d::UnitX()
                                  : Eigen::Vector3d::UnitY();

      x = ref - ref.dot(z) * z;
    }

    if (x.norm() < 1e-6) {
      // 理论上很少进入这里，作为最终保护。
      Eigen::Vector3d ref =
          (std::abs(z.z()) < 0.9) ? Eigen::Vector3d::UnitZ()
                                  : Eigen::Vector3d::UnitY();

      x = ref - ref.dot(z) * z;
    }

    x.normalize();
    return x;
  }

  // ========================================================================
  // 3) 由 [位置 + 法向量] 构造完整姿态四元数
  //
  //    重要：
  //    本函数不使用相邻点差分，不使用路径切线方向。
  // ========================================================================
  void buildPosesWithoutTrajectoryDirection() {
    const size_t N = positions_.size();
    quats_.assign(N, Eigen::Quaterniond::Identity());

    if (N == 0) {
      return;
    }

    Eigen::Vector3d prevX = referenceXAxis_;

    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d z = normals_[i];

      if (z.norm() < 1e-6) {
        z = Eigen::Vector3d::UnitZ();
      } else {
        z.normalize();
      }

      // 第一个点:
      //   X 轴由 reference_x_axis 投影到法平面。
      //
      // 后续点:
      //   X 轴由上一帧 X 轴投影到当前法平面。
      //
      // 这样可以保证绕 Z 轴的自旋连续，
      // 并且完全不依赖轨迹方向。
      Eigen::Vector3d x = projectXAxisToNormalPlane(prevX, z);

      Eigen::Vector3d y = z.cross(x);

      if (y.norm() < 1e-6) {
        // 极端保护，重新选择 X 轴。
        x = projectXAxisToNormalPlane(referenceXAxis_, z);
        y = z.cross(x);
      }

      y.normalize();

      // 重新正交化 X，减少数值误差。
      x = y.cross(z);
      x.normalize();

      Eigen::Matrix3d Rm;
      Rm.col(0) = x;
      Rm.col(1) = y;
      Rm.col(2) = z;

      Eigen::Quaterniond q(Rm);
      q.normalize();

      // 保证四元数连续，避免相邻点 q 和 -q 跳变。
      if (i > 0 && q.coeffs().dot(quats_[i - 1].coeffs()) < 0.0) {
        q.coeffs() *= -1.0;
      }

      quats_[i] = q;
      prevX = x;
    }
  }

  // ========================================================================
  // 4) TF: 查询当前末端位姿
  // ========================================================================
  std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> lookupCurrentEE() {
    try {
      auto tf = tf_buffer_->lookupTransform(
          markerFrame_,
          eeFrame_,
          tf2::TimePointZero,
          tf2::durationFromSec(0.1));

      Eigen::Vector3d p(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z);

      Eigen::Quaterniond q(
          tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);

      q.normalize();

      return std::make_pair(p, q);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "查询当前末端位姿失败: %s", ex.what());
      return std::nullopt;
    }
  }

  // ========================================================================
  // 5) 构造并发布 TargetTrajectories
  // ========================================================================
  bool buildAndPublishTrajectory() {
    if (positions_.empty()) {
      RCLCPP_ERROR(get_logger(), "没有可用路点。");
      return false;
    }

    if (!targetPub_) {
      RCLCPP_ERROR(get_logger(), "targetPub_ 尚未初始化。");
      return false;
    }

    // 时间基准：对齐 MPC 观测时间
    double t0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!haveObservation_) {
        RCLCPP_WARN(get_logger(),
            "尚未收到 MPC 观测，时间基准回退为 0.0。"
            "若 MPC 已运行，建议确认 %s_mpc_observation 话题正常。",
            robotName_.c_str());
        t0 = 0.0;
      } else {
        t0 = latestObsTime_;
      }
    }

    // 组装位姿与相对时间序列
    std::vector<Eigen::Vector3d>    posSeq;
    std::vector<Eigen::Quaterniond> quatSeq;
    std::vector<double>             timeSeq;

    posSeq.reserve(positions_.size() + 1);
    quatSeq.reserve(positions_.size() + 1);
    timeSeq.reserve(positions_.size() + 1);

    double cumT = startLead_;

    // 可选：把当前末端位姿作为第 0 个点，
    // 从当前位置平滑过渡到第一个路点。
    if (prependCurrent_) {
      if (auto cur = lookupCurrentEE()) {
        posSeq.push_back(cur->first);
        quatSeq.push_back(cur->second);
        timeSeq.push_back(cumT);

        const double d   = (positions_[0] - cur->first).norm();
        const double ang = cur->second.angularDistance(quats_[0]);

        cumT += std::max({d / vLin_, ang / wAng_, minDt_});
      }
    }

    // 第一个 CSV 路点
    posSeq.push_back(positions_[0]);
    quatSeq.push_back(quats_[0]);
    timeSeq.push_back(cumT);

    // 其余路点：仍然使用相邻点之间的距离来分配时间。
    // 注意：这里的相邻点距离只用于时间参数化，不用于姿态方向构造。
    for (size_t i = 1; i < positions_.size(); ++i) {
      const double d   = (positions_[i] - positions_[i - 1]).norm();
      const double ang = quats_[i - 1].angularDistance(quats_[i]);

      cumT += std::max({d / vLin_, ang / wAng_, minDt_});

      posSeq.push_back(positions_[i]);
      quatSeq.push_back(quats_[i]);
      timeSeq.push_back(cumT);
    }

    // 打包成 OCS2 TargetTrajectories
    ocs2::scalar_array_t timeTraj;
    ocs2::vector_array_t stateTraj;
    ocs2::vector_array_t inputTraj;

    const size_t M = posSeq.size();

    timeTraj.reserve(M);
    stateTraj.reserve(M);
    inputTraj.reserve(M);

    for (size_t i = 0; i < M; ++i) {
      timeTraj.push_back(t0 + timeSeq[i]);

      // state = [x, y, z, qx, qy, qz, qw]
      // Eigen::Quaterniond::coeffs() = [qx, qy, qz, qw]
      ocs2::vector_t s(7);
      s << posSeq[i], quatSeq[i].coeffs();

      stateTraj.push_back(std::move(s));

      // 期望输入设为 0，维度 = input_dim
      inputTraj.push_back(ocs2::vector_t::Zero(inputDim_));
    }

    ocs2::TargetTrajectories tt(
        std::move(timeTraj),
        std::move(stateTraj),
        std::move(inputTraj));

    targetPub_->publishTargetTrajectories(tt);

    RCLCPP_INFO(get_logger(),
        "已发布轨迹: %zu 个关键点, t0=%.3f, 总时长≈%.2f s。",
        M, t0, timeSeq.empty() ? 0.0 : timeSeq.back());

    publishPoseArrayViz();

    return true;
  }

  // ========================================================================
  // 6) 可视化
  // ========================================================================
  void publishPoseArrayViz() {
    if (positions_.empty() || quats_.size() != positions_.size()) {
      return;
    }

    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = now();
    pa.header.frame_id = markerFrame_;

    pa.poses.reserve(positions_.size());

    for (size_t i = 0; i < positions_.size(); ++i) {
      geometry_msgs::msg::Pose p;

      p.position.x = positions_[i].x();
      p.position.y = positions_[i].y();
      p.position.z = positions_[i].z();

      p.orientation.x = quats_[i].x();
      p.orientation.y = quats_[i].y();
      p.orientation.z = quats_[i].z();
      p.orientation.w = quats_[i].w();

      pa.poses.push_back(p);
    }

    poseArrayPub_->publish(pa);
  }

  // ========================================================================
  // 回调
  // ========================================================================
  void onPublishService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    const bool ok = buildAndPublishTrajectory();
    res->success = ok;
    res->message = ok ? "Trajectory published" : "Publish failed";
  }

  void onAutoTimer() {
    bool ready;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ready = haveObservation_;
    }

    if (!ready) {
      return;
    }

    buildAndPublishTrajectory();

    // 只自动发布一次
    autoTimer_->cancel();
  }

  // ========================================================================
  // 成员变量
  // ========================================================================
  std::string csvFile_;
  std::string robotName_;
  std::string markerFrame_;
  std::string eeFrame_;
  int inputDim_{8};

  double vLin_{0.1};
  double wAng_{0.5};
  double minDt_{0.05};
  double startLead_{0.0};

  bool flipNormal_{false};
  bool autoPublish_{false};
  bool prependCurrent_{true};

  std::string poseArrayTopic_;

  // 第一个姿态的工具 X 轴参考方向。
  // 后续姿态不再使用路径切线，而是使用上一帧 X 轴连续投影。
  Eigen::Vector3d referenceXAxis_{Eigen::Vector3d::UnitX()};

  // 路径数据
  std::vector<Eigen::Vector3d>    positions_;
  std::vector<Eigen::Vector3d>    normals_;
  std::vector<Eigen::Quaterniond> quats_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // OCS2
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPub_;

  // MPC 观测
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  std::mutex mtx_;
  double latestObsTime_{0.0};
  bool haveObservation_{false};

  // 服务 / 定时器 / 可视化
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publishSrv_;
  rclcpp::TimerBase::SharedPtr autoTimer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poseArrayPub_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PathTrajectoryTargetNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
