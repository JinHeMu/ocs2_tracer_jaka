# tracer_jaka_gazebo

**ROS 2 Jazzy + Gazebo Harmonic** 仿真包，用于验证基于 **OCS2** 的轮式移动机械臂 (Wheel-Based Mobile Manipulator) MPC 控制器。

机器人 = **AgileX Tracer** 差速底盘 + **Jaka ZU5** (6-DoF) + **DH AG95** 夹爪 + **RealSense D435**。

---

## 1. 与原 `demo.urdf` 的差异

| 项 | 原 `demo.urdf` | 本包 |
|---|---|---|
| 平台 | ROS 1 / Gazebo Classic | ROS 2 Jazzy / Gazebo Harmonic |
| `left_wheel` / `right_wheel` 关节类型 | `fixed` | **`continuous`** |
| 底盘驱动插件 | `libgazebo_ros_diff_drive.so` (Classic) | `gz_ros2_control/GazeboSimSystem` + `diff_drive_controller` |
| 机械臂控制 | `libgazebo_ros_control.so` + `<transmission>` | `gz_ros2_control` + `joint_trajectory_controller` |
| 文件结构 | 单一 `demo.urdf` | xacro 模块化 (`urdf.xacro` / `ros2_control.xacro` / `gazebo.xacro`) |
| 顺手修复 | — | 原 `Link_3` / `Link_4` `xyz` 缺第三位的小 bug |

碰撞模型保持与原文件一致（box / cylinder / sphere），OCS2 self-collision 性能不变。

---

## 2. 安装

```bash
# 系统依赖（Ubuntu 24.04 + ROS 2 Jazzy）
sudo apt install \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro

# 把本包放到工作空间
cd ~/ros2_ws/src
# (把整个 tracer_jaka_gazebo/ 文件夹拷贝到这里)
# 同时确保你的 jaka_tracer_ocs2 包也在这个工作空间，因为 mesh 路径还是
# package://jaka_tracer_ocs2/resource/...

cd ~/ros2_ws
colcon build --symlink-install --packages-select tracer_jaka_gazebo
source install/setup.bash
```

---

## 3. 运行

### 3.1 仅 RViz（先验证 URDF）



```bash
ros2 launch tracer_jaka_gazebo display.launch.py
```

### 3.2 完整 Gazebo 仿真

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(dirname $(ros2 pkg prefix --share jaka_tracer_ocs2))

```bash
ros2 launch tracer_jaka_gazebo gazebo.launch.py
```

启动后会有：

- Gazebo Harmonic GUI（默认空场景 + 一个红色立柱作为视觉 target）
- RViz2（显示 robot model + odom 路径）
- 控制器：
  - `joint_state_broadcaster` 发布 `/joint_states`
  - `diff_drive_controller` 订阅 **`/diff_drive_controller/cmd_vel`**，发布 **`/diff_drive_controller/odom`**
  - `arm_trajectory_controller` 订阅 **`/arm_trajectory_controller/joint_trajectory`**

### 3.3 简单遥控验证

```bash
# 让小车前进


ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_footprint'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" -r 10

# 给机械臂发轨迹（home -> 一个简单姿态）
ros2 topic pub --once /arm_trajectory_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
      points: [{positions: [0.0, 1.57, -1.0, 0.0, 1.0, 0.0],
                time_from_start: {sec: 3}}]}"
```

---

## 4. 与 OCS2 MPC 对接

OCS2 wheel-based mobile manipulator 的「输入」是：

\[
u = \begin{bmatrix} v_{base} \\ \omega_{base} \\ \dot q_1 \dots \dot q_6 \end{bmatrix} \in \mathbb{R}^{8}
\]

「状态」是：

\[
x = \begin{bmatrix} x_{base} \\ y_{base} \\ \theta_{base} \\ q_1 \dots q_6 \end{bmatrix} \in \mathbb{R}^{9}
\]

对接方式：

| OCS2 端 | 本仿真包接收方式 |
|---|---|
| `(v_base, omega_base)` | publish `geometry_msgs/Twist` -> `/diff_drive_controller/cmd_vel` |
| `(q_1 … q_6)` 或 `(q̇_1 … q̇_6)` | publish `trajectory_msgs/JointTrajectory` -> `/arm_trajectory_controller/joint_trajectory`（同时填 `positions` 与 `velocities` 字段） |
| 状态反馈：底盘位姿 | 订阅 `/diff_drive_controller/odom` (nav_msgs/Odometry)，提取 `(x, y, yaw)` |
| 状态反馈：关节角 | 订阅 `/joint_states`（也可用 `/dynamic_joint_states`） |

### 控制器切换：纯速度模式

如果你的 OCS2 直接以 **关节速度** 形式输出（不打包成 trajectory），把 `gazebo.launch.py` 里的

```python
arguments=['arm_trajectory_controller', ...]
```

改为

```python
arguments=['arm_velocity_controller', ...]
```

这时直接 publish `std_msgs/Float64MultiArray` 到 `/arm_velocity_controller/commands`。

---

## 5. 调试技巧

| 现象 | 排查 |
|---|---|
| 机器人加载后立即倒地 | 检查 `base_link` `<inertial>` 是否存在；本包已加默认惯量 |
| `right_wheel`/`left_wheel` 不转 | `ros2 control list_controllers` 看 `diff_drive_controller` 是否 active；用 `ros2 control list_hardware_interfaces` 看 `velocity` command 是否被 claim |
| 机器人在地上抖动 / 滑步 | 万向轮摩擦未生效 → 检查 `tracer_jaka.gazebo.xacro` 里 `mu1=0.001`；或 ground_plane 摩擦设太低 |
| MPC 输出与仿真状态延迟 | `controller_manager.update_rate` 默认 200 Hz，如需 400 Hz：改 `tracer_jaka_controllers.yaml` |
| OCS2 self-collision FCL 报错 | 确保你用的还是简化碰撞体（cylinder/box/sphere），原 `demo.urdf` 设计就是为此 |

---

## 6. 文件结构

```
tracer_jaka_gazebo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   ├── tracer_jaka.urdf.xacro          # 主 URDF（替代原 demo.urdf）
│   ├── tracer_jaka.ros2_control.xacro  # ros2_control 硬件接口
│   └── tracer_jaka.gazebo.xacro        # gz_ros2_control 插件 + 摩擦/自碰撞
├── config/
│   ├── tracer_jaka_controllers.yaml    # 控制器参数
│   └── gz_bridge.yaml                  # ros_gz_bridge 桥接（/clock）
├── launch/
│   ├── gazebo.launch.py                # 主启动：Gazebo + RViz + 全部控制器
│   └── display.launch.py               # 仅 RViz + JSP-GUI
├── worlds/
│   └── empty.world                     # 空场景 + 调试 target_box
└── rviz/
    └── tracer_jaka.rviz
```

---

## 7. 后续扩展建议

- **加相机**：在 `urdf` 里给 `camera_link` 加 `<gazebo>` `<sensor type="camera">` 块，并在 `gz_bridge.yaml` 增 `Image` topic
- **增加场景**：在 `worlds/` 写新的 `.sdf` 或 `.world`，launch 时传 `world:=path/to/your.world`
- **MoveIt 2 对照组**：可与 OCS2 同步跑一个 MoveIt 配置作为参考轨迹源
- **Sim2Real**：把 `sim_mode:=false` 换成你的实机 `hardware_interface` 插件即可复用同一套 URDF 与 controller config
