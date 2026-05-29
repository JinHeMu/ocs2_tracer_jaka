# tracer_jaka_mujoco

Tracer 底盘 + JAKA Zu5 机械臂 移动机械臂的 **MuJoCo 仿真 + ROS2 控制** 功能包。

---

## 本轮修复了什么

| 你的现象 | 根因 | 修复 |
|---|---|---|
| 速度/关节只能给 -1~1 | 执行器没写 `ctrlrange`，MuJoCo viewer 滑杆默认 [-1,1] | 所有执行器加 `ctrlrange`（轮 ±30 rad/s、各关节按其角度范围） |
| 无法旋转 | 被 ±1 卡死 + 支撑轮悬空导致接触不稳 | 同上 + 脚轮共面（见下） |
| 支撑轮没到地面、机械臂前后晃 | 4 个脚轮球底比驱动轮底高约 1.9 mm，整车只靠 2 驱动轮支撑像跷跷板 | 脚轮球半径 0.04 → 0.0419，使 6 个支撑点共面 |

> 几何核算：驱动轮底 = −0.082 − 0.065 = **−0.147**；原脚轮底 = −0.1051 − 0.04 = −0.1451；
> 改半径后脚轮底 = −0.1051 − 0.0419 = **−0.147**，与驱动轮共面。

（上一轮已做：加 `freejoint`/地面、轮子改 velocity 执行器、关节加 `armature`+`kv`、
修正轮子与 Link_2/Link_3 惯量、`integrator=implicitfast`。）

---

## 目录结构
```
tracer_jaka_mujoco/
├── package.xml / setup.py / setup.cfg
├── tracer_jaka_mujoco/mujoco_bridge_node.py   # 桥接节点（方式A）
├── launch/
│   ├── bridge.launch.py          # 方式A：桥接 + rsp + RViz（推荐先用）
│   └── ros2_control.launch.py    # 方式B：标准 mujoco_ros2_control 栈
├── config/controllers.yaml       # 方式B 控制器配置
├── models/
│   ├── tracer_jaka_zu5_fixed.xml # 修好的 MuJoCo 模型（meshdir="meshes"）
│   └── meshes/                   # ←把你的网格(.obj/.STL/.stl)全部拷进来
├── urdf/
│   ├── tracer_jaka_zu5.urdf
│   └── tracer_jaka_zu5.ros2_control.xacro
└── rviz/view.rviz
```

## ⚠️ 关键一步：放网格
模型里 `meshdir="meshes"`（相对 xml）。把你原来 `meshes/` 目录下的全部网格
（含 `tracer_jaka_gazebo/...` 子目录里的 STL）按相同相对结构拷到 `models/meshes/`，
否则 MuJoCo 加载报 mesh not found。或把 xml 里 `meshdir` 改回你的绝对路径。

---

## 构建
```bash
cd ~/ros2_ws/src
cp -r /path/to/tracer_jaka_mujoco .
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select tracer_jaka_mujoco
source install/setup.bash
pip install mujoco        # 方式A 需要
```

## 运行 —— 方式 A（推荐）
```bash
ros2 launch tracer_jaka_mujoco bridge.launch.py
# 另开终端：
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # 控底盘（前进/转向都可）
ros2 topic pub /arm_command std_msgs/Float64MultiArray "{data: [0,0.5,1.0,0,0.5,0]}"
ros2 topic echo /joint_states
```
弹出 MuJoCo 窗口 + RViz。底盘可前进/后退/原地转向，机械臂平稳到位不晃。

## 运行 —— 方式 B（标准 ros2_control，可接 MoveIt/nav2）
1. 安装插件：`git clone https://github.com/sangteak601/mujoco_ros2_control` 并 `colcon build`。
2. 把 `urdf/tracer_jaka_zu5.ros2_control.xacro` 的 `<ros2_control>` 段合并进
   `urdf/tracer_jaka_zu5.urdf` 的 `</robot>` 之前。
3. `ros2 launch tracer_jaka_mujoco ros2_control.launch.py`
4. 底盘发 `/base_controller/cmd_vel_unstamped`；机械臂用 `/arm_controller` 轨迹或 MoveIt。

> 注意：不同 fork 对 MJCF `<actuator>` 处理不同。若插件自己对关节做 PID，把
> `models/tracer_jaka_zu5_fixed.xml` 的 `<actuator>` 段删除以免双重驱动；若它写
> `mjData.ctrl` 则保留。以你安装版本的 README 为准，建议先试"删除 actuator"。

---

## 仍有问题时的调参顺序
1. 关节抖：加大该关节 `armature`（×2），再加大执行器 `kv`；仍抖减小 `kp`。
2. 关节"软/慢"：增大 `kp`（如 joint_1 1000→2000）。
3. 转向打滑/不灵：脚轮 `friction` 第一项再调小（如 0.02）；驱动轮 `friction` 第一项调大。
4. 整机穿地/发散：`timestep` 减到 0.001。
5. 起步瞬间弹跳：把 `base_link` 初始 z（当前 0.15）微调到刚好落地（约 0.147）。

## 话题接口（方式 A）
- 订阅 `/cmd_vel` (geometry_msgs/Twist)
- 订阅 `/arm_command` (std_msgs/Float64MultiArray，长度 6，单位 rad)
- 发布 `/joint_states` (sensor_msgs/JointState)
- 发布 TF `odom -> base_footprint`
