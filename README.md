# ocs2_tracer_jaka

colcon build --packages-up-to jaka_tracer_ocs2 --symlink-install

ros2 launch jaka_tracer_ocs2 real_robot.launch.py \
    robot_ip:=192.168.10.90 local_ip:=192.168.10.100 can_port:=can0

colcon build --symlink-install --packages-select dh_ag_ros2 jaka_hardware_interface jaka_ros2 tracer_jaka_moveit_config tracer_ros2 ugv_sdk