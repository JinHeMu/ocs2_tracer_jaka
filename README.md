# ocs2_tracer_jaka

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

xhost +local:docker

sudo ip link set can0 up type can bitrate 500000


ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py 

ros2 launch point_cloud_processor processor_launch.py 

ros2 run tracer_jaka_ocs2 tracer_jaka_trajectory_target_node 
