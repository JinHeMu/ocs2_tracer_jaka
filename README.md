# ocs2_tracer_jaka

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

xhost +local:docker

sudo ip link set can0 up type can bitrate 500000