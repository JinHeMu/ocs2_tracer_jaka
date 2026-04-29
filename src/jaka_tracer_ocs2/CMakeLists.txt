cmake_minimum_required(VERSION 3.16)
project(jaka_tracer_ocs2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -O3)

# ---------- 系统依赖 ----------
# OCS2 的 export 文件里把 Boost::system 写进了 INTERFACE_LINK_LIBRARIES，
# 所以下游必须主动 find_package(Boost)，否则 imported target 解析不到。
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(Eigen3 REQUIRED)
find_package(pinocchio QUIET)   # OCS2 间接需要

# ---------- ROS2 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ---------- OCS2 ----------
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_msgs REQUIRED)

# ---------- 可执行文件 ----------
add_executable(${PROJECT_NAME}_node
  src/jaka_tracer_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  ocs2_core
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_mobile_manipulator
  ocs2_msgs
)

# 显式链接 Boost（INTERFACE 链接到 OCS2 的目标会自动传播，
# 但显式写一遍更稳）
target_link_libraries(${PROJECT_NAME}_node
  Boost::system
  Boost::filesystem
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

