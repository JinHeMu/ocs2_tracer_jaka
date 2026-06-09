cmake_minimum_required(VERSION 3.8)
project(force_admittance_servo)

# ─── 编译选项 ─────────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

# ─── 依赖 ─────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)



set(DEPS
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

# ─── 库（可选：供 composable node 使用）────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/force_admittance_servo_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})

# ─── 可执行节点 ────────────────────────────────────────────────────────────────
add_executable(force_admittance_servo_node src/main.cpp)
target_link_libraries(force_admittance_servo_node ${PROJECT_NAME})
ament_target_dependencies(force_admittance_servo_node ${DEPS})
target_include_directories(force_admittance_servo_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ─── 安装 ─────────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}
  force_admittance_servo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# ─── 测试 ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

