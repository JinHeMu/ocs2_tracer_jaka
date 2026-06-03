cmake_minimum_required(VERSION 3.8)
project(point_cloud_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 接口必须生成到当前 ROS 包名下
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSaving.srv"
  "srv/GetSaverStatus.srv"
  "srv/CropPointCloud.srv"
  "action/ProcessPointCloud.action"
  "action/PlanCoveragePath.action"
  DEPENDENCIES
    std_msgs
    action_msgs
    sensor_msgs
)

# 你的 Python 代码放到单独的 Python 包目录里
ament_python_install_package(point_cloud_processor_py)

install(PROGRAMS
  point_cloud_processor_py/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME point_cloud_processor_node
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  point_cloud_processor_py/visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualizer_node
)
install(PROGRAMS
  point_cloud_processor_py/pcd_saver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pcd_saver_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
