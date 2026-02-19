cmake_minimum_required(VERSION 3.8)
project(robot_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(serial REQUIRED)

# 包含目录
include_directories(include)

# 创建库
add_library(${PROJECT_NAME}_lib
  # src/trajectory_subscribe.cpp
  src/serial_service_node.cpp
  src/fin_serial.cpp
)

# 库的依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_msgs
  std_msgs
  geometry_msgs
  robot_interfaces
  serial
)

# # 可执行文件：轨迹订阅节点
# add_executable(manipulator_node
#   src/trajectory_subscribe.cpp
# )

# target_link_libraries(manipulator_node
#   ${PROJECT_NAME}_lib
# )

# ament_target_dependencies(manipulator_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   robot_interfaces
#   moveit_msgs
#   serial
# )

# 可执行文件：串口服务节点
add_executable(serial_service_node
  src/serial_service_node.cpp
)

target_link_libraries(serial_service_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(serial_service_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
  moveit_msgs
  serial
)

# 安装目标文件
install(TARGETS
#  manipulator_node
  serial_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# # 安装启动文件
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
