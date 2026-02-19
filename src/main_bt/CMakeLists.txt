cmake_minimum_required(VERSION 3.8)
project(main_bt)

# ==============================
# 基础编译选项
# ==============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17（MoveIt2 推荐）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================
# 查找依赖
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ==============================
# 头文件路径
# ==============================
include_directories(
  include
)

# ==============================
# 轨迹插值库
# ==============================
add_library(trajectory_interpolator_lib
  src/trajectory_interpolator.cpp
)
target_include_directories(trajectory_interpolator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_interpolator_lib
  trajectory_msgs
)

# ==============================
# GraspManager 库
# ==============================
add_library(grasp_lib
  src/grasp.cpp
)
target_include_directories(grasp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(grasp_lib
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
  moveit_ros_planning_interface
  moveit_msgs
  trajectory_msgs
)
target_link_libraries(grasp_lib
  trajectory_interpolator_lib
)

# ==============================
# BT 节点库
# ==============================
add_library(bt_nodes_lib
  src/bt_nodes_motion.cpp
  src/bt_nodes_comm.cpp
  src/bt_nodes_main.cpp
  src/communication.cpp
)
target_include_directories(bt_nodes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_nodes_lib
  rclcpp
  std_msgs
  geometry_msgs
  shape_msgs
  robot_interfaces
  behaviortree_cpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
)
target_link_libraries(bt_nodes_lib
  Eigen3::Eigen
  grasp_lib
  trajectory_interpolator_lib
)

# ==============================
# BT Runner 可执行程序
# ==============================
add_executable(bt_runner
  src/bt_runner.cpp
)
ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_cpp
  moveit_ros_planning_interface
  moveit_visual_tools
  ament_index_cpp
)
target_link_libraries(bt_runner
  bt_nodes_lib
  grasp_lib
  trajectory_interpolator_lib
)

# ==============================
# 测试轨迹录制/回放
# ==============================
add_executable(test_trajectory
  src/test.cpp
)
ament_target_dependencies(test_trajectory
  rclcpp
  std_msgs
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
)
target_link_libraries(test_trajectory
  grasp_lib
)

# ==============================
# 安装规则
# ==============================
install(
  TARGETS
    bt_runner
    test_trajectory
    bt_nodes_lib
    grasp_lib
    trajectory_interpolator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ==============================
# ament 导出
# ==============================
ament_export_include_directories(include)
ament_export_libraries(grasp_lib trajectory_interpolator_lib bt_nodes_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  behaviortree_cpp
  robot_interfaces
)

ament_package()
