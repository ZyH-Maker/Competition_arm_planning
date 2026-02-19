#pragma once

// 说明：
// GraspManager 用于封装 MoveIt 抓取相关逻辑，
// 包括：规划、执行、轨迹插值、夹爪控制、物体附着/分离等。

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <robot_interfaces/srv/send_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "main_bt/trajectory_interpolator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <memory>
#include <vector>

class GraspManager {
public:
    // 构造函数：绑定 MoveIt 控制接口与可视化参数
    GraspManager(rclcpp::Node::SharedPtr node,
                 moveit::planning_interface::MoveGroupInterface &move_group,
                 moveit::planning_interface::MoveGroupInterface &gripper_group,
                 const moveit::core::JointModelGroup *joint_model_group,
                 bool enable_visualization = true);

    // 轨迹规划（带插值）：给定关节目标进行规划，并对轨迹进行稠密化
    bool trajPlan(const std::vector<double> &joint_target,
                  moveit::planning_interface::MoveGroupInterface::Plan &plan,
                  int max_attempts = 20);

    // IK 重试规划：给定目标位姿，多次随机种子求 IK 并规划
    bool planWithIKRetry(const geometry_msgs::msg::Pose &target_pose,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan,
                        int max_ik_attempts = 5);

    // 规划回退：优先 CHOMP，失败后改用 OMPL，最后预留备用方案
    bool planWithFallback(const geometry_msgs::msg::Pose &target_pose,
                          moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // 打开夹爪：使用 MoveIt 的命名目标 "open"
    void openGripper();

    // 关闭夹爪：使用 MoveIt 的命名目标 "close"
    void closeGripper();

    // 附加物体到夹爪：用于抓取后将物体绑定到末端
    void attachObject(const std::string &object_name,
                     const std::vector<std::string> &touch_links);

    // 从夹爪分离物体：用于放置后解除物体绑定
    void detachObject(const std::string &object_name);

    // 设置插值参数：每段插值点数量
    void setInterpolationSegments(int segments);

    // 获取 move_group 引用（用于外部执行或设置规划器）
    moveit::planning_interface::MoveGroupInterface& getMoveGroup();

    // 返回 HOME 位置：规划并执行到指定关节角度
    bool goHome(const std::vector<double> &home_joint_values);

    // 读取 Config/home.yaml 并执行 + 发布
    bool home();

    // 读取 Config/target.yaml 并执行 + 发布
    bool target();

private:
    // 发布插值后的轨迹：用于 RViz 可视化
    void publishDenseTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // 对规划轨迹做插值稠密化，并触发可视化发布
    void densifyTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // 发布夹爪状态：用于上位机或调试显示
    void publishGripperStatus(const std::string &status);

    // 读取轨迹文件并执行
    bool executeTrajectoryFile(const std::filesystem::path &path,
                               const std::string &label);
    
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface &move_group_;
    moveit::planning_interface::MoveGroupInterface &gripper_group_;
    const moveit::core::JointModelGroup *joint_model_group_;

    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr dense_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_status_pub_;
    rclcpp::Client<robot_interfaces::srv::SendCommand>::SharedPtr send_command_client_;

    bool enable_visualization_;
    int interpolation_segments_;
    static const rclcpp::Logger LOGGER;
};
