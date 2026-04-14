#include "main_bt/grasp.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace
{
geometry_msgs::msg::Pose makeTargetPose(double x, double y, double z,
                                        double roll, double pitch, double yaw)
{
  Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

bool saveTransformYaml(const std::filesystem::path& path,
                       const std::string& link_name,
                       const Eigen::Isometry3d& tf)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out.is_open())
  {
    return false;
  }

  const Eigen::Matrix4d mat = tf.matrix();
  out.setf(std::ios::fixed);
  out.precision(6);
  out << "frame_id: base_link\n";
  out << "child_frame_id: " << link_name << "\n";
  out << "matrix:\n";
  for (int r = 0; r < 4; ++r)
  {
    out << "  - [";
    for (int c = 0; c < 4; ++c)
    {
      out << mat(r, c);
      if (c < 3)
      {
        out << ", ";
      }
    }
    out << "]\n";
  }
  return true;
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("record_target_transform_runner", opts);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&](){ exec.spin(); });

  auto get_or_declare = [&node](const std::string& name, const auto& default_value)
  {
    using T = std::decay_t<decltype(default_value)>;
    if (node->has_parameter(name))
    {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name, default_value);
  };

  const std::string arm_group = get_or_declare("arm_group", std::string("arm"));
  const std::string gripper_group = get_or_declare("gripper_group", std::string("hand"));
  const bool enable_visualization = get_or_declare("enable_visualization", true);
  const int interpolation_segments = get_or_declare("interpolation_segments", 10);

  const double target_x = get_or_declare("target_x", -0.17);
  const double target_y = get_or_declare("target_y", -0.114);
  const double target_z = get_or_declare("target_z", 0.1375);
  const double target_roll = get_or_declare("target_roll", 0.0);
  const double target_pitch = get_or_declare("target_pitch", 0.0);
  const double target_yaw = get_or_declare("target_yaw", 0.0);
  const std::string output_yaml = get_or_declare(
    "output_yaml", std::string("/home/fins/robot/src/Config/target_transform.yaml"));

  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  moveit::planning_interface::MoveGroupInterface gripper(node, gripper_group);

  arm.setMaxVelocityScalingFactor(1.0);

  const moveit::core::JointModelGroup* jmg =
    arm.getCurrentState()->getJointModelGroup(arm_group);
  auto grasp_manager = std::make_shared<GraspManager>(
    node, arm, gripper, jmg, enable_visualization);
  grasp_manager->setInterpolationSegments(interpolation_segments);

  const auto target_pose =
    makeTargetPose(target_x, target_y, target_z, target_roll, target_pitch, target_yaw);

  RCLCPP_INFO(node->get_logger(),
              "目标位姿: xyz=(%.4f, %.4f, %.4f), rpy=(%.4f, %.4f, %.4f)",
              target_x, target_y, target_z, target_roll, target_pitch, target_yaw);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!grasp_manager->planWithFallback(target_pose, plan))
  {
    RCLCPP_ERROR(node->get_logger(), "目标位姿规划失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "目标位姿执行失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  auto state = arm.getCurrentState(5.0);
  if (!state)
  {
    RCLCPP_ERROR(node->get_logger(), "执行后获取机械臂状态失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  state->update();

  std::string ee_link = arm.getEndEffectorLink();
  if (ee_link.empty())
  {
    const auto& links = jmg->getLinkModelNames();
    if (links.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "无法确定末端链接名");
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }
    ee_link = links.back();
  }

  const Eigen::Isometry3d tf = state->getGlobalLinkTransform(ee_link);
  if (!saveTransformYaml(output_yaml, ee_link, tf))
  {
    RCLCPP_ERROR(node->get_logger(), "保存变换矩阵失败: %s", output_yaml.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "已保存末端变换矩阵到: %s", output_yaml.c_str());

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
