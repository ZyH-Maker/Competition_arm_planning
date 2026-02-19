#include "main_bt/grasp.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// =========================
// 最小可用版本：用于一次性规划 + 保存 + 回放两条轨迹
// 1) Home -> 固定点位  (规划 + 插值 + 可视化 + 保存)
// 2) 固定点位 -> Home  (规划 + 插值 + 可视化 + 保存)
// 3) 读取保存的两条轨迹并直接执行（不再插值）
// =========================

namespace
{
constexpr double kCylinderHeight = 0.075;
constexpr double kCylinderRadius = 0.025;

// 说明：保存目录（按你的要求固定为工作区 src/Config）
std::filesystem::path configDir()
{
  return std::filesystem::path("/home/fins/robot/src/Config");
}

// 说明：保存轨迹为极简 YAML（只保存关节名和关节轨迹）
bool saveTrajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                    const std::filesystem::path &path)
{
  std::ofstream out(path);
  if (!out.is_open())
  {
    return false;
  }

  out << "joint_names:\n";
  for (const auto &n : traj.joint_names)
  {
    out << "  - " << n << "\n";
  }
  out << "points:\n";
  out.setf(std::ios::fixed);
  out.precision(6);

  for (const auto &pt : traj.points)
  {
    const double t = static_cast<double>(pt.time_from_start.sec) +
                     static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
    out << "  - time_from_start: " << t << "\n";

    auto write_vec = [&out](const char *key, const std::vector<double> &v) {
      out << "    " << key << ": [";
      for (size_t i = 0; i < v.size(); ++i)
      {
        out << v[i];
        if (i + 1 < v.size())
        {
          out << ", ";
        }
      }
      out << "]\n";
    };

    write_vec("positions", pt.positions);
    write_vec("velocities", pt.velocities);
    write_vec("accelerations", pt.accelerations);
    write_vec("effort", pt.effort);
  }

  return true;
}

// 说明：从极简 YAML 加载轨迹（只解析本文件写出的格式）
bool loadTrajectory(const std::filesystem::path &path,
                    trajectory_msgs::msg::JointTrajectory &traj_out)
{
  std::ifstream in(path);
  if (!in.is_open())
  {
    return false;
  }

  std::string line;
  bool in_names = false;
  bool in_points = false;
  trajectory_msgs::msg::JointTrajectoryPoint current;

  auto parse_vec = [](const std::string &l) -> std::vector<double> {
    const auto lb = l.find('[');
    const auto rb = l.find(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb)
    {
      return {};
    }
    std::stringstream ss(l.substr(lb + 1, rb - lb - 1));
    std::string tok;
    std::vector<double> out;
    while (std::getline(ss, tok, ','))
    {
      const auto s = tok.find_first_not_of(" \t\r\n");
      const auto e = tok.find_last_not_of(" \t\r\n");
      if (s == std::string::npos || e == std::string::npos)
      {
        continue;
      }
      out.push_back(std::stod(tok.substr(s, e - s + 1)));
    }
    return out;
  };

  while (std::getline(in, line))
  {
    if (line.find("joint_names:") != std::string::npos)
    {
      in_names = true;
      in_points = false;
      continue;
    }
    if (line.find("points:") != std::string::npos)
    {
      in_names = false;
      in_points = true;
      continue;
    }

    if (in_names)
    {
      const auto dash = line.find("- ");
      if (dash != std::string::npos)
      {
        traj_out.joint_names.push_back(line.substr(dash + 2));
      }
      continue;
    }

    if (in_points)
    {
      if (line.find("- time_from_start:") != std::string::npos)
      {
        if (!current.positions.empty() || !current.velocities.empty() ||
            !current.accelerations.empty() || !current.effort.empty())
        {
          traj_out.points.push_back(current);
          current = trajectory_msgs::msg::JointTrajectoryPoint();
        }
        const auto pos = line.find(":");
        const double t = std::stod(line.substr(pos + 1));
        const int32_t sec = static_cast<int32_t>(std::floor(t));
        const uint32_t nsec =
            static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);
        current.time_from_start.sec = sec;
        current.time_from_start.nanosec = nsec;
        continue;
      }
      if (line.find("positions:") != std::string::npos)
      {
        current.positions = parse_vec(line);
      }
      else if (line.find("velocities:") != std::string::npos)
      {
        current.velocities = parse_vec(line);
      }
      else if (line.find("accelerations:") != std::string::npos)
      {
        current.accelerations = parse_vec(line);
      }
      else if (line.find("effort:") != std::string::npos)
      {
        current.effort = parse_vec(line);
      }
    }
  }

  if (!current.positions.empty() || !current.velocities.empty() ||
      !current.accelerations.empty() || !current.effort.empty())
  {
    traj_out.points.push_back(current);
  }

  return !traj_out.joint_names.empty() && !traj_out.points.empty();
}

// 说明：发布轨迹到 RViz（DisplayTrajectory）
void publishTrajectory(const rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr &pub,
                       const moveit::planning_interface::MoveGroupInterface &move_group,
                       const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (!pub)
  {
    return;
  }
  moveit_msgs::msg::DisplayTrajectory msg;
  auto state = move_group.getCurrentState();
  if (state)
  {
    state->update();
    moveit::core::robotStateToRobotStateMsg(*state, msg.trajectory_start);
  }
  msg.trajectory.emplace_back();
  msg.trajectory.back().joint_trajectory = traj;
  pub->publish(msg);
}

// 说明：固定点位的圆柱体
moveit_msgs::msg::CollisionObject makeFixedCylinder(const geometry_msgs::msg::Pose &pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "base_link";
  obj.id = "test_cylinder";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = kCylinderHeight;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = kCylinderRadius;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

// 说明：把规划起点设置为“上一条轨迹的终点”
void setStartFromTrajectoryEnd(moveit::planning_interface::MoveGroupInterface &move_group,
                               const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.empty())
  {
    return;
  }

  auto state_ptr = move_group.getCurrentState();
  if (!state_ptr)
  {
    return;
  }

  moveit::core::RobotState start_state(*state_ptr);
  const auto *jmg = move_group.getRobotModel()->getJointModelGroup(move_group.getName());
  if (!jmg)
  {
    return;
  }

  std::vector<double> group_positions(jmg->getVariableCount(), 0.0);
  const auto &group_names = jmg->getVariableNames();
  const auto &last = traj.points.back();
  for (size_t i = 0; i < group_names.size(); ++i)
  {
    for (size_t j = 0; j < traj.joint_names.size(); ++j)
    {
      if (group_names[i] == traj.joint_names[j] && j < last.positions.size())
      {
        group_positions[i] = last.positions[j];
        break;
      }
    }
  }

  start_state.setJointGroupPositions(jmg, group_positions);
  move_group.setStartState(start_state);
}
}  // namespace

int main(int argc, char **argv)
{
  // 说明：初始化 ROS2
  rclcpp::init(argc, argv);

  // 说明：创建节点并启动 executor
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("test_trajectory_runner", opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&](){ exec.spin(); });

  // 说明：MoveIt 组名参数
  const std::string arm_group = node->declare_parameter<std::string>("arm_group", "arm");
  const std::string gripper_group = node->declare_parameter<std::string>("gripper_group", "hand");
  const bool enable_visualization = node->declare_parameter<bool>("enable_visualization", true);
  const int interpolation_segments =
    node->declare_parameter<int>("interpolation_segments", 10);

  // 说明：MoveIt 接口
  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  moveit::planning_interface::MoveGroupInterface gripper(node, gripper_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(0.1);

  const moveit::core::JointModelGroup *jmg =
    arm.getCurrentState()->getJointModelGroup(arm_group);
  auto grasp_manager = std::make_shared<GraspManager>(
    node, arm, gripper, jmg, enable_visualization);
  grasp_manager->setInterpolationSegments(interpolation_segments);

  auto display_pub =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_dense_path", 10);

  // 说明：记录当前关节值当作 HOME（默认启动就在 HOME）
  std::vector<double> home_joints;
  {
    auto state = arm.getCurrentState();
    if (state)
    {
      state->copyJointGroupPositions(jmg, home_joints);
    }
  }

  // 说明：固定点位（你只需要改这里）
  geometry_msgs::msg::Pose fixed_pose;
  fixed_pose.orientation.w = 1.0;
  fixed_pose.position.x = -0.17;
  fixed_pose.position.y = -0.114;
  fixed_pose.position.z = 0.10+kCylinderHeight/2;

  // 说明：添加圆柱体
  planning_scene.applyCollisionObject(makeFixedCylinder(fixed_pose));
  auto grasp_pose = fixed_pose;
  grasp_pose.position.z += 0.2;
  Eigen::Vector3d axis_vec(0.0, 0.0, 1.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI*3/4, axis_vec));
  grasp_pose.orientation.x = quat.x();
  grasp_pose.orientation.y = quat.y();
  grasp_pose.orientation.z = quat.z();
  grasp_pose.orientation.w = quat.w();
  // 说明：规划 Home -> 固定点位
  moveit::planning_interface::MoveGroupInterface::Plan plan_a;
  if (!grasp_manager->planWithFallback(grasp_pose, plan_a))
  {
    RCLCPP_ERROR(node->get_logger(), "规划 回零 -> 目标 失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // 说明：保存第一条轨迹
  std::filesystem::create_directories(configDir());
  const auto file_a = configDir() / "plan_home_to_target.yaml";
  saveTrajectory(plan_a.trajectory_.joint_trajectory, file_a);
  RCLCPP_INFO(node->get_logger(), "已保存: %s", file_a.c_str());

  // 说明：下一次规划从第一条轨迹的末端开始
  setStartFromTrajectoryEnd(arm, plan_a.trajectory_.joint_trajectory);

  // 说明：规划 固定点位 -> Home
  moveit::planning_interface::MoveGroupInterface::Plan plan_b;
  if (!grasp_manager->trajPlan(home_joints, plan_b))
  {
    RCLCPP_ERROR(node->get_logger(), "规划 目标 -> 回零 失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // 说明：保存第二条轨迹
  const auto file_b = configDir() / "plan_target_to_home.yaml";
  saveTrajectory(plan_b.trajectory_.joint_trajectory, file_b);
  RCLCPP_INFO(node->get_logger(), "已保存: %s", file_b.c_str());

  // 说明：读取保存的轨迹并直接执行（不插值）
  trajectory_msgs::msg::JointTrajectory traj_a;
  trajectory_msgs::msg::JointTrajectory traj_b;
  if (!loadTrajectory(file_a, traj_a) || !loadTrajectory(file_b, traj_b))
  {
    RCLCPP_ERROR(node->get_logger(), "加载已保存的轨迹失败");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // 说明：执行第一条轨迹
  publishTrajectory(display_pub, arm, traj_a);
  moveit::planning_interface::MoveGroupInterface::Plan exec_a;
  exec_a.trajectory_.joint_trajectory = traj_a;
  arm.execute(exec_a);

  // 说明：执行第二条轨迹
  publishTrajectory(display_pub, arm, traj_b);
  moveit::planning_interface::MoveGroupInterface::Plan exec_b;
  exec_b.trajectory_.joint_trajectory = traj_b;
  arm.execute(exec_b);

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
