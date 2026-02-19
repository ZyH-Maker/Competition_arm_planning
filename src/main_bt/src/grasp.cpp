#include "main_bt/grasp.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

const rclcpp::Logger GraspManager::LOGGER = rclcpp::get_logger("grasp_manager");

namespace {
std::filesystem::path configDir() {
    return std::filesystem::path("/home/fins/robot/src/Config");
}

constexpr uint8_t kCmdGripperClose = 4;
constexpr uint8_t kCmdGripperOpen = 5;

bool loadTrajectory(const std::filesystem::path &path,
                    trajectory_msgs::msg::JointTrajectory &traj_out) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::string line;
    bool in_names = false;
    bool in_points = false;
    trajectory_msgs::msg::JointTrajectoryPoint current;

    auto parse_vec = [](const std::string &l) -> std::vector<double> {
        const auto lb = l.find('[');
        const auto rb = l.find(']');
        if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
            return {};
        }
        std::stringstream ss(l.substr(lb + 1, rb - lb - 1));
        std::string tok;
        std::vector<double> out;
        while (std::getline(ss, tok, ',')) {
            const auto s = tok.find_first_not_of(" \t\r\n");
            const auto e = tok.find_last_not_of(" \t\r\n");
            if (s == std::string::npos || e == std::string::npos) {
                continue;
            }
            out.push_back(std::stod(tok.substr(s, e - s + 1)));
        }
        return out;
    };

    while (std::getline(in, line)) {
        if (line.find("joint_names:") != std::string::npos) {
            in_names = true;
            in_points = false;
            continue;
        }
        if (line.find("points:") != std::string::npos) {
            in_names = false;
            in_points = true;
            continue;
        }

        if (in_names) {
            const auto dash = line.find("- ");
            if (dash != std::string::npos) {
                traj_out.joint_names.push_back(line.substr(dash + 2));
            }
            continue;
        }

        if (in_points) {
            if (line.find("- time_from_start:") != std::string::npos) {
                if (!current.positions.empty() || !current.velocities.empty() ||
                    !current.accelerations.empty() || !current.effort.empty()) {
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
            if (line.find("positions:") != std::string::npos) {
                current.positions = parse_vec(line);
            } else if (line.find("velocities:") != std::string::npos) {
                current.velocities = parse_vec(line);
            } else if (line.find("accelerations:") != std::string::npos) {
                current.accelerations = parse_vec(line);
            } else if (line.find("effort:") != std::string::npos) {
                current.effort = parse_vec(line);
            }
        }
    }

    if (!current.positions.empty() || !current.velocities.empty() ||
        !current.accelerations.empty() || !current.effort.empty()) {
        traj_out.points.push_back(current);
    }

    return !traj_out.joint_names.empty() && !traj_out.points.empty();
}
}  // namespace

GraspManager::GraspManager(rclcpp::Node::SharedPtr node,
                           moveit::planning_interface::MoveGroupInterface &move_group,
                           moveit::planning_interface::MoveGroupInterface &gripper_group,
                           const moveit::core::JointModelGroup *joint_model_group,
                           bool enable_visualization)
    : node_(node),
      move_group_(move_group),
      gripper_group_(gripper_group),
      joint_model_group_(joint_model_group),
      enable_visualization_(enable_visualization),
      interpolation_segments_(10) {

    // 说明：初始化发布器与基本配置
    // 创建发布器
    dense_path_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_dense_path", 10);
    gripper_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "gripper_status", 10);

    RCLCPP_INFO(LOGGER, "抓取管理器初始化完成（可视化：%s）",
                enable_visualization_ ? "开启" : "关闭");
}

void GraspManager::setInterpolationSegments(int segments) {
    // 说明：修改插值密度（每段插值点数量）
    interpolation_segments_ = segments;
    RCLCPP_INFO(LOGGER, "插值段数设置为 %d", segments);
}

moveit::planning_interface::MoveGroupInterface& GraspManager::getMoveGroup() {
    // 说明：提供 MoveGroupInterface 的引用给外部使用
    return move_group_;
}

void GraspManager::publishDenseTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    
    // 说明：将插值后的轨迹通过 DisplayTrajectory 发布到 RViz
    if (!enable_visualization_ || !dense_path_pub_) return;
    
    try {
        moveit_msgs::msg::DisplayTrajectory disp_msg;
        disp_msg.trajectory_start = plan.start_state_;
        disp_msg.trajectory.clear();
        disp_msg.trajectory.push_back(plan.trajectory_);
        dense_path_pub_->publish(disp_msg);
        RCLCPP_INFO(LOGGER, "已发布稠密轨迹");
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "发布轨迹失败: %s", e.what());
    }
}

void GraspManager::densifyTrajectory(
    moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    // 说明：对规划轨迹进行插值稠密化并发布
    try {
        auto &traj = plan.trajectory_.joint_trajectory;
        if (traj.points.size() < 2) {
            return;
        }

        TrajectoryInterpolator interpolator(
            traj.joint_names.size(),
            TrajectoryInterpolator::Method::CUBIC,
            interpolation_segments_);
        interpolator.setInput(traj);
        auto dense_traj = interpolator.generate();

        size_t orig_size = traj.points.size();
        plan.trajectory_.joint_trajectory = dense_traj;

        RCLCPP_INFO(LOGGER, "轨迹已插值：%zu -> %zu 个点",
                   orig_size, dense_traj.points.size());

        publishDenseTrajectory(plan);
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "插值异常: %s", e.what());
    }
}

void GraspManager::publishGripperStatus(const std::string &status) {
    // 说明：发布夹爪状态字符串（例如 open/close）
    if (!gripper_status_pub_) return;
    
    std_msgs::msg::String msg;
    msg.data = status;
    gripper_status_pub_->publish(msg);
    const char* status_zh = status == "open" ? "打开" :
                            status == "close" ? "关闭" :
                            status.c_str();
    RCLCPP_INFO(LOGGER, "夹爪状态：%s", status_zh);
}


bool GraspManager::trajPlan(const std::vector<double> &joint_target,
                           moveit::planning_interface::MoveGroupInterface::Plan &plan,
                           int max_attempts) {
    // 说明：
    // 1) 多次尝试规划关节目标
    // 2) 成功后对轨迹做插值稠密化
    bool success = false;
    int attempt_count = 0;
    move_group_.setPlanningTime(15.0);
    move_group_.setJointValueTarget(joint_target);

    while (attempt_count < max_attempts && !success) {
        success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            attempt_count++;
            RCLCPP_WARN(LOGGER, "规划尝试 %d/%d 失败", attempt_count, max_attempts);
        }
    }

    if (!success) {
        RCLCPP_ERROR(LOGGER, "规划重试 %d 次后失败", max_attempts);
        return false;
    }

    // 使用TrajectoryInterpolator对轨迹进行插值
    densifyTrajectory(plan);
    return true;
}

bool GraspManager::planWithIKRetry(const geometry_msgs::msg::Pose &target_pose,
                                  moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                  int max_ik_attempts) {
    // 说明：
    // - 使用多种随机种子尝试 IK
    // - 每次 IK 成功后尝试规划
    RCLCPP_INFO(LOGGER, "开始逆解/规划重试");

    auto base_state = move_group_.getCurrentState();
    if (!base_state) {
        RCLCPP_ERROR(LOGGER, "获取当前状态失败");
        return false;
    }

    for (int i = 0; i < max_ik_attempts; ++i) {
        moveit::core::RobotState ik_seed_state(*base_state);
        
        if (i > 0) {
            ik_seed_state.setToRandomPositions(joint_model_group_);
        }

        bool found_ik = ik_seed_state.setFromIK(joint_model_group_, target_pose, 0.5);
        if (!found_ik) {
            RCLCPP_WARN(LOGGER, "逆解尝试 %d/%d：无解", i + 1, max_ik_attempts);
            continue;
        }

        std::vector<double> joint_target;
        ik_seed_state.copyJointGroupPositions(joint_model_group_, joint_target);

        if (trajPlan(joint_target, plan, 1)) {
            RCLCPP_INFO(LOGGER, "逆解尝试 %d/%d：成功", i + 1, max_ik_attempts);
            return true;
        } else {
            RCLCPP_WARN(LOGGER, "逆解尝试 %d/%d：规划失败", i + 1, max_ik_attempts);
        }
    }

    RCLCPP_ERROR(LOGGER, "逆解共尝试 %d 次均失败", max_ik_attempts);
    return false;
}

bool GraspManager::planWithFallback(const geometry_msgs::msg::Pose &target_pose,
                                    moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    // 说明：
    // - 先用 CHOMP 进行 IK/规划
    move_group_.setStartStateToCurrentState();
    move_group_.setPlanningPipelineId("ompl");
    //设置为RRTconnect
    move_group_.setPlannerId("RRTConnectkConfigDefault");
    move_group_.setPoseTarget(target_pose);
    move_group_.setPlanningTime(10.0);

    bool success = false;
    const int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts && !success; ++attempt) {
        success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(LOGGER, "OMPL 位姿规划尝试 %d/%d 失败", attempt, max_attempts);
        }
    }
    move_group_.clearPoseTargets();

    if (success) {
        densifyTrajectory(plan);
        RCLCPP_INFO(LOGGER, "备用规划：OMPL 成功");
        return true;
    }
    // - 失败后切换到 OMPL 再尝试
    // - 备用方案暂未实现，当前仅打印提示
    move_group_.setStartStateToCurrentState();
    move_group_.setPlanningPipelineId("chomp");
    if (planWithIKRetry(target_pose, plan)) {
        RCLCPP_INFO(LOGGER, "备用规划：CHOMP 成功");
        return true;
    }

    RCLCPP_WARN(LOGGER, "备用规划：CHOMP 失败，改用 OMPL");
    return false;
}

void GraspManager::openGripper() {
    // 说明：调用命名目标 "open" 打开夹爪
    // gripper_group_.setNamedTarget("open");
    // gripper_group_.move();
    if (!send_command_client_) {
        send_command_client_ =
            node_->create_client<robot_interfaces::srv::SendCommand>("send_command");
    }
    if (send_command_client_->wait_for_service(std::chrono::seconds(2))) {
        auto request = std::make_shared<robot_interfaces::srv::SendCommand::Request>();
        request->command_type = kCmdGripperOpen;
        request->enable = true;
        auto future = send_command_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_WARN(LOGGER, "发送夹爪打开命令超时");
        }
    } else {
        RCLCPP_WARN(LOGGER, "发送命令服务不可用");
    }
    publishGripperStatus("open");
}

void GraspManager::closeGripper() {
    // 说明：调用命名目标 "close" 关闭夹爪
    // gripper_group_.setNamedTarget("close");
    // gripper_group_.move();
    if (!send_command_client_) {
        send_command_client_ =
            node_->create_client<robot_interfaces::srv::SendCommand>("send_command");
    }
    if (send_command_client_->wait_for_service(std::chrono::seconds(2))) {
        auto request = std::make_shared<robot_interfaces::srv::SendCommand::Request>();
        request->command_type = kCmdGripperClose;
        request->enable = true;
        auto future = send_command_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_WARN(LOGGER, "发送夹爪关闭命令超时");
        }
    } else {
        RCLCPP_WARN(LOGGER, "发送命令服务不可用");
    }
    publishGripperStatus("close");
}

void GraspManager::attachObject(const std::string &object_name,
                                const std::vector<std::string> &touch_links) {
    // 说明：将物体附加到末端执行器上，避免规划时被当作碰撞
    gripper_group_.attachObject(object_name, "link_hand", touch_links);
    RCLCPP_INFO(LOGGER, "已附加物体: %s", object_name.c_str());
}

void GraspManager::detachObject(const std::string &object_name) {
    // 说明：从末端执行器移除附着物体
    gripper_group_.detachObject(object_name);
    RCLCPP_INFO(LOGGER, "已移除物体: %s", object_name.c_str());
}

bool GraspManager::goHome(const std::vector<double> &home_joint_values) {
    // 说明：规划并执行到 HOME 关节角度
    RCLCPP_INFO(LOGGER, "========== 返回回零位 ==========");

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;

    if (!trajPlan(home_joint_values, home_plan)) {
        RCLCPP_ERROR(LOGGER, "回零位规划失败");
        return false;
    }

    RCLCPP_INFO(LOGGER, "执行回零位运动");
    if (move_group_.execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "回零位执行失败");
        return false;
    }

    move_group_.setStartStateToCurrentState();
    RCLCPP_INFO(LOGGER, "========== 已到达回零位 ==========");
    return true;
}

bool GraspManager::executeTrajectoryFile(const std::filesystem::path &path,
                                         const std::string &label) {
    trajectory_msgs::msg::JointTrajectory traj;
    if (!loadTrajectory(path, traj)) {
        RCLCPP_ERROR(LOGGER, "加载轨迹失败: %s", path.c_str());
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_.joint_trajectory = traj;

    auto state = move_group_.getCurrentState();
    if (state) {
        state->update();
        moveit::core::robotStateToRobotStateMsg(*state, plan.start_state_);
    }

    RCLCPP_INFO(LOGGER, "执行预置轨迹: %s", label.c_str());
    publishDenseTrajectory(plan);
    const auto exec_code = move_group_.execute(plan);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "预置轨迹执行失败: %s", label.c_str());
        return false;
    }

    move_group_.setStartStateToCurrentState();
    return true;
}

bool GraspManager::home() {
    return executeTrajectoryFile(configDir() / "home.yaml", "回零");
}

bool GraspManager::target() {
    return executeTrajectoryFile(configDir() / "target.yaml", "目标位");
}
