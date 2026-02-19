#pragma once

// 说明：
// 该头文件声明 main_bt 行为树(BT)节点所需的所有数据结构与节点类。
// 每个节点类都对应 XML 里的一个 Action ID，用于主树与子树共用。

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "main_bt/grasp.h"

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/send_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <atomic>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

namespace main_bt
{

class Communication;

// ============================
// 1) Blackboard 数据结构
// ============================

// 圆柱体配置：与视觉服务返回的数据结构相匹配。
// 用于描述一个目标物体的抓取位姿、颜色、是否存在等信息。
struct CylinderConfig
{
  int id = -1;                              // 目标ID（来自视觉或二维码的编号）
  geometry_msgs::msg::Pose pick_pose;       // 抓取位姿（通常在 base_link 坐标系）
  bool exist = false;                       // 目标是否有效/存在
  std::array<float, 3> color{0.f, 0.f, 0.f}; // 颜色 RGB
  std::string name;                         // 目标名称，如 "cylinder_1"
};

// BT 上下文：集中保存节点间共享资源（ROS 节点、MoveIt 句柄等）
// 这样每个 BT 节点只拿到 context 指针，不需要重复初始化资源。
struct BtContext
{
  rclcpp::Node::SharedPtr node;  // ROS 节点

  // MoveIt 执行接口
  moveit::planning_interface::MoveGroupInterface* arm = nullptr;
  moveit::planning_interface::MoveGroupInterface* gripper = nullptr;
  moveit::planning_interface::PlanningSceneInterface* planning_scene = nullptr;

  // 可视化工具（可选）
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;

  // Grasp 管理器：封装 IK/规划/夹爪控制/轨迹插值
  std::shared_ptr<GraspManager> grasp_manager;

  // 视觉/通信模块：统一封装服务调用与解析逻辑
  std::shared_ptr<Communication> communication;

  // 关节模型组指针：用于可视化轨迹线
  const moveit::core::JointModelGroup* joint_model_group = nullptr;

  // 转盘控制发布器：机械臂回 HOME 后发送转盘旋转信号
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr turntable_pub;
};

// ============================
// 2) BT 节点声明
// ============================

// 记录初始化时间（占位实现）。
class InitAllConfigs : public BT::SyncActionNode
{
public:
  InitAllConfigs(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  rclcpp::Time start_time_;
};

// 启动外部进程（视觉进程等），按 key 管理。
class StartProcessAction : public BT::SyncActionNode
{
public:
  StartProcessAction(const std::string& name, const BT::NodeConfig& config,
                     std::shared_ptr<BtContext> ctx,
                     const std::string& process_key,
                     const std::string& command);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  std::string process_key_;
  std::string command_;
};

// 关闭外部进程（优先 SIGTERM，失败则 SIGKILL）。
class StopProcessAction : public BT::SyncActionNode
{
public:
  StopProcessAction(const std::string& name, const BT::NodeConfig& config,
                    std::shared_ptr<BtContext> ctx,
                    const std::string& process_key);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  std::string process_key_;
};

// 杀死所有已启动的外部进程（按进程组 SIGKILL）。
class KillAllProcessesAction : public BT::SyncActionNode
{
public:
  KillAllProcessesAction(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 发布固定字符串到指定话题。
class PublishConstStringAction : public BT::SyncActionNode
{
public:
  PublishConstStringAction(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx,
                           const std::string& topic,
                           const std::string& message);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  std::string topic_;
  std::string message_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// 从端口读取字符串并发布到指定话题。
class PublishFromPortStringAction : public BT::SyncActionNode
{
public:
  PublishFromPortStringAction(const std::string& name, const BT::NodeConfig& config,
                              std::shared_ptr<BtContext> ctx,
                              const std::string& topic);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("task_order_in") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  std::string topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// 发送移动目标命令（替代 move 话题）。
class SendMoveCommandAction : public BT::SyncActionNode
{
public:
  SendMoveCommandAction(const std::string& name, const BT::NodeConfig& config,
                        std::shared_ptr<BtContext> ctx,
                        uint8_t target_id);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  uint8_t target_id_;
  rclcpp::Client<robot_interfaces::srv::SendCommand>::SharedPtr client_;
};

// 发送任务序列命令（替代 display 话题）。
class SendDisplayCommandAction : public BT::SyncActionNode
{
public:
  SendDisplayCommandAction(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("task_order_in") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  rclcpp::Client<robot_interfaces::srv::SendCommand>::SharedPtr client_;
};

// 等待 Bool 话题为 true。
class WaitForBoolTopicAction : public BT::StatefulActionNode
{
public:
  WaitForBoolTopicAction(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<BtContext> ctx,
                         const std::string& topic);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtContext> ctx_;
  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> received_;
};

// 读取 Config/QR.yaml 的任务码并输出到端口（读到即成功）。
class ReadQRCodeFileAction : public BT::StatefulActionNode
{
public:
  ReadQRCodeFileAction(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("task_order_out"),
      BT::OutputPort<std::string>("batch1_order_out"),
      BT::OutputPort<std::string>("batch2_order_out")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 从 order_in(字符串) 中弹出下一个目标ID，并回写剩余字符串。
class PopNextTargetId : public BT::SyncActionNode
{
public:
  PopNextTargetId(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("order_in"),
      BT::OutputPort<std::string>("order_out"),
      BT::OutputPort<int>("target_id")
    };
  }

  BT::NodeStatus tick() override;
};

// 请求视觉结果（带重试），输出圆柱体列表。
class RequestCylindersWithRetry : public BT::SyncActionNode
{
public:
  RequestCylindersWithRetry(const std::string& name, const BT::NodeConfig& config,
                            std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::vector<CylinderConfig>>("cylinders_out") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 请求视觉放置点：必须返回3个有效点，否则失败。
class RequestPlacePointsAllValid : public BT::SyncActionNode
{
public:
  RequestPlacePointsAllValid(const std::string& name, const BT::NodeConfig& config,
                             std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::vector<CylinderConfig>>("points_out") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 从顺序字符串弹出一个目标ID，并在放置点中选出对应目标。
class PopAndSelectPlaceTarget : public BT::SyncActionNode
{
public:
  PopAndSelectPlaceTarget(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("order_in"),
      BT::OutputPort<std::string>("order_out"),
      BT::InputPort<std::vector<CylinderConfig>>("points_in"),
      BT::OutputPort<CylinderConfig>("target_out"),
      BT::OutputPort<int>("target_id"),
      BT::OutputPort<std::string>("object_id")
    };
  }

  BT::NodeStatus tick() override;
};

// 根据 target_id 从 cylinders_in 中筛选目标，并输出 target + 有效列表。
class SelectTargetCylinder : public BT::SyncActionNode
{
public:
  SelectTargetCylinder(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("target_id"),
      BT::InputPort<std::vector<CylinderConfig>>("cylinders_in"),
      BT::OutputPort<CylinderConfig>("target_out"),
      BT::OutputPort<std::string>("object_id"),
      BT::OutputPort<std::vector<CylinderConfig>>("all_valid_out")
    };
  }

  BT::NodeStatus tick() override;
};

// 将圆柱体添加到规划场景（支持单个或列表，或按 target_id+fix_pose 生成）。
class AddCollisionCylinders : public BT::SyncActionNode
{
public:
  AddCollisionCylinders(const std::string& name, const BT::NodeConfig& config,
                        std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<CylinderConfig>>("cylinders_in"),
      BT::InputPort<CylinderConfig>("cylinder_in"),
      BT::InputPort<int>("target_id"),
      BT::InputPort<geometry_msgs::msg::Pose>("fix_pose"),
      BT::OutputPort<CylinderConfig>("cylinder_out"),
      BT::OutputPort<std::string>("object_id_out")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 统一规划节点：内部按 CHOMP -> OMPL -> 备用 顺序尝试规划与执行
class PlanWithFallback : public BT::SyncActionNode
{
public:
  PlanWithFallback(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<CylinderConfig>("target_in"),
      BT::InputPort<geometry_msgs::msg::Pose>("pose_in")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 关闭夹爪。
class CloseGripper : public BT::SyncActionNode
{
public:
  CloseGripper(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 打开夹爪。
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 将目标附加到夹爪。
class AttachObject : public BT::SyncActionNode
{
public:
  AttachObject(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_id") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 将目标从夹爪分离。
class DetachObject : public BT::SyncActionNode
{
public:
  DetachObject(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_id") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 机械臂回 HOME 位姿。
class GoHome : public BT::SyncActionNode
{
public:
  GoHome(const std::string& name, const BT::NodeConfig& config,
         std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 读取 Config/target.yaml 并执行预设轨迹。
class ExecPresetTarget : public BT::SyncActionNode
{
public:
  ExecPresetTarget(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 读取 Config/home.yaml 并执行预设轨迹。
class ExecPresetHome : public BT::SyncActionNode
{
public:
  ExecPresetHome(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 发送转盘旋转信号（机械臂回 HOME 后触发）。
class RotateTurntable : public BT::SyncActionNode
{
public:
  RotateTurntable(const std::string& name, const BT::NodeConfig& config,
                  std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
  rclcpp::Client<robot_interfaces::srv::SendCommand>::SharedPtr client_;
};

// 从场景中移除所有圆柱体碰撞体。
class RemoveAllCollisionObjects : public BT::SyncActionNode
{
public:
  RemoveAllCollisionObjects(const std::string& name, const BT::NodeConfig& config,
                            std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<CylinderConfig>>("cylinders_in") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

// 从场景中移除单个圆柱体碰撞体。
class RemoveCollisionObject : public BT::SyncActionNode
{
public:
  RemoveCollisionObject(const std::string& name, const BT::NodeConfig& config,
                        std::shared_ptr<BtContext> ctx);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<CylinderConfig>("cylinder_in") };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtContext> ctx_;
};

}  // namespace main_bt

// ============================
// 3) BT 类型转换（用于日志）
// ============================
namespace BT
{
// 让 BT 能够在日志里打印 CylinderConfig，方便调试。
template <>
inline std::string toStr<main_bt::CylinderConfig>(const main_bt::CylinderConfig& item)
{
  return "CylinderConfig(" + item.name + ")";
}
}  // namespace BT
