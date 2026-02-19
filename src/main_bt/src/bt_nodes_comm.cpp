#include "main_bt/bt_nodes.hpp"

// 说明：
// 该文件集中实现“视觉/通信 + 转盘”相关节点，
// 未来若新增二维码识别节点，也放在这里。

#include "main_bt/communication.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <cctype>
#include <chrono>
#include <future>
#include <vector>

namespace main_bt
{
namespace
{
constexpr uint8_t kCmdMoveTarget = 1;
constexpr uint8_t kCmdDisplayTaskOrder = 2;
constexpr uint8_t kCmdTurntableRotate = 3;
}  // namespace

// ============================
// RequestCylindersWithRetry
// ============================

RequestCylindersWithRetry::RequestCylindersWithRetry(const std::string& name,
                                                     const BT::NodeConfig& config,
                                                     std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus RequestCylindersWithRetry::tick()
{
  if (!ctx_ || !ctx_->communication)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::vector<CylinderConfig> cylinders;
  if (!ctx_->communication->requestCylindersWithRetry(cylinders))
  {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("cylinders_out", cylinders);
  return BT::NodeStatus::SUCCESS;
}

// ============================
// RequestPlacePointsAllValid
// ============================

RequestPlacePointsAllValid::RequestPlacePointsAllValid(const std::string& name,
                                                       const BT::NodeConfig& config,
                                                       std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus RequestPlacePointsAllValid::tick()
{
  if (!ctx_ || !ctx_->communication)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::vector<CylinderConfig> points;
  if (!ctx_->communication->requestPlacePointsAllValid(points, 3))
  {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("points_out", points);
  return BT::NodeStatus::SUCCESS;
}

// ============================
// SendMoveCommandAction
// ============================

SendMoveCommandAction::SendMoveCommandAction(const std::string& name,
                                             const BT::NodeConfig& config,
                                             std::shared_ptr<BtContext> ctx,
                                             uint8_t target_id)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)), target_id_(target_id)
{}

BT::NodeStatus SendMoveCommandAction::tick()
{
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_)
  {
    client_ = ctx_->node->create_client<robot_interfaces::srv::SendCommand>("send_command");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "发送命令服务不可用");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<robot_interfaces::srv::SendCommand::Request>();
  request->command_type = kCmdMoveTarget;
  request->enable = true;
  request->data = {static_cast<float>(target_id_)};

  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "发送移动目标命令超时");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "移动目标失败: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

// ============================
// SendDisplayCommandAction
// ============================

SendDisplayCommandAction::SendDisplayCommandAction(const std::string& name,
                                                   const BT::NodeConfig& config,
                                                   std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus SendDisplayCommandAction::tick()
{
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto input = getInput<std::string>("task_order_in");
  if (!input)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::vector<float> order;
  const std::string& raw = input.value();
  std::string token;
  token.reserve(raw.size());
  for (char ch : raw)
  {
    if (std::isdigit(static_cast<unsigned char>(ch)))
    {
      token.push_back(ch);
      continue;
    }
    if (!token.empty())
    {
      order.push_back(static_cast<float>(std::stoi(token)));
      token.clear();
    }
  }
  if (!token.empty())
  {
    order.push_back(static_cast<float>(std::stoi(token)));
  }

  if (order.empty())
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "任务序列为空");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_)
  {
    client_ = ctx_->node->create_client<robot_interfaces::srv::SendCommand>("send_command");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "发送命令服务不可用");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<robot_interfaces::srv::SendCommand::Request>();
  request->command_type = kCmdDisplayTaskOrder;
  request->enable = true;
  request->data = order;

  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "发送任务序列显示命令超时");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "任务序列显示失败: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

// ============================
// RotateTurntable
// ============================

RotateTurntable::RotateTurntable(const std::string& name,
                                 const BT::NodeConfig& config,
                                 std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus RotateTurntable::tick()
{
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_)
  {
    client_ = ctx_->node->create_client<robot_interfaces::srv::SendCommand>("send_command");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "发送命令服务不可用");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<robot_interfaces::srv::SendCommand::Request>();
  request->command_type = kCmdTurntableRotate;
  request->enable = true;

  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "发送转盘旋转命令超时");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "转盘旋转失败: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace main_bt
