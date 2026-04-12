#include "main_bt/bt_nodes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <thread>

namespace main_bt::test
{
class AlwaysSuccessAction : public BT::SyncActionNode
{
public:
  AlwaysSuccessAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

class SendGripperCommandAction : public BT::SyncActionNode
{
public:
  SendGripperCommandAction(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx,
                           uint8_t command_type)
    : BT::SyncActionNode(name, config), ctx_(std::move(ctx)), command_type_(command_type)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
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
    request->command_type = command_type_;
    request->enable = true;

    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
      RCLCPP_WARN(ctx_->node->get_logger(), "夹爪命令发送超时");
      return BT::NodeStatus::FAILURE;
    }

    return future.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<BtContext> ctx_;
  uint8_t command_type_;
  rclcpp::Client<robot_interfaces::srv::SendCommand>::SharedPtr client_;
};
}  // namespace main_bt::test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("main_bt_test_runner", opts);

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

  const std::string tree_relpath =
    get_or_declare("tree_relpath", std::string("test/trees/main_test.xml"));
  auto ctx = std::make_shared<main_bt::BtContext>();
  ctx->node = node;
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<main_bt::test::AlwaysSuccessAction>("InitAllConfigs");

  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("StartRobot", ctx, "start");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoTurntableArea", ctx, 2);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveTurntableArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoPlaceAndPickArea", ctx, 3);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArrivePlaceAndPickArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoBufferArea", ctx, 4);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveBufferArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoStartArea", ctx, 5);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveStartArea", ctx, "success");
  factory.registerNodeType<main_bt::KillAllProcessesAction>("KillAllProcesses", ctx);

  factory.registerNodeType<main_bt::RotateTurntable>("RotateTurntable", ctx);
  factory.registerNodeType<main_bt::test::SendGripperCommandAction>("OpenGripper", ctx, 5);
  factory.registerNodeType<main_bt::test::SendGripperCommandAction>("CloseGripper", ctx, 4);

  const auto share_dir = ament_index_cpp::get_package_share_directory("main_bt");
  const auto xml_file =
    (std::filesystem::path(share_dir) / tree_relpath).string();

  auto tree = factory.createTreeFromFile(xml_file);
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "=== 通信测试行为树开始: %s ===", xml_file.c_str());
  tree.tickWhileRunning();
  RCLCPP_INFO(node->get_logger(), "=== 通信测试行为树结束 ===");

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
