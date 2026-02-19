#include "main_bt/bt_nodes.hpp"
#include "main_bt/communication.hpp"

// 说明：
// 这是 main_bt 的主执行程序（BT Runner），负责：
// 1) 初始化 ROS2 节点和 MoveIt 接口
// 2) 构建 BtContext，注入 MoveIt 与 GraspManager
// 3) 注册 BT 自定义节点
// 4) 加载主行为树 XML 并执行

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <thread>

int main(int argc, char** argv)
{
  // 说明：初始化 ROS2
  rclcpp::init(argc, argv);

  // 说明：创建 BT 主节点（自动声明参数，便于通过命令行传参）
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("main_bt_runner", opts);

  // 说明：启动后台执行器，确保 ROS2 的服务/话题可正常工作
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&](){ exec.spin(); });

  // 说明：从参数获取 MoveIt 组名
  const std::string arm_group = node->declare_parameter<std::string>("arm_group", "arm");
  const std::string gripper_group = node->declare_parameter<std::string>("gripper_group", "hand");

  // 说明：是否启用可视化与插值密度
  const bool enable_visualization = node->declare_parameter<bool>("enable_visualization", true);
  const int interpolation_segments =
    node->declare_parameter<int>("interpolation_segments", 10);

  // 说明：视觉进程启动命令（为空则对应 Start*Vision 失败）
  const std::string qrcode_vision_cmd =
    node->declare_parameter<std::string>("qrcode_vision_cmd", "/home/fins/Vision/build/Bin/Qr");
  const std::string turntable_vision_cmd =
    node->declare_parameter<std::string>("turntable_vision_cmd", "/home/fins/Vision/build/Bin/Detect_P");
  const std::string place_and_pick_vision_cmd =
    node->declare_parameter<std::string>("place_and_pick_vision_cmd", "/home/fins/Vision/build/Bin/Detect2");
  const std::string buffer_vision_cmd =
    node->declare_parameter<std::string>("buffer_vision_cmd", "/home/fins/Vision/build/Bin/Detect");

  // 说明：创建 MoveIt 控制接口
  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  moveit::planning_interface::MoveGroupInterface gripper(node, gripper_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  // 说明：设置运动速度缩放（1.0 为最大速度）
  arm.setMaxVelocityScalingFactor(1.0);
  // 说明：设置运动加速度缩放（1.0 为最大加速度）
  //arm.setMaxAccelerationScalingFactor(1.0);

  // 说明：获取关节模型组（用于 IK 规划）
  const moveit::core::JointModelGroup* joint_model_group =
    arm.getCurrentState()->getJointModelGroup(arm_group);

  // 说明：创建 GraspManager（用于轨迹插值、夹爪状态发布等）
  auto grasp_manager = std::make_shared<GraspManager>(
    node, arm, gripper, joint_model_group, enable_visualization);
  grasp_manager->setInterpolationSegments(interpolation_segments);

  // 说明：可视化工具（可选）
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
  if (enable_visualization)
  {
    visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      node, "base_link", "/rviz_visual_tools", arm.getRobotModel());
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    visual_tools->trigger();
  }

  // 说明：构建 BT 上下文（供节点共享资源）
  auto ctx = std::make_shared<main_bt::BtContext>();
  ctx->node = node;
  ctx->arm = &arm;
  ctx->gripper = &gripper;
  ctx->planning_scene = &planning_scene;
  ctx->visual_tools = visual_tools;
  ctx->grasp_manager = grasp_manager;
  ctx->communication = std::make_shared<main_bt::Communication>(node);
  ctx->joint_model_group = joint_model_group;
  ctx->turntable_pub = node->create_publisher<std_msgs::msg::String>("turntable_cmd", 10);

  // 说明：注册自定义 BT 节点
  BT::BehaviorTreeFactory factory;

  // 无额外构造参数的节点
  factory.registerNodeType<main_bt::PopNextTargetId>("PopNextTargetId");
  factory.registerNodeType<main_bt::PopAndSelectPlaceTarget>("PopAndSelectPlaceTarget");
  factory.registerNodeType<main_bt::SelectTargetCylinder>("SelectTargetCylinder");

  // main.xml 新增的流程节点
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("StartRobot", ctx, "start");
  factory.registerNodeType<main_bt::InitAllConfigs>("InitAllConfigs", ctx);
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoQRCodeArea", ctx, 1);
  factory.registerNodeType<main_bt::StartProcessAction>("StartQRCodeVision", ctx,
                                                        "qrcode", qrcode_vision_cmd);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveQRCodeArea", ctx, "success");
  factory.registerNodeType<main_bt::ReadQRCodeFileAction>("ScanQRCode", ctx);
  factory.registerNodeType<main_bt::SendDisplayCommandAction>("DisplayQRCode", ctx);
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoTurntableArea", ctx, 2);
  factory.registerNodeType<main_bt::StartProcessAction>("StartTurntableVision", ctx,
                                                        "turntable", turntable_vision_cmd);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveTurntableArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoPlaceAndPickArea", ctx, 3);
  factory.registerNodeType<main_bt::StopProcessAction>("StopTurntableVision", ctx, "turntable");
  factory.registerNodeType<main_bt::StartProcessAction>("StartPlaceAndPickVision", ctx,
                                                        "place_and_pick", place_and_pick_vision_cmd);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArrivePlaceAndPickArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoBufferArea", ctx, 4);
  factory.registerNodeType<main_bt::StopProcessAction>("StopPlaceAndPickVision", ctx, "place_and_pick");
  factory.registerNodeType<main_bt::StartProcessAction>("StartBufferVision", ctx,
                                                        "buffer", buffer_vision_cmd);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveBufferArea", ctx, "success");
  factory.registerNodeType<main_bt::SendMoveCommandAction>("GoStartArea", ctx, 5);
  factory.registerNodeType<main_bt::WaitForBoolTopicAction>("WaitArriveStartArea", ctx, "success");
  factory.registerNodeType<main_bt::StopProcessAction>("StopBufferVision", ctx, "buffer");
  factory.registerNodeType<main_bt::KillAllProcessesAction>("KillAllProcesses", ctx);

  // 需要上下文的节点
  factory.registerNodeType<main_bt::RequestCylindersWithRetry>("RequestCylindersWithRetry", ctx);
  factory.registerNodeType<main_bt::RequestPlacePointsAllValid>("RequestPlacePointsAllValid", ctx);
  factory.registerNodeType<main_bt::AddCollisionCylinders>("AddCollisionCylinders", ctx);
  factory.registerNodeType<main_bt::PlanWithFallback>("PlanWithFallback", ctx);
  factory.registerNodeType<main_bt::CloseGripper>("CloseGripper", ctx);
  factory.registerNodeType<main_bt::OpenGripper>("OpenGripper", ctx);
  factory.registerNodeType<main_bt::AttachObject>("AttachObject", ctx);
  factory.registerNodeType<main_bt::DetachObject>("DetachObject", ctx);
  factory.registerNodeType<main_bt::GoHome>("GoHome", ctx);
  factory.registerNodeType<main_bt::ExecPresetTarget>("ExecPresetTarget", ctx);
  factory.registerNodeType<main_bt::ExecPresetHome>("ExecPresetHome", ctx);
  factory.registerNodeType<main_bt::RotateTurntable>("RotateTurntable", ctx);
  factory.registerNodeType<main_bt::RemoveAllCollisionObjects>("RemoveAllCollisionObjects", ctx);
  factory.registerNodeType<main_bt::RemoveCollisionObject>("RemoveCollisionObject", ctx);

  // 说明：加载 XML 主行为树文件
  const auto share_dir = ament_index_cpp::get_package_share_directory("main_bt");
  const auto xml_file =
    (std::filesystem::path(share_dir) / "trees" / "main.xml").string();

  auto tree = factory.createTreeFromFile(xml_file);

  auto bb = tree.rootBlackboard();
  // 说明：预置车上固定存放位坐标
  geometry_msgs::msg::Pose fix_pose;
  fix_pose.orientation.w = 1.0;
  fix_pose.position.x = -0.17;
  fix_pose.position.y = -0.114;
  fix_pose.position.z = 0.1 + 0.075 / 2.0;
  bb->set("fix_pose", fix_pose);

  // 说明：用于放回车上时的目标（CylinderConfig）
  main_bt::CylinderConfig onboard_target;
  onboard_target.id = 0;
  onboard_target.name = "onboard_target";
  onboard_target.exist = true;
  onboard_target.pick_pose = fix_pose;
  bb->set("onboard_target", onboard_target);

  // 说明：使用标准控制台日志器
  BT::StdCoutLogger logger(tree);

  // 说明：执行行为树（统计 bt_runner 总耗时）
  RCLCPP_INFO(node->get_logger(), "=== 行为树开始: %s ===", xml_file.c_str());
  const auto bt_start_time = std::chrono::steady_clock::now();
  tree.tickWhileRunning();
  const auto bt_end_time = std::chrono::steady_clock::now();
  const auto bt_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(bt_end_time - bt_start_time).count();
  RCLCPP_INFO(node->get_logger(), "=== 行为树结束 ===");
  RCLCPP_INFO(node->get_logger(), "=== 行为树总耗时: %ld 毫秒 ===", bt_ms);

  // 说明：清理资源
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
