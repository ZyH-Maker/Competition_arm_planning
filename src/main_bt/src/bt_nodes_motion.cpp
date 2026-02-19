#include "main_bt/bt_nodes.hpp"

// 说明：
// 该文件集中实现“场景 + 运动 + 夹爪”相关节点：
// - 场景：AddCollisionCylinders / RemoveAllCollisionObjects / RemoveCollisionObject
// - 运动：PlanWithFallback / GoHome
// - 夹爪：Open/Close/Attach/Detach
// - 控制：PopNextTargetId / PopAndSelectPlaceTarget / SelectTargetCylinder

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// 圆柱体的几何参数（与实际物料尺寸保持一致）
constexpr double kCylinderHeight = 0.075;
constexpr double kCylinderRadius = 0.025;

// 抓取/放置通用的 Z 偏移（与原始逻辑一致）
constexpr double kPoseZOffset = 0.22;

// 解析 "3,2,1" 这种字符串为整数列表（忽略空字段与非法字段）
std::vector<int> parseOrderString(const std::string& order_str)
{
  std::vector<int> result;
  std::stringstream ss(order_str);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    token.erase(0, token.find_first_not_of(" \t\r\n"));
    token.erase(token.find_last_not_of(" \t\r\n") + 1);
    if (token.empty())
    {
      continue;
    }
    try
    {
      result.push_back(std::stoi(token));
    }
    catch (const std::exception&)
    {
      continue;
    }
  }
  return result;
}

// 将整数列表拼回 "3,2,1" 字符串
std::string joinOrderString(const std::vector<int>& order)
{
  std::ostringstream oss;
  for (size_t i = 0; i < order.size(); ++i)
  {
    if (i > 0)
    {
      oss << ",";
    }
    oss << order[i];
  }
  return oss.str();
}

// 根据 CylinderConfig 生成 MoveIt 碰撞体
moveit_msgs::msg::CollisionObject makeCylinderCollisionObject(const main_bt::CylinderConfig& cfg)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "base_link";
  obj.id = cfg.name;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = kCylinderHeight;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = kCylinderRadius;

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position = cfg.pick_pose.position;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  return obj;
}
}  // namespace

namespace main_bt
{

// ============================
// ============================
// PopNextTargetId
// ============================

PopNextTargetId::PopNextTargetId(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus PopNextTargetId::tick()
{
  const auto order_res = getInput<std::string>("order_in");
  if (!order_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto order_vec = parseOrderString(order_res.value());
  if (order_vec.empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const int target_id = order_vec.front();
  std::vector<int> remaining(order_vec.begin() + 1, order_vec.end());

  setOutput("target_id", target_id);
  setOutput("order_out", joinOrderString(remaining));
  return BT::NodeStatus::SUCCESS;
}

// ============================
// PopAndSelectPlaceTarget
// ============================

PopAndSelectPlaceTarget::PopAndSelectPlaceTarget(const std::string& name,
                                                 const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus PopAndSelectPlaceTarget::tick()
{
  const auto order_res = getInput<std::string>("order_in");
  const auto points_res = getInput<std::vector<CylinderConfig>>("points_in");
  if (!order_res || !points_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto order_vec = parseOrderString(order_res.value());
  if (order_vec.empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const int target_id = order_vec.front();
  std::vector<int> remaining(order_vec.begin() + 1, order_vec.end());
  setOutput("order_out", joinOrderString(remaining));
  setOutput("target_id", target_id);

  const auto& points = points_res.value();
  for (const auto& p : points)
  {
    if (p.id == target_id && p.exist)
    {
      setOutput("target_out", p);
      setOutput("object_id", p.name);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

// ============================
// SelectTargetCylinder
// ============================

SelectTargetCylinder::SelectTargetCylinder(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus SelectTargetCylinder::tick()
{
  const auto target_id_res = getInput<int>("target_id");
  const auto cylinders_res = getInput<std::vector<CylinderConfig>>("cylinders_in");

  if (!target_id_res || !cylinders_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const int target_id = target_id_res.value();
  const auto& cylinders = cylinders_res.value();

  std::vector<CylinderConfig> valid;
  valid.reserve(cylinders.size());

  const CylinderConfig* target_ptr = nullptr;
  for (const auto& c : cylinders)
  {
    if (c.exist)
    {
      valid.push_back(c);
      if (c.id == target_id)
      {
        target_ptr = &c;
      }
    }
  }

  if (!target_ptr)
  {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("target_out", *target_ptr);
  setOutput("object_id", target_ptr->name);
  setOutput("all_valid_out", valid);
  return BT::NodeStatus::SUCCESS;
}

// ============================
// AddCollisionCylinders
// ============================

AddCollisionCylinders::AddCollisionCylinders(const std::string& name,
                                             const BT::NodeConfig& config,
                                             std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus AddCollisionCylinders::tick()
{
  if (!ctx_ || !ctx_->planning_scene)
  {
    return BT::NodeStatus::FAILURE;
  }

  std::vector<CylinderConfig> cylinders;
  const auto single_res = getInput<CylinderConfig>("cylinder_in");
  const auto list_res = getInput<std::vector<CylinderConfig>>("cylinders_in");
  const auto target_id_res = getInput<int>("target_id");
  const auto fix_pose_res = getInput<geometry_msgs::msg::Pose>("fix_pose");

  if (single_res)
  {
    cylinders.push_back(single_res.value());
  }
  else if (list_res)
  {
    cylinders = list_res.value();
  }
  else if (target_id_res && fix_pose_res)
  {
    CylinderConfig cfg;
    cfg.id = target_id_res.value();
    cfg.name = "cylinder_" + std::to_string(cfg.id);
    cfg.exist = true;
    cfg.pick_pose = fix_pose_res.value();
    if (cfg.pick_pose.orientation.w == 0.0)
    {
      cfg.pick_pose.orientation.w = 1.0;
    }

    if (cfg.id == 1)
      cfg.color = {1.f, 0.f, 0.f};
    else if (cfg.id == 2)
      cfg.color = {0.f, 1.f, 0.f};
    else if (cfg.id == 3)
      cfg.color = {0.f, 0.f, 1.f};
    else
      cfg.color = {0.5f, 0.5f, 0.5f};

    cylinders.push_back(cfg);
    setOutput("cylinder_out", cfg);
    setOutput("object_id_out", cfg.name);
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  if (cylinders.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::vector<moveit_msgs::msg::ObjectColor> colors;

  objects.reserve(cylinders.size());
  colors.reserve(cylinders.size());

  for (const auto& c : cylinders)
  {
    objects.push_back(makeCylinderCollisionObject(c));

    moveit_msgs::msg::ObjectColor color;
    color.id = c.name;
    color.color.r = c.color[0];
    color.color.g = c.color[1];
    color.color.b = c.color[2];
    color.color.a = 1.0;
    colors.push_back(color);
  }

  ctx_->planning_scene->addCollisionObjects(objects, colors);
  // rclcpp::sleep_for(std::chrono::milliseconds(300));
  return BT::NodeStatus::SUCCESS;
}

// ============================
// PlanWithFallback
// ============================

PlanWithFallback::PlanWithFallback(const std::string& name,
                                   const BT::NodeConfig& config,
                                   std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus PlanWithFallback::tick()
{
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose pose;
  bool has_pose = false;

  if (auto pose_res = getInput<geometry_msgs::msg::Pose>("pose_in"))
  {
    pose = pose_res.value();
    has_pose = true;
  }

  if (!has_pose)
  {
    const auto target_res = getInput<CylinderConfig>("target_in");
    if (!target_res)
    {
      return BT::NodeStatus::FAILURE;
    }

    const auto& target = target_res.value();
    pose = target.pick_pose;
  }

  Eigen::Vector3d axis_vec(0.0, 1.0, 0.0);
  Eigen::Quaterniond quat(Eigen::AngleAxisd(0, axis_vec));
  pose.position.z += kPoseZOffset;
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!ctx_->grasp_manager->planWithFallback(pose, plan))
  {
    return BT::NodeStatus::FAILURE;
  }

  if (ctx_->visual_tools && ctx_->joint_model_group)
  {
    ctx_->visual_tools->publishAxisLabeled(pose, "target");
    ctx_->visual_tools->publishTrajectoryLine(plan.trajectory_, ctx_->joint_model_group);
    ctx_->visual_tools->trigger();
  }

  auto& move_group = ctx_->grasp_manager->getMoveGroup();
  const auto exec_code = move_group.execute(plan);
  return exec_code == moveit::core::MoveItErrorCode::SUCCESS
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

// ============================
// CloseGripper
// ============================

CloseGripper::CloseGripper(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus CloseGripper::tick()
{
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  ctx_->grasp_manager->closeGripper();
  return BT::NodeStatus::SUCCESS;
}

// ============================
// OpenGripper
// ============================

OpenGripper::OpenGripper(const std::string& name,
                         const BT::NodeConfig& config,
                         std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus OpenGripper::tick()
{
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  ctx_->grasp_manager->openGripper();
  return BT::NodeStatus::SUCCESS;
}

// ============================
// AttachObject
// ============================

AttachObject::AttachObject(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus AttachObject::tick()
{
  const auto obj_res = getInput<std::string>("object_id");
  if (!obj_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const std::vector<std::string> touch_links = {"link_right", "link_left"};
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  ctx_->grasp_manager->attachObject(obj_res.value(), touch_links);
  return BT::NodeStatus::SUCCESS;
}

// ============================
// DetachObject
// ============================

DetachObject::DetachObject(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus DetachObject::tick()
{
  const auto obj_res = getInput<std::string>("object_id");
  if (!obj_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  ctx_->grasp_manager->detachObject(obj_res.value());
  return BT::NodeStatus::SUCCESS;
}

// ============================
// GoHome
// ============================

GoHome::GoHome(const std::string& name,
               const BT::NodeConfig& config,
               std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus GoHome::tick()
{
  if (!ctx_)
  {
    return BT::NodeStatus::FAILURE;
  }

  const std::vector<double> home_joint_values = {0, 0, 0, 0, 0, 0};

  if (ctx_->grasp_manager)
  {
    return ctx_->grasp_manager->goHome(home_joint_values)
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
  }

  if (!ctx_->arm)
  {
    return BT::NodeStatus::FAILURE;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ctx_->arm->setPlanningTime(10.0);
  ctx_->arm->setJointValueTarget(home_joint_values);

  const auto code = ctx_->arm->plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto exec_code = ctx_->arm->execute(plan);
  return (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

// ============================
// ExecPresetTarget
// ============================

ExecPresetTarget::ExecPresetTarget(const std::string& name,
                                   const BT::NodeConfig& config,
                                   std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus ExecPresetTarget::tick()
{
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  return ctx_->grasp_manager->target()
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

// ============================
// ExecPresetHome
// ============================

ExecPresetHome::ExecPresetHome(const std::string& name,
                               const BT::NodeConfig& config,
                               std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus ExecPresetHome::tick()
{
  if (!ctx_ || !ctx_->grasp_manager)
  {
    return BT::NodeStatus::FAILURE;
  }

  return ctx_->grasp_manager->home()
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

// ============================
// RemoveAllCollisionObjects
// ============================

RemoveAllCollisionObjects::RemoveAllCollisionObjects(const std::string& name,
                                                     const BT::NodeConfig& config,
                                                     std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus RemoveAllCollisionObjects::tick()
{
  if (!ctx_ || !ctx_->planning_scene)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto cylinders_res = getInput<std::vector<CylinderConfig>>("cylinders_in");
  if (!cylinders_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto& cylinders = cylinders_res.value();
  if (cylinders.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<std::string> ids;
  ids.reserve(cylinders.size());
  for (const auto& c : cylinders)
  {
    ids.push_back(c.name);
  }

  ctx_->planning_scene->removeCollisionObjects(ids);
  // rclcpp::sleep_for(std::chrono::milliseconds(200));
  return BT::NodeStatus::SUCCESS;
}

// ============================
// RemoveCollisionObject
// ============================

RemoveCollisionObject::RemoveCollisionObject(const std::string& name,
                                             const BT::NodeConfig& config,
                                             std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus RemoveCollisionObject::tick()
{
  if (!ctx_ || !ctx_->planning_scene)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto cyl_res = getInput<CylinderConfig>("cylinder_in");
  if (!cyl_res)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto& cyl = cyl_res.value();
  const std::string id = !cyl.name.empty()
                         ? cyl.name
                         : ("cylinder_" + std::to_string(cyl.id));

  ctx_->planning_scene->removeCollisionObjects({id});
  // rclcpp::sleep_for(std::chrono::milliseconds(200));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace main_bt
