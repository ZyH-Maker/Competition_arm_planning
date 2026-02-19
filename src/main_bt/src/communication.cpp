#include "main_bt/communication.hpp"

// 说明：
// 该 cpp 文件实现 Communication 具体逻辑，包括：
// 1) 视觉服务请求与重试
// 2) circles_string 的解析

#include <chrono>
#include <future>
#include <sstream>
#include <string>
#include <thread>

namespace main_bt
{
namespace
{
const char* colorToChineseName(const std::array<float, 3>& color)
{
  if (color[0] == 1.f && color[1] == 0.f && color[2] == 0.f)
  {
    return "红色";
  }
  if (color[0] == 0.f && color[1] == 1.f && color[2] == 0.f)
  {
    return "绿色";
  }
  if (color[0] == 0.f && color[1] == 0.f && color[2] == 1.f)
  {
    return "蓝色";
  }
  if (color[0] == 0.5f && color[1] == 0.5f && color[2] == 0.5f)
  {
    return "灰色";
  }
  return "未知";
}
}  // namespace

Communication::Communication(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{}

bool Communication::requestCylindersWithRetry(std::vector<CylinderConfig>& out_cylinders,
                                              int max_retries,
                                              int timeout_sec,
                                              int sleep_ms)
{
  // 说明：
  // - 调用 GetCircles 服务获取视觉结果字符串
  // - 解析成 CylinderConfig 列表
  // - 失败则按次数重试
  if (!node_)
  {
    return false;
  }

  if (!client_)
  {
    client_ = node_->create_client<robot_interfaces::srv::GetCircles>("Circles");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "圆检测服务不可用");
    return false;
  }

  for (int attempt = 1; attempt <= max_retries; ++attempt)
  {
    auto request = std::make_shared<robot_interfaces::srv::GetCircles::Request>();
    request->trigger = true;

    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready)
    {
      RCLCPP_WARN(node_->get_logger(), "服务调用超时，重试 %d/%d",
                  attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    auto response = future.get();
    if (response->circles_string.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "圆检测字符串为空，重试 %d/%d",
                  attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    auto cylinders = parseCylinderString(response->circles_string);
    if (cylinders.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "解析到的圆柱为空，重试 %d/%d",
                  attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    out_cylinders = std::move(cylinders);
    RCLCPP_INFO(node_->get_logger(), "收到 %zu 个圆柱", out_cylinders.size());
    for (const auto& c : out_cylinders)
    {
      if (!c.exist)
      {
        continue;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "有效圆柱: id=%d, 颜色=%s, 坐标=(%.4f, %.4f, %.4f)",
                  c.id,
                  colorToChineseName(c.color),
                  c.pick_pose.position.x,
                  c.pick_pose.position.y,
                  c.pick_pose.position.z);
    }
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "重试 %d 次后仍失败", max_retries);
  return false;
}

bool Communication::requestPlacePointsAllValid(std::vector<CylinderConfig>& out_points,
                                               size_t required_valid,
                                               int max_retries,
                                               int timeout_sec,
                                               int sleep_ms)
{
  // 说明：
  // - 调用 GetCircles 服务获取视觉结果字符串
  // - 解析成 CylinderConfig 列表
  // - 必须有 required_valid 个 exist==true 的点，否则重试
  if (!node_)
  {
    return false;
  }

  if (!client_)
  {
    client_ = node_->create_client<robot_interfaces::srv::GetCircles>("Circles");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "圆检测服务不可用");
    return false;
  }

  for (int attempt = 1; attempt <= max_retries; ++attempt)
  {
    auto request = std::make_shared<robot_interfaces::srv::GetCircles::Request>();
    request->trigger = true;

    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready)
    {
      RCLCPP_WARN(node_->get_logger(), "放置点超时，重试 %d/%d",
                  attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    auto response = future.get();
    if (response->circles_string.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "放置点为空，重试 %d/%d",
                  attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    auto points = parseCylinderString(response->circles_string);
    std::vector<CylinderConfig> valid;
    for (const auto& p : points)
    {
      if (p.exist)
      {
        valid.push_back(p);
      }
    }

    if (valid.size() != required_valid)
    {
      RCLCPP_WARN(node_->get_logger(), "需要 %zu 个放置点，收到 %zu 个，重试 %d/%d",
                  required_valid, valid.size(), attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    out_points = std::move(valid);
    RCLCPP_INFO(node_->get_logger(), "收到 %zu 个放置点", out_points.size());
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "重试 %d 次后仍未获取到放置点",
               max_retries);
  return false;
}

std::vector<CylinderConfig> Communication::parseCylinderString(const std::string& data) const
{
  // 说明：
  // 格式示例： "1:red,true,0.1,0.2,0.3;2:green,false,..." 
  std::vector<CylinderConfig> cylinders;
  std::stringstream ss(data);
  std::string segment;

  while (std::getline(ss, segment, ';'))
  {
    const size_t colon = segment.find(':');
    if (colon == std::string::npos)
    {
      continue;
    }

    CylinderConfig cfg;
    try
    {
      cfg.id = std::stoi(segment.substr(0, colon));
    }
    catch (const std::exception&)
    {
      continue;
    }
    cfg.name = "cylinder_" + std::to_string(cfg.id);

    const std::string rest = segment.substr(colon + 1);
    std::stringstream ts(rest);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(ts, field, ','))
    {
      fields.push_back(field);
    }

    if (fields.size() != 5)
    {
      continue;
    }

    // 颜色解析
    if (fields[0] == "red")
    {
      cfg.color = {1.f, 0.f, 0.f};
    }
    else if (fields[0] == "green")
    {
      cfg.color = {0.f, 1.f, 0.f};
    }
    else if (fields[0] == "blue")
    {
      cfg.color = {0.f, 0.f, 1.f};
    }
    else
    {
      cfg.color = {0.5f, 0.5f, 0.5f};
    }

    // exist 字段解析
    cfg.exist = (fields[1] == "True");

    // 位姿解析
    try
    {
      cfg.pick_pose.position.x = std::stof(fields[2]);
      cfg.pick_pose.position.y = std::stof(fields[3]);
      cfg.pick_pose.position.z = std::stof(fields[4]);
      cfg.pick_pose.orientation.w = 1.0;
    }
    catch (const std::exception&)
    {
      continue;
    }

    cylinders.push_back(cfg);
  }

  return cylinders;
}

}  // namespace main_bt
