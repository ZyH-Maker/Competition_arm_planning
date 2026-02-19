#pragma once

// 说明：
// 该文件用于封装视觉/通信相关逻辑，避免 BT 节点重复写服务调用与解析。
// 当前实现 GetCircles 请求与解析，未来可扩展二维码、任务码等通信接口。

#include "main_bt/bt_nodes.hpp"

#include <robot_interfaces/srv/get_circles.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace main_bt
{

class Communication
{
public:
  // 构造：绑定 ROS2 节点
  explicit Communication(const rclcpp::Node::SharedPtr& node);

  // 请求视觉圆柱体列表（带重试）
  bool requestCylindersWithRetry(std::vector<CylinderConfig>& out_cylinders,
                                 int max_retries = 50,
                                 int timeout_sec = 3,
                                 int sleep_ms = 500);

  // 请求放置点（必须有 required_valid 个 exist==true）
  bool requestPlacePointsAllValid(std::vector<CylinderConfig>& out_points,
                                  size_t required_valid = 3,
                                  int max_retries = 50,
                                  int timeout_sec = 3,
                                  int sleep_ms = 500);

private:
  // 解析视觉服务返回的字符串，生成 CylinderConfig 列表
  std::vector<CylinderConfig> parseCylinderString(const std::string& data) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robot_interfaces::srv::GetCircles>::SharedPtr client_;
};

}  // namespace main_bt
