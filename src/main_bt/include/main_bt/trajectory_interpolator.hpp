#pragma once

// 说明：
// TrajectoryInterpolator 用于对 MoveIt 规划出的关节轨迹进行稠密化处理，
// 以提升轨迹的平滑度和执行效果。

#include <trajectory_msgs/msg/joint_trajectory.hpp>

class TrajectoryInterpolator {
public:
  enum class Method { LINEAR, CUBIC };
  TrajectoryInterpolator(std::size_t dof,
                         Method method,
                         std::size_t num_per_segment);
  // 设置输入轨迹：从外部传入原始轨迹点
  void setInput(const trajectory_msgs::msg::JointTrajectory &input);
  // 生成稠密轨迹：按设定插值方法生成输出轨迹
  trajectory_msgs::msg::JointTrajectory generate();

private:
  // 线性插值：对相邻两个轨迹点做线性插值
  trajectory_msgs::msg::JointTrajectoryPoint lerp(const trajectory_msgs::msg::JointTrajectoryPoint &p1,
                                                  const trajectory_msgs::msg::JointTrajectoryPoint &p2,
                                                  double lambda);
  // 三次样条插值：使用速度约束进行平滑插值
  trajectory_msgs::msg::JointTrajectoryPoint cubicInterpolate(const trajectory_msgs::msg::JointTrajectoryPoint &p1,
                                                              const trajectory_msgs::msg::JointTrajectoryPoint &p2,
                                                              double lambda);
  std::size_t dof_;
  Method method_;
  std::size_t num_per_segment_;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> input_points_;
  trajectory_msgs::msg::JointTrajectory output_traj_;
};
