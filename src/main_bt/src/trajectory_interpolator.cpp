#include "main_bt/trajectory_interpolator.hpp"
#include <algorithm>
#include <cmath>

/* -------------------------------------------------------------------------- */
/* 构造 / 输入                                                                 */
/* -------------------------------------------------------------------------- */

TrajectoryInterpolator::TrajectoryInterpolator(std::size_t dof,
                                               Method method,
                                               std::size_t num_per_segment)
    : dof_(dof), method_(method), num_per_segment_(num_per_segment) {}

void TrajectoryInterpolator::setInput(const trajectory_msgs::msg::JointTrajectory &input) {
  // 说明：保存原始轨迹点，供后续插值使用
  input_points_            = input.points;
  output_traj_.joint_names = input.joint_names;
}

/* -------------------------------------------------------------------------- */
/* 生成稠密轨迹                                                                */
/* -------------------------------------------------------------------------- */

trajectory_msgs::msg::JointTrajectory TrajectoryInterpolator::generate() {
  // 说明：根据插值方法对每一段轨迹进行加密
  output_traj_.points.clear();
  if (input_points_.size() < 2) {
    return output_traj_;
  }
  for (std::size_t seg_idx = 0; seg_idx + 1 < input_points_.size(); ++seg_idx) {
    const auto &p1 = input_points_[seg_idx];
    const auto &p2 = input_points_[seg_idx + 1];
    if (seg_idx == 0) {
      output_traj_.points.push_back(p1);
    }
    for (std::size_t j = 1; j < num_per_segment_ - 1; ++j) {
      double lambda = static_cast<double>(j) / (num_per_segment_ - 1);
      switch (method_) {
        case Method::LINEAR:
          output_traj_.points.push_back(lerp(p1, p2, lambda));
          break;
        case Method::CUBIC:
          output_traj_.points.push_back(cubicInterpolate(p1, p2, lambda));
          break;
      }
    }
    output_traj_.points.push_back(p2);
  }
  return output_traj_;
}

/* -------------------------------------------------------------------------- */
/* 线性插值实现                                                                */
/* -------------------------------------------------------------------------- */

trajectory_msgs::msg::JointTrajectoryPoint
TrajectoryInterpolator::lerp(const trajectory_msgs::msg::JointTrajectoryPoint &p1,
                             const trajectory_msgs::msg::JointTrajectoryPoint &p2,
                             double lambda) {
  // 说明：对两点位置/速度/时间线性插值
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.resize(dof_);
  p.velocities.resize(dof_);
  for (std::size_t k = 0; k < dof_; ++k) {
    p.positions[k] = p1.positions[k] + lambda * (p2.positions[k] - p1.positions[k]);
    double v1 = (k < p1.velocities.size()) ? p1.velocities[k] : 0.0;
    double v2 = (k < p2.velocities.size()) ? p2.velocities[k] : 0.0;
    p.velocities[k] = v1 + lambda * (v2 - v1);
  }
  double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
  double t2 = p2.time_from_start.sec + p2.time_from_start.nanosec * 1e-9;
  double t_interp = t1 + lambda * (t2 - t1);
  p.time_from_start.sec     = static_cast<int32_t>(std::floor(t_interp));
  p.time_from_start.nanosec = static_cast<uint32_t>(std::round((t_interp - p.time_from_start.sec) * 1e9));
  return p;
}

/* -------------------------------------------------------------------------- */
/* 三次样条插值实现                                                            */
/* -------------------------------------------------------------------------- */

trajectory_msgs::msg::JointTrajectoryPoint
TrajectoryInterpolator::cubicInterpolate(const trajectory_msgs::msg::JointTrajectoryPoint &p1,
                                         const trajectory_msgs::msg::JointTrajectoryPoint &p2,
                                         double lambda) {
  // 说明：对两点进行三次样条插值，输出位置/速度/加速度
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.resize(dof_);
  p.velocities.resize(dof_);
  p.accelerations.resize(dof_);
  double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
  double t2 = p2.time_from_start.sec + p2.time_from_start.nanosec * 1e-9;
  double T  = t2 - t1;
  if (T < 1e-9) T = 1e-9;
  for (std::size_t k = 0; k < dof_; ++k) {
    double q0 = p1.positions[k];
    double q1 = p2.positions[k];
    double v0 = (k < p1.velocities.size()) ? p1.velocities[k] : 0.0;
    double v1 = (k < p2.velocities.size()) ? p2.velocities[k] : 0.0;
    double h  = q1 - q0;
    double a0 = q0;
    double a1 = v0 * T;
    double a2 = 3*h - (2*v0 + v1)*T;
    double a3 = -2*h + (v0 + v1)*T;
    double lam  = lambda;
    double lam2 = lam * lam;
    double lam3 = lam2 * lam;
    double q   = a0 + a1*lam + a2*lam2 + a3*lam3;
    double dq  = (a1 + 2*a2*lam + 3*a3*lam2) / T;
    double ddq = (2*a2 + 6*a3*lam) / (T*T);
    p.positions[k]     = q;
    p.velocities[k]    = dq;
    p.accelerations[k] = ddq;
  }
  double t_interp = t1 + lambda * T;
  p.time_from_start.sec     = static_cast<int32_t>(std::floor(t_interp));
  p.time_from_start.nanosec = static_cast<uint32_t>(std::round((t_interp - p.time_from_start.sec)*1e9));
  return p;
}
