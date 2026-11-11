// Owner: Member 4. Foot placement.
#include "go1_ctrl/foot_planner.h"

#include <algorithm>

namespace go1 {

FootPlanner::FootPlanner(const FootPlannerParams& params) : params_(params) { reset(); }

void FootPlanner::reset() {
  held_.fill(Vec3::Zero());
  last_progress_.fill(1.0);
  v_filtered_.setZero();
  has_v_filtered_ = false;
}

const Vec3& FootPlanner::filterVelocity(const Vec3& v, double dt) {
  if (!has_v_filtered_) {
    v_filtered_ = v;
    has_v_filtered_ = true;
  } else if (dt > 0.0) {
    const double alpha = dt / (params_.velocity_filter_tau + dt);
    v_filtered_ += alpha * (v - v_filtered_);
  }
  return v_filtered_;
}

Vec3 FootPlanner::foothold(const Vec3& p_nominal, const Vec3& v, const Vec3& v_cmd,
                           double t_stance, double foot_z, double yaw_rate,
                           double yaw_rate_cmd) const {
  // Velocity of the hip point, including the contribution of yaw rate (w x r).
  const Eigen::Vector2d r = p_nominal.head<2>();
  const Eigen::Vector2d v_hip(v.x() - yaw_rate * r.y(), v.y() + yaw_rate * r.x());
  const Eigen::Vector2d v_hip_cmd(v_cmd.x() - yaw_rate_cmd * r.y(),
                                  v_cmd.y() + yaw_rate_cmd * r.x());

  Eigen::Vector2d offset = v_hip * t_stance / 2.0 + params_.k_raibert * (v_hip - v_hip_cmd);
  offset.x() = std::max(-params_.max_offset_x, std::min(params_.max_offset_x, offset.x()));
  offset.y() = std::max(-params_.max_offset_y, std::min(params_.max_offset_y, offset.y()));

  return Vec3(r.x() + offset.x(), r.y() + offset.y(), foot_z);
}

Vec3 FootPlanner::update(int leg, double swing_progress, const Vec3& p_nominal, const Vec3& v,
                         const Vec3& v_cmd, double t_stance, double foot_z, double yaw_rate,
                         double yaw_rate_cmd) {
  // A drop in progress means a new swing has started.
  const bool new_swing = swing_progress < last_progress_[leg];
  last_progress_[leg] = swing_progress;

  if (new_swing || swing_progress < params_.freeze_progress) {
    held_[leg] = foothold(p_nominal, v, v_cmd, t_stance, foot_z, yaw_rate, yaw_rate_cmd);
  }
  return held_[leg];
}

}  // namespace go1
