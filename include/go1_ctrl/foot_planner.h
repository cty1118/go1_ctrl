#pragma once
// Owner: Member 4. Foot placement (Raibert).
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {

struct FootPlannerParams {
  double k_raibert = 0.1;        // velocity feedback gain k [s], roughly sqrt(body_height / g)
  double max_offset_x = 0.15;    // limit on |foothold - nominal| in x [m]
  double max_offset_y = 0.08;    // limit on |foothold - nominal| in y [m]
  double freeze_progress = 0.8;  // stop moving the foothold after this swing progress
  double velocity_filter_tau = 0.1;  // low-pass time constant on the body velocity [s]
};

// Raibert foot placement:
//   p_land = p_nominal + v_hip * T_stance / 2 + k * (v_hip - v_hip_cmd)
//
// All vectors are in the body frame (x forward, y left, z up). p_nominal is the
// neutral foot position under the hip (hip joint plus the lateral thigh offset);
// only its x, y are used. foot_z is the ground height in the body frame.
class FootPlanner {
 public:
  explicit FootPlanner(const FootPlannerParams& params = FootPlannerParams());

  void setParams(const FootPlannerParams& params) { params_ = params; }
  const FootPlannerParams& params() const { return params_; }

  // Stateless foothold. v and v_cmd are body linear velocities (only x, y used).
  Vec3 foothold(const Vec3& p_nominal, const Vec3& v, const Vec3& v_cmd, double t_stance,
                double foot_z, double yaw_rate = 0.0, double yaw_rate_cmd = 0.0) const;

  // Per-leg foothold during swing: recomputed every call until swing_progress
  // reaches freeze_progress, then held until the next swing starts.
  Vec3 update(int leg, double swing_progress, const Vec3& p_nominal, const Vec3& v,
              const Vec3& v_cmd, double t_stance, double foot_z, double yaw_rate = 0.0,
              double yaw_rate_cmd = 0.0);

  // First-order low-pass on the body velocity; call once per control cycle and
  // pass the result to update(). The body velocity ripples within every step,
  // and unfiltered that ripple shakes the foothold and the swing path.
  const Vec3& filterVelocity(const Vec3& v, double dt);

  void reset();

 private:
  FootPlannerParams params_;
  std::array<Vec3, kNumLegs> held_;
  std::array<double, kNumLegs> last_progress_;
  Vec3 v_filtered_;
  bool has_v_filtered_;
};

}  // namespace go1
