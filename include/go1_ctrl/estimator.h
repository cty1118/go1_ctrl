#pragma once
// Owner: Member 2. Body state estimation (attitude, velocity, height).
// PLACEHOLDER (by Godfred) so the whole loop builds and runs.
// main.cpp calls this interface: keep it, and replace the body in src/estimator.cpp.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {
class Estimator {
 public:
  // Fill s.pos / s.vel (and refine s.quat) from IMU + stance-leg kinematics.
  // contact: actual contact flags from the ContactDetector.
  void update(RobotState& s, const std::array<bool, kNumLegs>& contact);
};
}  // namespace go1
