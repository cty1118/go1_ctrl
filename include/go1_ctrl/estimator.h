#pragma once
// OWNER: Member 2 (Thomas Buretz). PLACEHOLDER written by Member 1 so the loop runs.
// Proposed interface, confirm at the freeze meeting. Replace the body in src/estimator.cpp.
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
