// Owner: Member 2. State estimator.
// PLACEHOLDER (by Godfred): uses Gazebo ground truth as-is. Replace with the real estimator.
#include "go1_ctrl/estimator.h"

namespace go1 {
void Estimator::update(RobotState& /*s*/, const std::array<bool, kNumLegs>& /*contact*/) {
  // Ground truth (if any) is already in s.pos / s.vel. On the real robot they stay zero.
}
}  // namespace go1
