// Owner: Member 6. Contact detection.
// PLACEHOLDER (by Godfred): plain threshold. Replace with hysteresis + debouncing.
#include "go1_ctrl/contact_detector.h"

namespace go1 {
std::array<bool, kNumLegs> ContactDetector::update(const RobotState& s) {
  std::array<bool, kNumLegs> c{};
  for (int leg = 0; leg < kNumLegs; ++leg) c[leg] = s.foot_force[leg] > 20.0;
  return c;
}
}  // namespace go1
