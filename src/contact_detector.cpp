// Owner: Member 6. Contact detection.
// LOCAL TEST VERSION (Member 4): force threshold with hysteresis.
#include "go1_ctrl/contact_detector.h"

namespace go1 {
std::array<bool, kNumLegs> ContactDetector::update(const RobotState& s) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double f = s.foot_force[leg];
    if (contact_[leg]) {
      if (f < off_force) contact_[leg] = false;
    } else if (f > on_force) {
      contact_[leg] = true;
    }
  }
  return contact_;
}
}  // namespace go1
