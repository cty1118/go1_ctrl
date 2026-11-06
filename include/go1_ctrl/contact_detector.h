#pragma once
// Owner: Member 6. Contact detection from foot force with hysteresis.
// PLACEHOLDER (by Godfred) so the whole loop builds and runs.
// Proposed interface, confirm at the freeze meeting. Replace the body in src/contact_detector.cpp.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {
class ContactDetector {
 public:
  // Actual contact per leg from the foot forces.
  std::array<bool, kNumLegs> update(const RobotState& s);
};
}  // namespace go1
