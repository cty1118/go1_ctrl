#pragma once
// Owner: Member 6. Contact detection from foot force with hysteresis.
// LOCAL TEST VERSION (Member 4): hysteresis instead of a single threshold, so the
// contact does not flicker when a foot carries 20-40 N in the four-leg phase of a trot.
// main.cpp calls this interface: keep it.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {
class ContactDetector {
 public:
  // Actual contact per leg from the foot forces.
  std::array<bool, kNumLegs> update(const RobotState& s);

  double on_force = 15.0;  // contact starts above this force [N]
  double off_force = 5.0;  // and ends below this one [N]

 private:
  std::array<bool, kNumLegs> contact_{};
};
}  // namespace go1
