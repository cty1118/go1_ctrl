#pragma once
// Owner: Member 6. Blending between swing and stance commands at touchdown and lift-off.
// PLACEHOLDER (by Godfred) so the whole loop builds and runs.
// Proposed interface, confirm at the freeze meeting. Replace the body in src/transition_blender.cpp.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {
class TransitionBlender {
 public:
  // Per leg: mix swing and stance commands around touchdown and liftoff.
  void blend(const GaitState& g, const std::array<bool, kNumLegs>& contact,
             const RobotCommand& swing, const RobotCommand& stance, RobotCommand& out);
};
}  // namespace go1
