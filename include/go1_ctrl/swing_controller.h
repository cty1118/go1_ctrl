#pragma once
// OWNER: Member 4 (Tianyu Cheng). PLACEHOLDER written by Member 1 so the loop runs.
// Proposed interface, confirm at the freeze meeting. Replace the body in src/swing_controller.cpp.
#include "go1_ctrl/types.h"

namespace go1 {
class SwingController {
 public:
  // Command for every leg as if it were swinging; the blender picks per leg.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);
};
}  // namespace go1
