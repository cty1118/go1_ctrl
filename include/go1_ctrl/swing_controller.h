#pragma once
// Owner: Member 4. Swing leg trajectory, position tracking and impedance feedforward.
// PLACEHOLDER (by Godfred) so the whole loop builds and runs.
// main.cpp calls this interface: keep it, and replace the body in src/swing_controller.cpp.
#include "go1_ctrl/types.h"

namespace go1 {
class SwingController {
 public:
  // Command for every leg as if it were swinging; the blender picks per leg.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);
};
}  // namespace go1
