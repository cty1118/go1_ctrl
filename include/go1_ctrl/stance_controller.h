#pragma once
// Owner: Member 5. Stance leg impedance control.
// PLACEHOLDER (by Godfred) so the whole loop builds and runs.
// main.cpp calls this interface: keep it, and replace the body in src/stance_controller.cpp.
#include "go1_ctrl/types.h"

namespace go1 {
class StanceController {
 public:
  // Command for every leg as if it were in stance; the blender picks per leg.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);
};
}  // namespace go1
