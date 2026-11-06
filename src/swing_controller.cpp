// OWNER: Member 4. PLACEHOLDER: joint-space lift around the standing pose (no kinematics).
// Replace with the Bezier/cycloid swing + IK + Cartesian impedance.
#include "go1_ctrl/swing_controller.h"

#include <cmath>

namespace go1 {
void SwingController::compute(const RobotState& /*s*/, const GaitState& g, const BodyCmd& /*cmd*/,
                              RobotCommand& out) {
  const Vec3 stand(0.0, 0.67, -1.3);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double lift = std::sin(M_PI * g.leg[leg].swing_progress);  // 0 -> 1 -> 0
    LegCommand& l = out.leg[leg];
    l.q = stand + lift * Vec3(0.0, 0.25, -0.5);  // fold the leg up
    l.dq.setZero();
    l.kp.setConstant(60.0);
    l.kd.setConstant(3.0);
    l.tau.setZero();
  }
}
}  // namespace go1
