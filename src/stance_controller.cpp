// Owner: Member 5. Stance leg impedance control.
// PLACEHOLDER (by Godfred): stiff joint PD on the standing pose (no impedance yet).
// Replace with F = K(p_ref - p) + D(v_ref - v) + F_ff and tau = J^T F.
#include "go1_ctrl/stance_controller.h"

namespace go1 {
void StanceController::compute(const RobotState& /*s*/, const GaitState& /*g*/,
                               const BodyCmd& /*cmd*/, RobotCommand& out) {
  const Vec3 stand(0.0, 0.67, -1.3);
  for (LegCommand& l : out.leg) {
    l.q = stand;
    l.dq.setZero();
    l.kp.setConstant(180.0);
    l.kd.setConstant(8.0);
    l.tau.setZero();
  }
}
}  // namespace go1
