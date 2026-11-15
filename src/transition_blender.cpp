// Owner: Member 6. Transition blending.
// LOCAL TEST VERSION (Member 4): contact-aware switch with a ramped stance command.
#include "go1_ctrl/transition_blender.h"

#include <algorithm>

namespace go1 {

namespace {
// Mix two leg commands so that the motor torque tau + kp (q_des - q) + kd (dq_des - dq)
// is (1 - w) * swing torque + w * stance torque for any q, dq.
LegCommand mix(const LegCommand& a, const LegCommand& b, double w) {
  LegCommand m;
  for (int j = 0; j < kJointsPerLeg; ++j) {
    m.kp[j] = (1.0 - w) * a.kp[j] + w * b.kp[j];
    m.kd[j] = (1.0 - w) * a.kd[j] + w * b.kd[j];
    m.tau[j] = (1.0 - w) * a.tau[j] + w * b.tau[j];
    const double kq = (1.0 - w) * a.kp[j] * a.q[j] + w * b.kp[j] * b.q[j];
    const double kdq = (1.0 - w) * a.kd[j] * a.dq[j] + w * b.kd[j] * b.dq[j];
    m.q[j] = m.kp[j] > 1e-9 ? kq / m.kp[j] : (1.0 - w) * a.q[j] + w * b.q[j];
    m.dq[j] = m.kd[j] > 1e-9 ? kdq / m.kd[j] : (1.0 - w) * a.dq[j] + w * b.dq[j];
  }
  return m;
}
}  // namespace

void TransitionBlender::blend(const GaitState& g, const std::array<bool, kNumLegs>& contact,
                              const RobotCommand& swing, const RobotCommand& stance,
                              RobotCommand& out) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const LegGait& lg = g.leg[leg];

    bool use_stance;
    if (lg.contact) {
      wait_[leg] = contact[leg] ? 0.0 : wait_[leg] + dt;
      use_stance = contact[leg] || lg.stance_progress >= late_window || wait_[leg] >= late_time;
    } else {
      wait_[leg] = 0.0;
      use_stance = contact[leg] && lg.swing_progress >= early_window;
    }

    if (!use_stance)
      w_[leg] = 0.0;
    else
      w_[leg] = ramp_time > 0.0 ? std::min(1.0, w_[leg] + dt / ramp_time) : 1.0;

    if (w_[leg] <= 0.0)
      out.leg[leg] = swing.leg[leg];
    else if (w_[leg] >= 1.0)
      out.leg[leg] = stance.leg[leg];
    else
      out.leg[leg] = mix(swing.leg[leg], stance.leg[leg], w_[leg]);
  }
}

}  // namespace go1
