// Owner: Member 4. Swing leg control.
#include "go1_ctrl/swing_controller.h"

#include <algorithm>
#include <cmath>

#include "go1_ctrl/estimator.h"  // defaultGo1Geometry() (Member 2)

namespace go1 {

SwingTarget cycloidSwing(const Vec3& p0, const Vec3& pf, double height, double s,
                         double t_swing, const Vec3& pf_rate) {
  const double w = 2.0 * M_PI;
  s = std::max(0.0, std::min(1.0, s));

  // Cycloid blend c(s) = s - sin(2 pi s) / (2 pi), with c' = 1 - cos(2 pi s).
  const double c = s - std::sin(w * s) / w;
  const double dc = 1.0 - std::cos(w * s);

  SwingTarget target;
  target.p = p0 + (pf - p0) * c;
  target.p.z() += 0.5 * height * (1.0 - std::cos(w * s));

  // d/dt [p0 + (pf - p0) c(s)] = (pf - p0) c'(s) ds/dt + c(s) dpf/dt
  target.v = (pf - p0) * dc / t_swing + c * pf_rate;
  target.v.z() += 0.5 * height * w * std::sin(w * s) / t_swing;
  return target;
}

Vec3 nominalFoot(const LegGeometry& g) {
  return Vec3(g.hipPosition_B.x(), g.hipPosition_B.y() + g.lateralOffset, 0.0);
}

SwingController::SwingController(const SwingParams& params)
    : SwingController(params, defaultGo1Geometry()) {}

SwingController::SwingController(const SwingParams& params,
                                 const std::array<LegGeometry, kNumLegs>& geometry)
    : params_(params), geometry_(geometry), kin_(geometry), t_prev_(0.0), ik_failures_(0) {
  swinging_.fill(false);
  last_progress_.fill(0.0);
  last_t_.fill(0.0);
  progress_rate_.fill(0.0);
  p_liftoff_.fill(Vec3::Zero());
  q_des_.fill(Vec3::Zero());
  target_.fill(SwingTarget());
  measured_.fill(SwingTarget());
  foothold_.fill(Vec3::Zero());
}

void SwingController::compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd,
                              RobotCommand& out) {
  // RobotState::vel is in the world frame; the planner works in the body frame
  // and uses the low-passed velocity.
  const Vec3 v_body = planner_.filterVelocity(s.quat.conjugate() * s.vel, s.t - t_prev_);
  t_prev_ = s.t;
  const double l2 = params_.dls_lambda * params_.dls_lambda;

  for (int leg = 0; leg < kNumLegs; ++leg) {
    const LegGait& lg = g.leg[leg];
    const Vec3 q = s.legQ(leg);
    const Vec3 dq = s.legDq(leg);
    LegCommand& l = out.leg[leg];
    l.kp = params_.joint_kp;
    l.kd = params_.joint_kd;

    // Kinematics throws on non-finite angles. Pass them through instead, so that
    // main.cpp's isFinite() check sends the robot to PASSIVE.
    if (!q.allFinite() || !dq.allFinite()) {
      swinging_[leg] = false;
      l.q = q;
      l.dq = dq;
      l.tau.setZero();
      continue;
    }

    // Measured foot state in the body frame (FK already includes the hip offset).
    const Mat3 J = kin_.jacobian(leg, q);
    const Vec3 p = kin_.forwardKinematics(leg, q);
    const Vec3 v = J * dq;
    measured_[leg].p = p;
    measured_[leg].v = v;

    if (lg.contact) {
      // Stance leg: hold in place. The blender normally uses the stance command here.
      swinging_[leg] = false;
      target_[leg].p = p;
      target_[leg].v.setZero();
      l.q = q;
      l.dq.setZero();
      l.tau.setZero();
      continue;
    }

    // Swing timing. GaitState has no duty factor, so measure 1 / T_swing as the
    // rate of the swing progress; until then fall back to a fraction of the period.
    const double sp = lg.swing_progress;
    const bool new_swing = !swinging_[leg] || sp < last_progress_[leg];
    const double dt_leg = s.t - last_t_[leg];
    if (new_swing) {
      p_liftoff_[leg] = p;
      q_des_[leg] = q;
    } else if (sp > last_progress_[leg] && dt_leg > 0.0) {
      progress_rate_[leg] = (sp - last_progress_[leg]) / dt_leg;
    }
    swinging_[leg] = true;
    last_progress_[leg] = sp;
    last_t_[leg] = s.t;
    if (progress_rate_[leg] <= 0.0) {
      const double period = g.period > 0.0 ? g.period : 0.5;
      progress_rate_[leg] = 1.0 / (params_.default_swing_fraction * period);
    }
    const double t_swing = 1.0 / progress_rate_[leg];
    const double t_stance = std::max(0.0, g.period - t_swing);

    // Foothold on the ground at the liftoff height, then the cycloid towards it.
    const Vec3 foothold_prev = foothold_[leg];
    foothold_[leg] = planner_.update(leg, sp, nominalFoot(geometry_[leg]), v_body, cmd.vel,
                                     t_stance, p_liftoff_[leg].z(), s.gyro.z(), cmd.yaw_rate);
    Vec3 foothold_rate = Vec3::Zero();
    if (!new_swing && dt_leg > 0.0) foothold_rate = (foothold_[leg] - foothold_prev) / dt_leg;
    const SwingTarget target = cycloidSwing(p_liftoff_[leg], foothold_[leg], params_.step_height,
                                            sp, t_swing, foothold_rate);
    target_[leg] = target;

    // Joint targets. IK picks the solution closest to the last q_des (smooth, same
    // knee branch). If the target is unreachable or outside the joint limits, IK
    // returns false and leaves q_des_ unchanged, so the leg holds its last valid pose.
    Vec3 q_ik = q_des_[leg];
    if (kin_.inverseKinematics(leg, target.p, q_des_[leg], q_ik)) {
      q_des_[leg] = q_ik;
    } else {
      ++ik_failures_;
    }
    l.q = q_des_[leg];
    // Damped least squares instead of a plain inverse: dq = (J^T J + l^2 I)^-1 J^T v.
    const Mat3 JtJ = J.transpose() * J + l2 * Mat3::Identity();
    l.dq = JtJ.ldlt().solve(J.transpose() * target.v);

    // Cartesian impedance feedforward.
    const Vec3 force =
        params_.cart_kp.cwiseProduct(target.p - p) + params_.cart_kd.cwiseProduct(target.v - v);
    const Vec3 tau = J.transpose() * force;
    l.tau = tau.cwiseMax(-params_.tau_ff_limit).cwiseMin(params_.tau_ff_limit);
  }
}

}  // namespace go1
