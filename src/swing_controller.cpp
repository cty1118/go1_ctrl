// Owner: Member 4. Swing leg control.
#include "go1_ctrl/swing_controller.h"

#include <algorithm>
#include <cmath>

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

// ---- TEMPORARY leg kinematics (until Member 2's kinematics.h) ----
// Go1 URDF: hip joints at (+-0.1881, +-0.04675, 0), hip-to-thigh offset 0.08,
// thigh and calf 0.213 m. Right legs (FR, RR) have the thigh offset on -y.
namespace swing_kin {
namespace {
constexpr double kHipX = 0.1881;
constexpr double kHipY = 0.04675;
constexpr double kAbad = 0.08;
constexpr double kThigh = 0.213;
constexpr double kCalf = 0.213;

double sideSign(int leg) { return (leg == FR || leg == RR) ? -1.0 : 1.0; }
}  // namespace

Vec3 hipOffset(int leg) {
  const double sx = (leg == FR || leg == FL) ? 1.0 : -1.0;
  return Vec3(sx * kHipX, sideSign(leg) * kHipY, 0.0);
}

Vec3 nominalFoot(int leg) { return hipOffset(leg) + Vec3(0.0, sideSign(leg) * kAbad, 0.0); }

Vec3 fk(int leg, const Vec3& q) {
  const double l1 = sideSign(leg) * kAbad, l2 = -kThigh, l3 = -kCalf;
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const double s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  return Vec3(l3 * s23 + l2 * s2,
              -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1,
              l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);
}

Mat3 jacobian(int leg, const Vec3& q) {
  const double l1 = sideSign(leg) * kAbad, l2 = -kThigh, l3 = -kCalf;
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const double s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  Mat3 J;
  J << 0.0, l3 * c23 + l2 * c2, l3 * c23,
      -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1, l3 * s1 * s23 + l2 * s1 * s2, l3 * s1 * s23,
      -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1, -l3 * c1 * s23 - l2 * c1 * s2, -l3 * c1 * s23;
  return J;
}

Vec3 ik(int leg, const Vec3& p) {
  const double l1 = sideSign(leg) * kAbad;
  const double a = kThigh, b = kCalf;

  // Hip: [py, pz] is [l1, zp] rotated about x by q1, zp = -(leg length in the leg plane).
  const double zp = -std::sqrt(std::max(p.y() * p.y() + p.z() * p.z() - l1 * l1, 1e-9));
  const double q1 = std::atan2(p.z(), p.y()) - std::atan2(zp, l1);

  // Knee: law of cosines in the leg plane, knee bent backwards (q3 < 0).
  const double c3 = (p.x() * p.x() + zp * zp - a * a - b * b) / (2.0 * a * b);
  const double q3 = -std::acos(std::max(-1.0, std::min(1.0, c3)));

  // Thigh: (-x, -zp) = (k1 s2 + k2 c2, k1 c2 - k2 s2).
  const double k1 = a + b * std::cos(q3), k2 = b * std::sin(q3);
  const double q2 = std::atan2(-p.x(), -zp) - std::atan2(k2, k1);
  return Vec3(q1, q2, q3);
}
}  // namespace swing_kin

SwingController::SwingController(const SwingParams& params) : params_(params), t_prev_(0.0) {
  swinging_.fill(false);
  last_progress_.fill(0.0);
  last_t_.fill(0.0);
  progress_rate_.fill(0.0);
  p_liftoff_.fill(Vec3::Zero());
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
    const Vec3 hip = swing_kin::hipOffset(leg);
    const Mat3 J = swing_kin::jacobian(leg, q);
    const Vec3 p = hip + swing_kin::fk(leg, q);
    const Vec3 v = J * dq;
    measured_[leg].p = p;
    measured_[leg].v = v;

    LegCommand& l = out.leg[leg];
    l.kp = params_.joint_kp;
    l.kd = params_.joint_kd;

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
    foothold_[leg] = planner_.update(leg, sp, swing_kin::nominalFoot(leg), v_body, cmd.vel,
                                     t_stance, p_liftoff_[leg].z(), s.gyro.z(), cmd.yaw_rate);
    Vec3 foothold_rate = Vec3::Zero();
    if (!new_swing && dt_leg > 0.0) foothold_rate = (foothold_[leg] - foothold_prev) / dt_leg;
    const SwingTarget target = cycloidSwing(p_liftoff_[leg], foothold_[leg], params_.step_height,
                                            sp, t_swing, foothold_rate);
    target_[leg] = target;

    // Joint targets. Damped least squares instead of a plain inverse:
    // dq = (J^T J + l^2 I)^-1 J^T v.
    const Mat3 JtJ = J.transpose() * J + l2 * Mat3::Identity();
    l.q = swing_kin::ik(leg, target.p - hip);
    l.dq = JtJ.ldlt().solve(J.transpose() * target.v);

    // Cartesian impedance feedforward.
    const Vec3 force =
        params_.cart_kp.cwiseProduct(target.p - p) + params_.cart_kd.cwiseProduct(target.v - v);
    const Vec3 tau = J.transpose() * force;
    l.tau = tau.cwiseMax(-params_.tau_ff_limit).cwiseMin(params_.tau_ff_limit);
  }
}

}  // namespace go1
