// Owner: Member 5. Stance leg impedance control.
#include "go1_ctrl/stance_controller.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace go1 {

// =====================================================================================
// TODO(Member 5): TEMPORARY leg kinematics. REMOVE when Member 2's kinematics.h lands.
//   Copied from Member 4's swing_kin (feature/swing_controller). See the note in
//   stance_controller.h for the replacement steps.
// =====================================================================================
// Go1 URDF: hip joints at (+-0.1881, +-0.04675, 0), hip-to-thigh offset 0.08,
// thigh and calf 0.213 m. Right legs (FR, RR) have the thigh offset on -y.
namespace stance_kin {
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
}  // namespace stance_kin

// ---- parameters ----
namespace {
void loadVec3(ros::NodeHandle& nh, const std::string& name, Vec3& v) {
  std::vector<double> x;
  if (!nh.getParam(name, x)) return;
  if (x.size() == 3)
    v = Vec3(x[0], x[1], x[2]);
  else
    ROS_WARN("[stance] param %s must have 3 values, keeping the default", name.c_str());
}
}  // namespace

StanceController::StanceController() {
  // config/stance.yaml, loaded under the controller node as ~stance/...
  ros::NodeHandle nh("~stance");
  nh.param("mass", p_.mass, p_.mass);
  nh.param("gravity", p_.gravity, p_.gravity);
  loadVec3(nh, "kp", p_.kp);
  loadVec3(nh, "kd", p_.kd);
  nh.param("kp_roll", p_.kp_roll, p_.kp_roll);
  nh.param("kd_roll", p_.kd_roll, p_.kd_roll);
  nh.param("kp_pitch", p_.kp_pitch, p_.kp_pitch);
  nh.param("kd_pitch", p_.kd_pitch, p_.kd_pitch);
  loadVec3(nh, "joint_kp", p_.joint_kp);
  loadVec3(nh, "joint_kd", p_.joint_kd);
  loadVec3(nh, "tau_max", p_.tau_max);
}

// ---- control law ----
void StanceController::compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd,
                               RobotCommand& out) {
  // Body orientation. R maps body axes to world axes.
  const Mat3 R = s.quat.normalized().toRotationMatrix();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  // Heading-only rotation: the references follow the body's yaw but stay level.
  const Mat3 R_yaw = Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
  const Vec3 omega_w = R * s.gyro;  // body angular velocity, world axes

  // Feet in world-aligned axes, relative to the body origin.
  std::array<Vec3, kNumLegs> p_w, v_w;
  std::array<Mat3, kNumLegs> J;
  int n_stance = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 q = s.legQ(leg);
    J[leg] = stance_kin::jacobian(leg, q);
    const Vec3 p_b = stance_kin::hipOffset(leg) + stance_kin::fk(leg, q);
    const Vec3 v_b = J[leg] * s.legDq(leg) + s.gyro.cross(p_b);  // d/dt (R p_b) = R (w x p + p')
    p_w[leg] = R * p_b;
    v_w[leg] = R * v_b;
    if (g.leg[leg].contact) ++n_stance;
  }

  // Gravity share: each stance leg carries mg / n. Points down, because F is the force
  // the leg applies to the ground (the ground pushes the body up in return).
  const double share = p_.mass * p_.gravity / std::max(n_stance, 1);
  const Vec3 F_ff(0.0, 0.0, -share);

  // Attitude PD: the body torque that would level the body, spread over the stance legs as
  // extra vertical force. Torque about x from an upward force f at y is y*f, about y it is -x*f.
  const double tau_x = -p_.kp_roll * roll - p_.kd_roll * omega_w.x();
  const double tau_y = -p_.kp_pitch * pitch - p_.kd_pitch * omega_w.y();
  double sum_xx = 0.0, sum_yy = 0.0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (!g.leg[leg].contact) continue;
    sum_xx += p_w[leg].x() * p_w[leg].x();
    sum_yy += p_w[leg].y() * p_w[leg].y();
  }

  // Desired foot motion relative to the body: feet stay put on the ground while the body
  // moves at the commanded velocity, so the feet move backwards relative to it.
  const Vec3 body_vel_w = R_yaw * cmd.vel;
  const Vec3 yaw_rate_w(0.0, 0.0, cmd.yaw_rate);
  const Vec3 com_shift_w = R_yaw * Vec3(g.com_shift.x(), g.com_shift.y(), 0.0);

  for (int leg = 0; leg < kNumLegs; ++leg) {
    // Reference: under the hip at the target height, moved opposite to the CoM shift
    // (moving the body by +d moves the feet by -d relative to it).
    const Vec3 nominal = stance_kin::nominalFoot(leg);
    Vec3 p_ref = R_yaw * Vec3(nominal.x(), nominal.y(), 0.0) - com_shift_w;
    p_ref.z() = -cmd.height;
    const Vec3 v_ref = -body_vel_w - yaw_rate_w.cross(p_w[leg]);

    // Virtual spring-damper + gravity share.
    Vec3 F = p_.kp.cwiseProduct(p_ref - p_w[leg]) + p_.kd.cwiseProduct(v_ref - v_w[leg]) + F_ff;

    // Attitude correction (stance legs only). f_up pushes the body up at this foot,
    // so the leg pushes the ground down by the same amount.
    if (g.leg[leg].contact) {
      double f_up = 0.0;
      if (sum_yy > 1e-4) f_up += tau_x * p_w[leg].y() / sum_yy;
      if (sum_xx > 1e-4) f_up -= tau_y * p_w[leg].x() / sum_xx;
      F.z() -= f_up;
    }
    force_[leg] = F;

    // Force -> joint torque: rotate into body (= leg) axes, then tau = J^T F.
    Vec3 tau = J[leg].transpose() * (R.transpose() * F);
    for (int j = 0; j < kJointsPerLeg; ++j)
      tau[j] = std::max(-p_.tau_max[j], std::min(p_.tau_max[j], tau[j]));

    LegCommand& l = out.leg[leg];
    l.q = s.legQ(leg);  // with joint_kp = 0 the position target has no effect
    l.dq.setZero();     // joint_kd adds light damping
    l.kp = p_.joint_kp;
    l.kd = p_.joint_kd;
    l.tau = tau;
  }
}

}  // namespace go1
