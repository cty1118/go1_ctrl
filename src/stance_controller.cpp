// Owner: Member 5. Stance leg impedance control.
#include "go1_ctrl/stance_controller.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "go1_ctrl/estimator.h"  // defaultGo1Geometry() (Member 2)

namespace go1 {

namespace {
void loadVec3(ros::NodeHandle& nh, const std::string& name, Vec3& v) {
  std::vector<double> x;
  if (!nh.getParam(name, x)) return;
  if (x.size() == 3)
    v = Vec3(x[0], x[1], x[2]);
  else
    ROS_WARN("[stance] param %s must have 3 values, keeping the default", name.c_str());
}

// Neutral foot (x, y) under the hip, body frame, z = 0: hip joint plus the hip-to-thigh offset.
Vec3 footUnderHip(const LegGeometry& g) {
  return Vec3(g.hipPosition_B.x(), g.hipPosition_B.y() + g.lateralOffset, 0.0);
}
}  // namespace

// ---- construction and parameters ----
StanceController::StanceController(const StanceParams& p,
                                   const std::array<LegGeometry, kNumLegs>& geometry)
    : p_(p), geometry_(geometry), kin_(geometry) {}

StanceController::StanceController(const StanceParams& p)
    : StanceController(p, defaultGo1Geometry()) {}

StanceController::StanceController() : StanceController(StanceParams(), defaultGo1Geometry()) {
  // config/stance.yaml, loaded under the controller node as ~stance/... (sim.launch)
  ros::NodeHandle nh("~stance");
  nh.param("mass", p_.mass, p_.mass);
  nh.param("gravity", p_.gravity, p_.gravity);
  nh.param("height", p_.height, p_.height);
  loadVec3(nh, "kp", p_.kp);
  loadVec3(nh, "kd", p_.kd);
  nh.param("kp_roll", p_.kp_roll, p_.kp_roll);
  nh.param("kd_roll", p_.kd_roll, p_.kd_roll);
  nh.param("kp_pitch", p_.kp_pitch, p_.kp_pitch);
  nh.param("kd_pitch", p_.kd_pitch, p_.kd_pitch);
  nh.param("ki_roll", p_.ki_roll, p_.ki_roll);
  nh.param("ki_pitch", p_.ki_pitch, p_.ki_pitch);
  nh.param("i_max", p_.i_max, p_.i_max);
  loadVec3(nh, "joint_kp", p_.joint_kp);
  loadVec3(nh, "joint_kd", p_.joint_kd);
  loadVec3(nh, "tau_max", p_.tau_max);
  nh.param("balance", p_.balance, p_.balance);
  nh.param("kp_height", p_.kp_height, p_.kp_height);
  nh.param("kd_height", p_.kd_height, p_.kd_height);
  nh.param("kd_vel_xy", p_.kd_vel_xy, p_.kd_vel_xy);
  nh.param("kp_com", p_.kp_com, p_.kp_com);
  nh.param("kp_hold", p_.kp_hold, p_.kp_hold);
  nh.param("hold_max", p_.hold_max, p_.hold_max);
  nh.param("kd_yaw", p_.kd_yaw, p_.kd_yaw);
  nh.param("w_torque", p_.w_torque, p_.w_torque);
  nh.param("reg", p_.reg, p_.reg);
  nh.param("friction", p_.friction, p_.friction);
  nh.param("fz_min", p_.fz_min, p_.fz_min);
  nh.param("fz_max", p_.fz_max, p_.fz_max);
  nh.param("support_force", p_.support_force, p_.support_force);
  ROS_INFO("[stance] roll %.0f/%.1f/%.0f pitch %.0f/%.1f/%.0f (P/D/I) joint kp %.0f kd %.1f height %s",
           p_.kp_roll, p_.kd_roll, p_.ki_roll, p_.kp_pitch, p_.kd_pitch, p_.ki_pitch,
           p_.joint_kp.x(), p_.joint_kd.x(),
           p_.height > 0.0 ? std::to_string(p_.height).c_str() : "from BodyCmd");
  ROS_INFO("[stance] balance %s: height %.0f/%.0f vel_xy %.0f yaw %.1f w_torque %.0f mu %.2f",
           p_.balance ? "ON" : "off", p_.kp_height, p_.kd_height, p_.kd_vel_xy, p_.kd_yaw,
           p_.w_torque, p_.friction);
}

// ---- control law ----
void StanceController::compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd,
                               RobotCommand& out) {
  // Kinematics throws on non-finite joint angles. Safety normally stops the loop
  // before that; if not, send a passive command and let Safety handle it.
  if (!s.q.allFinite() || !s.dq.allFinite()) {
    for (int leg = 0; leg < kNumLegs; ++leg) {
      LegCommand& l = out.leg[leg];
      l.q = s.legQ(leg);
      l.dq.setZero();
      l.kp.setZero();
      l.kd = p_.joint_kd;
      l.tau.setZero();
    }
    return;
  }

  const double height = p_.height > 0.0 ? p_.height : cmd.height;

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
    J[leg] = kin_.jacobian(leg, q);
    const Vec3 p_b = kin_.forwardKinematics(leg, q);  // body frame, hip offset included
    const Vec3 v_b = J[leg] * s.legDq(leg) + s.gyro.cross(p_b);  // d/dt (R p_b) = R (w x p + p')
    p_w[leg] = R * p_b;
    v_w[leg] = R * v_b;
    if (g.leg[leg].contact) ++n_stance;
  }

  // Gravity share: each stance leg carries mg / n. Points down, because F is the force
  // the leg applies to the ground (the ground pushes the body up in return).
  const double share = p_.mass * p_.gravity / std::max(n_stance, 1);
  const Vec3 F_ff(0.0, 0.0, -share);

  // Attitude integral, only while at least one foot supports the body. A gap of more than
  // 0.1 s means GAIT was left and entered again: start from zero.
  const double dt = s.t - t_prev_;
  if (!has_t_ || dt > 0.1 || dt < 0.0) {
    i_roll_ = 0.0;
    i_pitch_ = 0.0;
  } else if (n_stance > 0) {
    i_roll_ = std::max(-p_.i_max, std::min(p_.i_max, i_roll_ - p_.ki_roll * roll * dt));
    i_pitch_ = std::max(-p_.i_max, std::min(p_.i_max, i_pitch_ - p_.ki_pitch * pitch * dt));
  }
  t_prev_ = s.t;
  has_t_ = true;

  // Attitude PID: the body torque that would level the body, spread over the stance legs as
  // extra vertical force. Torque about x from an upward force f at y is y*f, about y it is -x*f.
  const double tau_x = -p_.kp_roll * roll - p_.kd_roll * omega_w.x() + i_roll_;
  const double tau_y = -p_.kp_pitch * pitch - p_.kd_pitch * omega_w.y() + i_pitch_;
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

  // ---- Balance controller: ground reaction forces of all stance legs together ----
  // grf[leg] is the force of the ground ON the foot, world-aligned axes. Horizontal forces
  // are allowed, so a torque about the line between two diagonal feet is possible.
  std::array<Vec3, kNumLegs> grf{{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero()}};
  bool balance_ok = false;
  // Support feet: scheduled in stance AND carrying load. A foot scheduled in stance that
  // has not touched down yet (late touchdown, Walk waiting) pushes nothing; in the solve it
  // would bias the height estimate and take force away from the feet that are down.
  std::array<bool, kNumLegs> support{};
  int n_support = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    support[leg] = g.leg[leg].contact && s.foot_force[leg] > p_.support_force;
    if (support[leg]) ++n_support;
  }
  if (p_.balance && n_support > 0) {
    // Body height above the stance feet and body velocity from leg odometry: a stance
    // foot does not move, so the body moves at minus the foot velocity relative to it.
    double h = 0.0;
    Vec3 v_body = Vec3::Zero();
    std::array<int, kNumLegs> idx{};
    int n = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (!support[leg]) continue;
      h -= p_w[leg].z();
      v_body -= v_w[leg];
      idx[n++] = leg;
    }
    h /= n;
    v_body /= n;

    // Desired net force and torque on the body (gravity compensation included).
    Eigen::Matrix<double, 6, 1> b;
    // Station keeping (no command, not Walk): pull back to the held world position.
    Vec3 f_hold = Vec3::Zero();
    const bool no_cmd = cmd.vel.head<2>().norm() < 1e-3 && std::abs(cmd.yaw_rate) < 1e-3;
    if (no_cmd && g.com_shift.head<2>().norm() < 1e-6 && s.pos.allFinite()) {
      if (!hold_valid_ || dt > 0.1 || dt < 0.0) {
        hold_pos_ = s.pos;
        hold_valid_ = true;
      }
      f_hold.head<2>() = p_.kp_hold * (hold_pos_ - s.pos).head<2>();
      const double fn = f_hold.head<2>().norm();
      if (fn > p_.hold_max) f_hold *= p_.hold_max / fn;
    } else {
      hold_valid_ = false;
    }
    // Walk: g.com_shift is how far the CoM still has to move to get over the support
    // triangle (Member 3); push the body that way.
    b << p_.kd_vel_xy * (body_vel_w.x() - v_body.x()) + p_.kp_com * com_shift_w.x(),
        p_.kd_vel_xy * (body_vel_w.y() - v_body.y()) + p_.kp_com * com_shift_w.y(),
        p_.mass * p_.gravity + p_.kp_height * (height - h) - p_.kd_height * v_body.z(),
        tau_x, tau_y, p_.kd_yaw * (cmd.yaw_rate - omega_w.z());
    b(0) += f_hold.x();
    b(1) += f_hold.y();

    // A f = [sum f_i; sum r_i x f_i]
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 3 * n);
    for (int k = 0; k < n; ++k) {
      const Vec3& r = p_w[idx[k]];
      Mat3 rx;
      rx << 0.0, -r.z(), r.y(),
            r.z(), 0.0, -r.x(),
            -r.y(), r.x(), 0.0;
      A.block<3, 3>(0, 3 * k) = Mat3::Identity();
      A.block<3, 3>(3, 3 * k) = rx;
    }
    Eigen::Matrix<double, 6, 1> w;
    w << 1.0, 1.0, 1.0, p_.w_torque, p_.w_torque, p_.w_torque;
    const Eigen::MatrixXd AtW = A.transpose() * w.asDiagonal();
    const Eigen::MatrixXd H = AtW * A + p_.reg * Eigen::MatrixXd::Identity(3 * n, 3 * n);
    const Eigen::VectorXd f = H.ldlt().solve(AtW * b);

    if (f.allFinite()) {
      balance_ok = true;
      for (int k = 0; k < n; ++k) {
        // Friction cone (as a cylinder clip): fz in [fz_min, fz_max], |f_xy| <= mu fz.
        Vec3 fi = f.segment<3>(3 * k);
        fi.z() = std::max(p_.fz_min, std::min(p_.fz_max, fi.z()));
        const double ft = std::hypot(fi.x(), fi.y());
        const double ft_max = p_.friction * fi.z();
        if (ft > ft_max) {
          fi.x() *= ft_max / ft;
          fi.y() *= ft_max / ft;
        }
        grf[idx[k]] = fi;
      }
    }
  }

  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 q = s.legQ(leg);

    // Reference: under the hip at the target height, moved opposite to the CoM shift
    // (moving the body by +d moves the feet by -d relative to it).
    const Vec3 nominal = footUnderHip(geometry_[leg]);
    Vec3 p_ref = R_yaw * Vec3(nominal.x(), nominal.y(), 0.0) - com_shift_w;
    p_ref.z() = -height;
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
    // Balance controller: the foot pushes the ground with minus the ground reaction force.
    if (balance_ok && support[leg]) F = -grf[leg];
    force_[leg] = F;

    // Force -> joint torque: rotate into body (= leg) axes, then tau = J^T F.
    Vec3 tau = J[leg].transpose() * (R.transpose() * F);
    for (int j = 0; j < kJointsPerLeg; ++j)
      tau[j] = std::max(-p_.tau_max[j], std::min(p_.tau_max[j], tau[j]));

    // Joint PD target: the foot where it is now horizontally, `height` below a LEVEL body.
    // In the body frame that point moves when the body rolls or pitches, so the joint PD
    // pushes the low side up and the high side down. If IK fails, no joint stiffness.
    Vec3 q_ref = q;
    bool ik_ok = false;
    if (g.leg[leg].contact) {
      const Vec3 target_w(p_w[leg].x(), p_w[leg].y(), -height);
      ik_ok = kin_.inverseKinematics(leg, R.transpose() * target_w, q, q_ref);
    }

    LegCommand& l = out.leg[leg];
    l.q = q_ref;
    l.dq.setZero();  // joint_kd adds light damping
    l.kp = ik_ok ? p_.joint_kp : Vec3::Zero();
    l.kd = p_.joint_kd;
    l.tau = tau;
  }
}

}  // namespace go1
