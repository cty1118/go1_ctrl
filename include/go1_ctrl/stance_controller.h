#pragma once
// Owner: Member 5. Stance leg impedance control.
// main.cpp calls compute(): keep its signature (see the README).
//
// Each stance leg acts as a virtual spring-damper between the body and its foot:
//   F_i = K (p_ref - p) + D (v_ref - v) + F_ff        (world-aligned axes, at the foot)
//   + attitude PD: extra vertical force from the roll / pitch errors
//   tau_i = J^T R^T F_i                                (rotate into the body/leg frame)
// F is the force the leg applies to the ground, so the gravity share F_ff points DOWN.
//
// On top of tau, a light joint PD holds each stance leg towards a LEVEL body at the
// target height (see joint_kp below). It resists roll, pitch and sinking directly in
// the joints, which the force terms alone did not do strongly enough on 2 legs (trot).
#include <array>
#include "go1_ctrl/kinematics.h"
#include "go1_ctrl/types.h"

namespace go1 {

struct StanceParams {
  double mass = 13.1;  // total robot mass [kg] (sum of go1.urdf links)
  double gravity = 9.81;

  // Body height target [m]. <= 0 uses BodyCmd::height (0.30 by default).
  double height = 0.0;

  // Virtual spring-damper per leg (x, y, z), world-aligned axes.
  Vec3 kp = Vec3(300.0, 300.0, 800.0);  // [N/m]
  Vec3 kd = Vec3(30.0, 30.0, 60.0);     // [N s/m]

  // Attitude PD: corrective body torque from roll / pitch error and rate.
  // Raised from 50/3 (roll) and 80/5 (pitch): on the ground the trot rolled 15-20 deg
  // every step (Member 7, PR #11), 20 deg gave only ~17 Nm of correction.
  double kp_roll = 150.0;   // [Nm/rad]
  double kd_roll = 6.0;     // [Nm s/rad]
  double kp_pitch = 150.0;  // [Nm/rad]
  double kd_pitch = 6.0;    // [Nm s/rad]

  // Attitude integral. Every ground test leaned nose-down and left-down (the CoM is not
  // at the body origin, but gravity is shared equally between the stance legs), and a PD
  // only corrects once the error is there. The integral builds up the missing torque.
  double ki_roll = 60.0;   // [Nm/(rad s)]
  double ki_pitch = 60.0;  // [Nm/(rad s)]
  double i_max = 10.0;     // clamp on each integral torque [Nm]

  // Joint PD during stance. The joint target is the IK of the foot at its CURRENT
  // horizontal position and the target height below a level body, so it resists roll,
  // pitch and sinking but does not hold the foot back while the body walks over it.
  // Was 0 / 1 (pure torque control).
  Vec3 joint_kp = Vec3(0.0, 0.0, 0.0);
  Vec3 joint_kd = Vec3(1.5, 1.5, 1.5);

  // ---- Balance controller (force distribution over all stance legs) ----
  // With only two diagonal feet on the ground, vertical forces cannot make a torque
  // about the line between them, so the per-leg law above cannot stop the body from
  // rolling over that line (ground tests, Nov 14). The balance controller solves for
  // the ground reaction forces of all stance feet together, horizontal ones included:
  //   minimise |A f - [F_des; T_des]|_W^2 + reg |f|^2,   A = [I ... I; [r_1]x ... [r_n]x]
  // then clips every force to the friction cone. balance = false: per-leg law only.
  bool balance = true;
  double kp_height = 1000.0;  // vertical force per height error [N/m]
  double kd_height = 100.0;   // [N s/m]
  double kd_vel_xy = 150.0;   // horizontal force per velocity error [N s/m]
  double kp_com = 600.0;      // Walk: horizontal force per metre of g.com_shift [N/m]
  // Station keeping: with no velocity command (and no Walk CoM shift), pull the body back
  // to where it stopped. Without it the in-place gallop drifted sideways.
  double kp_hold = 200.0;     // [N/m]
  double hold_max = 30.0;     // clamp on the station-keeping force [N]
  double kd_yaw = 5.0;        // yaw torque per yaw-rate error [Nm s/rad]
  double w_torque = 10.0;     // weight of the torque rows against the force rows
  double reg = 1e-3;          // regularisation on the forces
  double friction = 0.6;      // friction coefficient for the cone clip
  double fz_min = 5.0;        // minimum normal force of a stance foot [N]
  double fz_max = 250.0;      // maximum normal force of a stance foot [N]
  double support_force = 10.0;  // a stance foot counts as support above this force [N]

  // Clamp on the stance torque per joint (hip, thigh, calf) [Nm].
  // Below the motor limits in fsm.yaml; Safety clamps again after the blender.
  Vec3 tau_max = Vec3(20.0, 20.0, 30.0);
};

class StanceController {
 public:
  // Default Go1 geometry (Member 2's defaultGo1Geometry()). Reads the gains from the ROS
  // parameters ~stance/... if they exist (config/stance.yaml, loaded by sim.launch),
  // otherwise keeps the defaults in StanceParams.
  StanceController();
  explicit StanceController(const StanceParams& p);
  StanceController(const StanceParams& p, const std::array<LegGeometry, kNumLegs>& geometry);

  // Command for every leg as if it were in stance; the blender picks per leg.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);

  const StanceParams& params() const { return p_; }
  // Last foot force per leg (world-aligned axes, force the leg applies to the ground),
  // for the debug topic and plots.
  const Vec3& lastForce(int leg) const { return force_[leg]; }
  // Current integral torques about x (roll) and y (pitch) [Nm], for debugging.
  double integralRoll() const { return i_roll_; }
  double integralPitch() const { return i_pitch_; }

 private:
  StanceParams p_;
  std::array<LegGeometry, kNumLegs> geometry_;
  Kinematics kin_;  // Member 2
  std::array<Vec3, kNumLegs> force_{{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero()}};

  // Attitude integral state. Reset when compute() was not called for a while
  // (a new GAIT session).
  double i_roll_ = 0.0;
  double i_pitch_ = 0.0;
  double t_prev_ = 0.0;
  bool has_t_ = false;
  Vec3 hold_pos_ = Vec3::Zero();  // world position held while there is no command
  bool hold_valid_ = false;
};

}  // namespace go1
