#pragma once
// Owner: Member 5. Stance leg impedance control.
// main.cpp calls compute(): keep its signature (see the README).
//
// Each stance leg acts as a virtual spring-damper between the body and its foot:
//   F_i = K (p_ref - p) + D (v_ref - v) + F_ff        (world-aligned axes, at the foot)
//   + attitude PD: extra vertical force from the roll / pitch errors
//   tau_i = J^T R^T F_i                                (rotate into the body/leg frame)
// F is the force the leg applies to the ground, so the gravity share F_ff points DOWN.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {

struct StanceParams {
  double mass = 13.1;  // total robot mass [kg] (sum of go1.urdf links)
  double gravity = 9.81;

  // Virtual spring-damper per leg (x, y, z), world-aligned axes.
  Vec3 kp = Vec3(300.0, 300.0, 800.0);  // [N/m]
  Vec3 kd = Vec3(30.0, 30.0, 60.0);     // [N s/m]

  // Attitude PD: corrective body torque from roll / pitch error and rate.
  double kp_roll = 50.0;   // [Nm/rad]
  double kd_roll = 3.0;    // [Nm s/rad]
  double kp_pitch = 80.0;  // [Nm/rad]
  double kd_pitch = 5.0;   // [Nm s/rad]

  // Joint-level command during stance: no position stiffness, light damping.
  // The torque from J^T F does the work instead of a stiff joint PD.
  Vec3 joint_kp = Vec3(0.0, 0.0, 0.0);
  Vec3 joint_kd = Vec3(1.0, 1.0, 1.0);

  // Clamp on the stance torque per joint (hip, thigh, calf) [Nm].
  // Below the motor limits in fsm.yaml; Safety clamps again after the blender.
  Vec3 tau_max = Vec3(20.0, 20.0, 30.0);
};

// =====================================================================================
// TODO(Member 5): TEMPORARY leg kinematics. REMOVE when Member 2's kinematics.h lands.
//   Copied from Member 4's swing_kin (branch feature/swing_controller) because
//   kinematics.cpp is still empty. When Member 2 delivers:
//     1. #include "go1_ctrl/kinematics.h"
//     2. replace every stance_kin:: call in stance_controller.cpp with Member 2's version
//     3. delete this namespace here and in stance_controller.cpp
//   Check Member 2's frame convention first (foot relative to the hip or to the body?).
// =====================================================================================
// fk / jacobian give the foot position relative to the hip joint, in body axes.
namespace stance_kin {
Vec3 hipOffset(int leg);    // hip joint position in the body frame
Vec3 nominalFoot(int leg);  // neutral foot (x, y) under the hip, body frame, z = 0
Vec3 fk(int leg, const Vec3& q);
Mat3 jacobian(int leg, const Vec3& q);
}  // namespace stance_kin

class StanceController {
 public:
  // Reads the gains from the ROS parameters ~stance/... if they exist (config/stance.yaml),
  // otherwise keeps the defaults in StanceParams.
  StanceController();
  explicit StanceController(const StanceParams& p) : p_(p) {}

  // Command for every leg as if it were in stance; the blender picks per leg.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);

  const StanceParams& params() const { return p_; }
  // Last foot force per leg (world-aligned axes, force the leg applies to the ground),
  // for the debug topic and plots.
  const Vec3& lastForce(int leg) const { return force_[leg]; }

 private:
  StanceParams p_;
  std::array<Vec3, kNumLegs> force_{{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero()}};
};

}  // namespace go1
