#pragma once
// Owner: Member 4. Swing leg trajectory, position tracking and impedance feedforward.
// main.cpp calls compute(): keep its signature (see the README).
#include <array>
#include "go1_ctrl/foot_planner.h"
#include "go1_ctrl/kinematics.h"
#include "go1_ctrl/types.h"

namespace go1 {

struct SwingParams {
  double step_height = 0.08;  // apex height above the line from liftoff to foothold [m]

  // Joint PD sent to the motors (high gains for position tracking).
  // Starting values for simulation; tune in the hanging test.
  Vec3 joint_kp = Vec3(60.0, 60.0, 60.0);
  Vec3 joint_kd = Vec3(3.0, 3.0, 3.0);

  // Cartesian impedance used for the feedforward torque J^T * F.
  Vec3 cart_kp = Vec3(300.0, 300.0, 300.0);
  Vec3 cart_kd = Vec3(10.0, 10.0, 10.0);

  double dls_lambda = 0.01;    // damping for J^-1 near singular (straight leg) poses
  double tau_ff_limit = 10.0;  // clamp on each feedforward joint torque [Nm]

  // Swing duration as a fraction of the period, used only until the swing
  // progress rate has been measured (GaitState carries no duty factor).
  double default_swing_fraction = 0.5;
};

// Desired foot position and velocity at one instant of the swing.
struct SwingTarget {
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
};

// Cycloid swing from p0 to pf. s is the swing progress in [0, 1] and t_swing
// the swing duration, so v = dp/ds / t_swing. Velocity is zero at both ends.
// pf_rate is the foothold velocity while it is still being updated, so that v
// stays the time derivative of p when pf moves.
SwingTarget cycloidSwing(const Vec3& p0, const Vec3& pf, double height, double s,
                         double t_swing, const Vec3& pf_rate = Vec3::Zero());

// Neutral foot position under the hip, body frame (z = 0): the hip joint
// position plus the hip-to-thigh offset along y.
Vec3 nominalFoot(const LegGeometry& g);

// Leg kinematics (FK, IK, Jacobian) come from Member 2's Kinematics class.
class SwingController {
 public:
  // Uses defaultGo1Geometry() (Member 2, estimator.h).
  explicit SwingController(const SwingParams& params = SwingParams());
  SwingController(const SwingParams& params, const std::array<LegGeometry, kNumLegs>& geometry);

  void setParams(const SwingParams& params) { params_ = params; }
  const SwingParams& params() const { return params_; }
  FootPlanner& planner() { return planner_; }
  const Kinematics& kinematics() const { return kin_; }

  // Command for every leg as if it were swinging; the blender picks per leg.
  //   q_des  = IK(p_des),  dq_des = J^-1 v_des   (tracked with joint Kp, Kd)
  //   tau_ff = J^T [Kp (p_des - p) + Kd (v_des - v)]
  // Legs scheduled in contact get a hold-in-place command.
  // If IK has no solution inside the joint limits, the last valid q_des is kept.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);

  // Last target, measured foot state and foothold of a leg (body frame), for the debug topic.
  const SwingTarget& lastTarget(int leg) const { return target_[leg]; }
  const SwingTarget& lastMeasured(int leg) const { return measured_[leg]; }
  const Vec3& lastFoothold(int leg) const { return foothold_[leg]; }
  long ikFailures() const { return ik_failures_; }  // IK calls that returned false

 private:
  SwingParams params_;
  std::array<LegGeometry, kNumLegs> geometry_;
  Kinematics kin_;
  FootPlanner planner_;
  double t_prev_;  // time of the previous compute() call
  long ik_failures_;

  std::array<bool, kNumLegs> swinging_;
  std::array<double, kNumLegs> last_progress_;
  std::array<double, kNumLegs> last_t_;
  std::array<double, kNumLegs> progress_rate_;  // d(swing_progress)/dt = 1 / T_swing
  std::array<Vec3, kNumLegs> p_liftoff_;
  std::array<Vec3, kNumLegs> q_des_;  // last valid IK solution of each leg
  std::array<SwingTarget, kNumLegs> target_;
  std::array<SwingTarget, kNumLegs> measured_;
  std::array<Vec3, kNumLegs> foothold_;
};

}  // namespace go1
