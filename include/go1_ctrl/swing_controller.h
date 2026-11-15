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

  // Touchdown height. With level_footholds the foot lands on the ground estimated from
  // the stance feet in gravity-aligned axes (IMU attitude), so it still reaches the
  // ground when the body rolls or pitches. Otherwise it lands at its liftoff height
  // in the body frame (the old behaviour).
  bool level_footholds = true;
  double touchdown_depth = 0.01;    // aim this far below the ground estimate [m]
  double ground_filter_tau = 0.05;  // low-pass on the ground estimate [s]

  // Late touchdown / bounce: a leg scheduled in stance whose foot force is still below
  // touch_force early in stance (stance_progress < reach_window) keeps reaching down
  // along gravity at reach_speed, at most reach_depth below where it started. The
  // transition blender uses this command while the contact detector reports no contact.
  // Only after some foot has carried load (so not on the hanging rig).
  // reach_window = 0 turns it off.
  double reach_window = 0.25;  // [stance progress]
  double touch_force = 15.0;   // [N], same as ContactDetector::on_force
  double reach_speed = 0.3;    // [m/s]
  double reach_depth = 0.06;   // [m]
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
  // Legs scheduled in contact get a hold-in-place command, or a reach-down command if
  // their foot has not found the ground yet (see reach_window).
  // If IK has no solution inside the joint limits, the last valid q_des is kept.
  void compute(const RobotState& s, const GaitState& g, const BodyCmd& cmd, RobotCommand& out);

  // Last target, measured foot state and foothold of a leg (body frame), for the debug topic.
  const SwingTarget& lastTarget(int leg) const { return target_[leg]; }
  const SwingTarget& lastMeasured(int leg) const { return measured_[leg]; }
  const Vec3& lastFoothold(int leg) const { return foothold_[leg]; }
  long ikFailures() const { return ik_failures_; }  // IK calls that returned false
  // Ground height below the body origin in gravity-aligned axes [m] (negative), from the
  // stance feet. Valid once a stance foot has been seen.
  double groundHeight() const { return ground_z_; }
  bool reaching(int leg) const { return reaching_[leg]; }  // reach-down active this cycle

 private:
  // Joint and torque command that tracks a foot target (IK, damped dq, impedance tau).
  void track(int leg, const Mat3& J, const Vec3& p, const Vec3& v, const SwingTarget& target,
             LegCommand& l);

  SwingParams params_;
  std::array<LegGeometry, kNumLegs> geometry_;
  Kinematics kin_;
  FootPlanner planner_;
  double t_prev_;  // time of the previous compute() call
  long ik_failures_;
  double ground_z_;   // filtered ground height, gravity-aligned, relative to the body origin
  bool has_ground_;

  std::array<bool, kNumLegs> swinging_;
  std::array<double, kNumLegs> last_progress_;
  std::array<double, kNumLegs> last_t_;
  std::array<double, kNumLegs> progress_rate_;  // d(swing_progress)/dt = 1 / T_swing
  std::array<Vec3, kNumLegs> p_liftoff_;
  std::array<Vec3, kNumLegs> q_des_;  // last valid IK solution of each leg
  std::array<SwingTarget, kNumLegs> target_;
  std::array<SwingTarget, kNumLegs> measured_;
  std::array<Vec3, kNumLegs> foothold_;
  std::array<bool, kNumLegs> reaching_;
  std::array<Vec3, kNumLegs> reach_start_;  // body frame
  std::array<bool, kNumLegs> loaded_;  // foot has carried load since its touchdown
};

}  // namespace go1
