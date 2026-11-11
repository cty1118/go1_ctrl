#pragma once
// Owner: Member 2. Go1 leg kinematics.
// Body frame B: x forward, y left, z up. Angles are in radians.
// Joint order and leg indices come from types.h.

#include <array>
#include <limits>
#include "go1_ctrl/types.h"

namespace go1 {

struct LegGeometry {
  Vec3 hipPosition_B = Vec3::Zero(); // Position of the HAA/DH frame 0 origin in B
  double lateralOffset = 0.0;      // Signed d2: +left, -right [m]
  double thighLength = 0.0;        // a2 [m]
  double calfLength = 0.0;         // a3 [m]
};

struct IKOptions {
  // Keep these synchronized with the SafetyParams / config/fsm.yaml limits.
  Vec3 jointMin = Vec3(-1.047, -0.663, -2.721);
  Vec3 jointMax = Vec3( 1.047,  2.966, -0.837);

  // Optional maximum deviation from qCurrent for ONE IK call, in rad.
  // Infinity disables the guard; the gait controller should also ensure
  // smooth Cartesian targets and use velocity/acceleration limits.
  Vec3 maxJointStep = Vec3::Constant(std::numeric_limits<double>::infinity());
};

// Full geometric Jacobian: 3 linear rows, followed by 3 angular rows.
using Mat63 = Eigen::Matrix<double, 6, 3>;

class Kinematics {
 public:
  // Geometry must be in FR, FL, RR, RL order.
  explicit Kinematics(const std::array<LegGeometry, kNumLegs>& geometry,
                      const IKOptions& ikOptions = IKOptions());

  // Returns the foot-center position in body frame B [m].
  Vec3 forwardKinematics(int leg, const Vec3& q) const;

  // Analytic 3-DOF position IK. Returns true only for a reachable target
  // with a solution inside joint limits (and the optional step guard).
  // Among all valid solutions, select the one closest to qCurrent.
  // On false, qSolution is left unchanged. Invalid leg index throws.
  bool inverseKinematics(int leg, const Vec3& footPosition_B,
                         const Vec3& qCurrent, Vec3& qSolution) const;

  // Linear foot-velocity Jacobian, expressed in the BODY frame B [m/rad].
  // vFootRelative_B = jacobian(leg, q) * dq, with the body held fixed.
  // Rows correspond to foot velocity (vx,vy,vz), columns to HAA,HFE,KFE.
  Mat3 jacobian(int leg, const Vec3& q) const;

  // Full 6x3 geometric Jacobian [Jv; Jw] expressed in body frame B.
  // Rows are [vx,vy,vz,wx,wy,wz]. Jw maps joint rates to foot angular
  // velocity relative to the body (not Euler-angle derivatives).
  Mat63 geometricJacobian(int leg, const Vec3& q) const;

  // Foot linear velocity relative to body B [m/s], for dq [rad/s].
  // For world velocity, body linear and angular motion must also be added.
  Vec3 footVelocity(int leg, const Vec3& q, const Vec3& dq) const;

  // Optional damped inverse differential kinematics for desired foot velocity.
  // Uses J^T (J J^T + damping^2 I)^-1 v to avoid unbounded joint rates
  // near a singularity. Caller must provide positive damping [m/rad] and
  // per-joint maximum absolute rates [rad/s]. Return false if rate limits
  // would be exceeded; on false, leave dqSolution unchanged.
  // This is not a trajectory generator and does not enforce acceleration limits.
  bool inverseVelocityKinematics(int leg, const Vec3& q,
                                 const Vec3& desiredFootVelocity_B,
                                 double damping, const Vec3& maxAbsJointSpeed,
                                 Vec3& dqSolution) const;

 private:
  std::array<LegGeometry, kNumLegs> geometry_;
  IKOptions ikOptions_;
};

}  // namespace go1
