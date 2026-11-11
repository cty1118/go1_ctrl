// Owner: Member 2. Go1 FK, analytical IK, and velocity kinematics from our DH derivation.
#include "go1_ctrl/kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace go1 {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kLengthTol = 1e-9; // Meters: roundoff near lateral reach boundary
constexpr double kCosTol = 1e-10;   // Roundoff for acos domain
constexpr double kAngleTol = 1e-9; // Radians: joint limit comparisons
constexpr double kFootTol = 1e-6;   // Meters: FK verification of each IK candidate

// Choose the 2*pi-equivalent angle inside the joint limits that is closest
// to the measured current angle. Do not use an unrestricted wrap-to-pi:
// motor joint limits define which angle representation is physically valid.
bool angleInLimits(double angle, double minAngle, double maxAngle,
                   double current, double& result) {
  bool found = false;
  double bestDist = std::numeric_limits<double>::infinity();
  for (int k = -2; k <= 2; ++k) {
    const double value = angle + k * kTwoPi;
    if (value < minAngle - kAngleTol || value > maxAngle + kAngleTol)
      continue;

    const double withinLimits = std::max(minAngle, std::min(maxAngle, value));
    const double dist = std::abs(withinLimits - current);
    if (!found || dist < bestDist) {
      found = true;
      bestDist = dist;
      result = withinLimits;
    }
  }
  return found;
}

}  // namespace

Kinematics::Kinematics(const std::array<LegGeometry, kNumLegs>& geometry,
                       const IKOptions& ikOptions)
    : geometry_(geometry), ikOptions_(ikOptions) {
  for (const LegGeometry& g : geometry_) {
    if (!g.hipPosition_B.allFinite() || !std::isfinite(g.lateralOffset) ||
        !std::isfinite(g.thighLength) || !std::isfinite(g.calfLength) ||
        g.thighLength <= 0.0 || g.calfLength <= 0.0) {
      throw std::invalid_argument("Invalid Go1 leg geometry");
    }
  }
  if (!ikOptions_.jointMin.allFinite() || !ikOptions_.jointMax.allFinite())
    throw std::invalid_argument("IK joint limits must be finite");
  for (int j = 0; j < kJointsPerLeg; ++j) {
    const double step = ikOptions_.maxJointStep[j];
    if (ikOptions_.jointMin[j] >= ikOptions_.jointMax[j] ||
        std::isnan(step) || step <= 0.0) {
      throw std::invalid_argument("Invalid IK joint limits or max step");
    }
  }
}

Vec3 Kinematics::forwardKinematics(int leg, const Vec3& q) const {
  if (leg < 0 || leg >= kNumLegs)
    throw std::out_of_range("Invalid Go1 leg index");
  if (!q.allFinite())
    throw std::invalid_argument("Joint angles must be finite");

  const LegGeometry& g = geometry_[leg];
  const double q1 = q[HIP];
  const double q2 = q[THIGH];
  const double q3 = q[CALF];

  const double s1 = std::sin(q1), c1 = std::cos(q1);
  const double s2 = std::sin(q2);
  const double s23 = std::sin(q2 + q3);
  const double U = g.thighLength * std::cos(q2) +
                   g.calfLength * std::cos(q2 + q3);

  // expressed relative to the body frame B.
  const double px = g.hipPosition_B.x() - g.thighLength * s2 - g.calfLength * s23;
  const double py = g.hipPosition_B.y() + s1 * U + g.lateralOffset * c1;
  const double pz = g.hipPosition_B.z() - c1 * U + g.lateralOffset * s1;
  return Vec3(px, py, pz);
}

bool Kinematics::inverseKinematics(int leg, const Vec3& footPosition_B,
                                   const Vec3& qCurrent, Vec3& qSolution) const {
  if (leg < 0 || leg >= kNumLegs)
    throw std::out_of_range("Invalid Go1 leg index");
  if (!footPosition_B.allFinite() || !qCurrent.allFinite())
    return false;

  const LegGeometry& g = geometry_[leg];
  const double a2 = g.thighLength;
  const double a3 = g.calfLength;
  const double d2 = g.lateralOffset;

  // Desired foot position relative to the HAA joint / DH frame 0 origin.
  const Vec3 p = footPosition_B - g.hipPosition_B;
  const double X = p.x(), Y = p.y(), Z = p.z();

  // HAA geometry: Y^2 + Z^2 = U^2 + d2^2,
  // where U = a2*cos(q2) + a3*cos(q2 + q3).
  const double radial = std::hypot(Y, Z);
  if (radial < std::abs(d2) - kLengthTol)
    return false;  // Foot projection is inside the hip offset circle.

  const double Uabs = std::sqrt(std::max(0.0, radial * radial - d2 * d2));

  // Planar two-link IK: D = cos(q3). Independent of the sign of U.
  const double D = (X * X + Uabs * Uabs - a2 * a2 - a3 * a3) /
                   (2.0 * a2 * a3);
  if (D < -1.0 - kCosTol || D > 1.0 + kCosTol)
    return false;

  const double cos3 = std::max(-1.0, std::min(1.0, D));
  const double sin3abs = std::sqrt(std::max(0.0, 1.0 - cos3 * cos3));

  bool found = false;
  double bestCost = std::numeric_limits<double>::infinity();
  Vec3 bestQ = Vec3::Zero();

  // Up to 2 hip branches x 2 knee branches = 4 candidate IK solutions.
  for (int hipBranch = 0; hipBranch < 2; ++hipBranch) {
    if (hipBranch == 1 && Uabs < kLengthTol)
      continue; // The hip branches coincide at U = 0.

    const double U = (hipBranch == 0) ? Uabs : -Uabs;
    // (Y,Z) = R(q1)*(d2,-U)
    const double q1 = std::atan2(Z, Y) - std::atan2(-U, d2);

    for (int kneeBranch = 0; kneeBranch < 2; ++kneeBranch) {
      if (kneeBranch == 1 && sin3abs < kAngleTol)
        continue; // Same knee configuration at q3 = 0 or +/-pi.

      const double sin3 = (kneeBranch == 0) ? sin3abs : -sin3abs;
      const double q3 = std::atan2(sin3, cos3);
      const double q2 = std::atan2(-X, U) -
                        std::atan2(a3 * sin3, a2 + a3 * cos3);

      const double raw[3] = {q1, q2, q3};
      Vec3 candidate;
      bool withinLimits = true;
      for (int j = 0; j < kJointsPerLeg; ++j) {
        double angle = 0.0;
        if (!angleInLimits(raw[j], ikOptions_.jointMin[j],
                           ikOptions_.jointMax[j], qCurrent[j], angle)) {
          withinLimits = false;
          break;
        }
        candidate[j] = angle;
      }
      if (!withinLimits)
        continue;

      // Optional guard: reject large single-update joint changes.
      // The calling controller should retain its previous safe command on false.
      bool stepAllowed = true;
      for (int j = 0; j < kJointsPerLeg; ++j) {
        if (std::abs(candidate[j] - qCurrent[j]) > ikOptions_.maxJointStep[j]) {
          stepAllowed = false;
          break;
        }
      }
      if (!stepAllowed)
        continue;

      // Check that the IK solution really reproduces the requested target.
      // This protects against floating-point rounding near workspace edges.
      if ((forwardKinematics(leg, candidate) - footPosition_B).norm() > kFootTol)
        continue;

      // Select the solution requiring the least joint displacement.
      const double cost = (candidate - qCurrent).squaredNorm();
      if (!found || cost < bestCost) {
        found = true;
        bestCost = cost;
        bestQ = candidate;
      }
    }
  }

  if (found)
    qSolution = bestQ;  // Do not modify output on failure.
  return found;
}

// Jv_B(q) = partial pFoot_B(q) / partial q.
// This 3x3 Jacobian relates joint velocity to translational foot velocity
// RELATIVE TO THE BODY, expressed in body coordinates B.
Mat3 Kinematics::jacobian(int leg, const Vec3& q) const {
  if (leg < 0 || leg >= kNumLegs)
    throw std::out_of_range("Invalid Go1 leg index");
  if (!q.allFinite())
    throw std::invalid_argument("Joint angles must be finite");

  const LegGeometry& g = geometry_[leg];
  const double q1 = q[HIP];
  const double q2 = q[THIGH];
  const double q3 = q[CALF];
  const double s1 = std::sin(q1), c1 = std::cos(q1);
  const double s2 = std::sin(q2), s23 = std::sin(q2 + q3);
  const double c2 = std::cos(q2), c23 = std::cos(q2 + q3);

  const double a2 = g.thighLength;
  const double a3 = g.calfLength;
  const double d2 = g.lateralOffset;
  const double U = a2 * c2 + a3 * c23;
  const double V = a2 * s2 + a3 * s23;

  Mat3 J;
  // Columns: HAA, HFE, KFE. Rows: vx, vy, vz in B.
  J << 0.0,            -U,        -a3 * c23,
       c1*U - d2*s1, -s1*V,       -a3 * s1 * s23,
       s1*U + d2*c1,  c1*V,        a3 * c1 * s23;
  return J;
}

Mat63 Kinematics::geometricJacobian(int leg, const Vec3& q) const {
  // jacobian() validates leg and q as well.
  Mat63 J = Mat63::Zero();
  J.topRows<3>() = jacobian(leg, q);

  // From T_B0 and the three DH rotations:
  // z0_B = [1,0,0]^T, z1_B = z2_B = [0,cos(q1),sin(q1)]^T.
  const double s1 = std::sin(q[HIP]), c1 = std::cos(q[HIP]);
  J(3, 0) = 1.0;
  J(4, 1) = c1;
  J(4, 2) = c1;
  J(5, 1) = s1;
  J(5, 2) = s1;
  return J;
}

Vec3 Kinematics::footVelocity(int leg, const Vec3& q,
                               const Vec3& dq) const {
  // Perform leg/q validation through jacobian(), even if dq is invalid.
  const Mat3 J = jacobian(leg, q);
  if (!dq.allFinite())
    throw std::invalid_argument("Joint velocities must be finite");
  return J * dq;
}

bool Kinematics::inverseVelocityKinematics(int leg, const Vec3& q,
                                            const Vec3& desiredFootVelocity_B,
                                            double damping,
                                            const Vec3& maxAbsJointSpeed,
                                            Vec3& dqSolution) const {
  // Also validates leg index and joint angle finiteness.
  const Mat3 J = jacobian(leg, q);
  if (!desiredFootVelocity_B.allFinite() || !maxAbsJointSpeed.allFinite() ||
      !std::isfinite(damping) || damping <= 0.0 ||
      (maxAbsJointSpeed.array() <= 0.0).any()) {
    return false;
  }

  // Damped least squares is regularized, and remains finite at singularities.
  // Units: J [m/rad], lambda [m/rad], desiredFootVelocity_B [m/s].
  const Mat3 regularized = J * J.transpose() +
                           (damping * damping) * Mat3::Identity();
  const Vec3 dqCandidate =
      J.transpose() * regularized.ldlt().solve(desiredFootVelocity_B);
  if (!dqCandidate.allFinite() ||
      (dqCandidate.array().abs() > maxAbsJointSpeed.array()).any()) {
    return false;
  }

  dqSolution = dqCandidate;
  return true;
}

}  // namespace go1
