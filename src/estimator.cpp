// Owner: Member 2. State estimator.
#include "go1_ctrl/estimator.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go1 {

std::array<LegGeometry, kNumLegs> defaultGo1Geometry() {
  std::array<LegGeometry, kNumLegs> g;
  const double hx = 0.1881, hy = 0.04675;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const bool front = (leg == FR || leg == FL);
    const bool left = (leg == FL || leg == RL);
    g[leg].hipPosition_B = Vec3(front ? hx : -hx, left ? hy : -hy, 0.0);
    g[leg].lateralOffset = left ? 0.080 : -0.080;
    g[leg].thighLength = 0.213;
    g[leg].calfLength = 0.213;
  }
  return g;
}

Estimator::Estimator() : Estimator(defaultGo1Geometry()) {}

Estimator::Estimator(const std::array<LegGeometry, kNumLegs>& geometry,
                     const EstimatorParams& params)
    : kin_(geometry), params_(params) {
  if (!std::isfinite(params_.gravity) || params_.gravity <= 0.0 ||
      !std::isfinite(params_.maxDt) || params_.maxDt <= 0.0 ||
      !std::isfinite(params_.minContactTime) || params_.minContactTime < 0.0 ||
      !std::isfinite(params_.positionTimeConstant) || params_.positionTimeConstant <= 0.0 ||
      !std::isfinite(params_.velocityTimeConstant) || params_.velocityTimeConstant <= 0.0 ||
      !std::isfinite(params_.slipThreshold) || params_.slipThreshold <= 0.0 ||
      !std::isfinite(params_.maxImuAcceleration) || params_.maxImuAcceleration <= 0.0 ||
      !std::isfinite(params_.maxContactSpeed) || params_.maxContactSpeed <= 0.0)
    throw std::invalid_argument("Invalid estimator parameters");
  reset();
}

void Estimator::reset() {
  position_W_.setZero();
  velocity_W_.setZero();
  lastQuat_.setIdentity();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    footAnchor_W_[leg].setZero();
    anchorValid_[leg] = false;
    contactAge_[leg] = 0.0;
  }
  lastTime_ = 0.0;
  haveTime_ = false;
  haveQuat_ = false;
  initialized_ = false;
  supportingFeet_ = 0;
}

void Estimator::update(RobotState& s, const std::array<bool, kNumLegs>& contact) {
  // Reject bad IMU orientation. A zero quaternion is not a valid attitude.
  const double quatNorm = s.quat.norm();
  if (s.quat.coeffs().allFinite() && std::isfinite(quatNorm) && quatNorm > 1e-6) {
    s.quat.normalize();
    lastQuat_ = s.quat;
    haveQuat_ = true;
  } else if (haveQuat_) {
    s.quat = lastQuat_;
  } else {
    s.pos = position_W_;
    s.vel = velocity_W_;
    s.has_ground_truth = false;
    return;
  }
  const Mat3 R_WB = s.quat.toRotationMatrix();

  // Optional debug comparison: retain Gazebo's actual world pose and velocity.
  if (params_.useGroundTruth && s.has_ground_truth &&
      s.pos.allFinite() && s.vel.allFinite()) {
    position_W_ = s.pos;
    velocity_W_ = s.vel;
    initialized_ = true;
    return;
  }

  if (!std::isfinite(s.t) || !s.q.allFinite() || !s.dq.allFinite() ||
      !s.gyro.allFinite() || !s.acc.allFinite()) {
    s.pos = position_W_;
    s.vel = velocity_W_;
    s.has_ground_truth = false;
    return;
  }

  double dt = 0.0;
  if (haveTime_) {
    dt = s.t - lastTime_;
    if (dt < 0.0 || dt > params_.maxDt) {
      // E.g., Gazebo reset, paused/restarted node or GAIT resumed after STAND.
      reset();
      lastQuat_ = s.quat;
      haveQuat_ = true;
      dt = 0.0;
    }
  }
  lastTime_ = s.t;
  haveTime_ = true;

  // Filter contact flags independently of Member 6's contact detector.
  std::array<bool, kNumLegs> stance{};
  std::array<Vec3, kNumLegs> foot_B;
  std::array<Vec3, kNumLegs> footVelocity_B;
  int nStance = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (!contact[leg]) {
      contactAge_[leg] = 0.0;
      anchorValid_[leg] = false;
      continue;
    }
    contactAge_[leg] += dt;
    if (contactAge_[leg] + 1e-12 < params_.minContactTime)
      continue;

    const Vec3 q = s.legQ(leg);
    const Vec3 dq = s.legDq(leg);
    foot_B[leg] = kin_.forwardKinematics(leg, q);
    footVelocity_B[leg] = kin_.footVelocity(leg, q, dq);
    if (!foot_B[leg].allFinite() || !footVelocity_B[leg].allFinite())
      continue;
    // A stance foot should not be moving extremely fast relative to the body.
    if ((footVelocity_B[leg] + s.gyro.cross(foot_B[leg])).norm() >
        params_.maxContactSpeed)
      continue;
    stance[leg] = true;
    ++nStance;
  }
  supportingFeet_ = nStance;

  if (!initialized_) {
    if (nStance == 0) {
      // No absolute height information until at least one supported foot.
      s.pos = position_W_;
      s.vel = velocity_W_;
      s.has_ground_truth = false;
      return;
    }
    // Initial assumption: supported feet are on world z=0 (flat ground).
    // World x/y origin is arbitrary, and cannot be recovered from proprioception.
    double height = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg)
      if (stance[leg]) height -= (R_WB * foot_B[leg]).z();
    position_W_ = Vec3(0.0, 0.0, height / nStance);
    velocity_W_.setZero();
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (stance[leg]) {
        footAnchor_W_[leg] = position_W_ + R_WB * foot_B[leg];
        anchorValid_[leg] = true;
      }
    }
    initialized_ = true;
    s.pos = position_W_;
    s.vel = velocity_W_;
    s.has_ground_truth = false;
    return;
  }

  // IMU specific force is measured in B and includes +g at rest.
  // Rotate it into W and add world gravity. Bias will cause drift without contact.
  Vec3 a_W = R_WB * s.acc + Vec3(0.0, 0.0, -params_.gravity);
  if (!a_W.allFinite() || a_W.norm() > params_.maxImuAcceleration)
    a_W.setZero();
  const Vec3 predictedPos = position_W_ + velocity_W_ * dt + 0.5 * a_W * dt * dt;
  const Vec3 predictedVel = velocity_W_ + a_W * dt;

  Vec3 measuredPos = Vec3::Zero();
  Vec3 measuredVel = Vec3::Zero();
  int nMeasurements = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (!stance[leg]) continue;
    const Vec3 rotatedFoot = R_WB * foot_B[leg];
    if (!anchorValid_[leg]) {
      // Newly landed foot: it becomes a fixed world point at touchdown.
      footAnchor_W_[leg] = predictedPos + rotatedFoot;
      anchorValid_[leg] = true;
    }
    const Vec3 footDerivedPos = footAnchor_W_[leg] - rotatedFoot;
    if ((footDerivedPos - predictedPos).norm() > params_.slipThreshold) {
      // Contact probably slipped, or contact detector made a false positive.
      // Do not let a stale world anchor create a sudden body-position jump.
      footAnchor_W_[leg] = predictedPos + rotatedFoot;
    }
    measuredPos += footAnchor_W_[leg] - rotatedFoot;

    // Stance foot is stationary in W:
    // 0 = vBody_W + R_WB * (gyro_B x foot_B + Jv_B * dq).
    measuredVel -= R_WB * (s.gyro.cross(foot_B[leg]) + footVelocity_B[leg]);
    ++nMeasurements;
  }

  if (nMeasurements > 0) {
    measuredPos /= nMeasurements;
    measuredVel /= nMeasurements;
    // Time-step independent complementary filter gains [0,1].
    const double ap = dt / (params_.positionTimeConstant + dt);
    const double av = dt / (params_.velocityTimeConstant + dt);
    position_W_ = (1.0 - ap) * predictedPos + ap * measuredPos;
    velocity_W_ = (1.0 - av) * predictedVel + av * measuredVel;
  } else {
    // Flight/no reliable contact: IMU-only dead reckoning (drifts with bias).
    position_W_ = predictedPos;
    velocity_W_ = predictedVel;
  }

  s.pos = position_W_;
  s.vel = velocity_W_;
  s.has_ground_truth = false;
}

}  // namespace go1
