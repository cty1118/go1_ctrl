#pragma once
// Owner: Member 2. Body state estimation (attitude, velocity, height).
// Frames: B = body (x forward, y left, z up), W = world (z up).
// Uses IMU orientation/gyro/acceleration and FK/Jacobians of stance feet.
#include <array>
#include "go1_ctrl/types.h"
#include "go1_ctrl/kinematics.h"

namespace go1 {

struct EstimatorParams {
  double gravity = 9.81;                // [m/s^2], world gravity is [0,0,-gravity]
  double maxDt = 0.05;                  // [s], reset after clock jumps/long gaps
  double minContactTime = 0.008;        // [s], ignore brief contact signals
  double positionTimeConstant = 0.06;   // [s], stance position correction
  double velocityTimeConstant = 0.025;  // [s], stance velocity correction
  double slipThreshold = 0.12;         // [m], re-anchor inconsistent stance foot
  double maxImuAcceleration = 50.0;    // [m/s^2], reject implausible IMU samples
  double maxContactSpeed = 5.0;        // [m/s], reject implausible stance kinematics
  bool useGroundTruth = false;         // if true, pass through valid Gazebo pos/vel
};

std::array<LegGeometry, kNumLegs> defaultGo1Geometry();

class Estimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Estimator();
  explicit Estimator(const std::array<LegGeometry, kNumLegs>& geometry,
                     const EstimatorParams& params = EstimatorParams());

  // Writes s.pos and s.vel in WORLD coordinates [m, m/s]. Uses s.quat as
  // body-to-world orientation, normalized if valid; gyro, acc in body(B) frame.
  // `contact` comes from Member 6's actual force-based contact detector.
  // On non-ground-truth output sets s.has_ground_truth=false.
  void update(RobotState& s, const std::array<bool, kNumLegs>& contact);

  void reset();
  bool initialized() const { return initialized_; }
  int supportingFeet() const { return supportingFeet_; }

 private:
  Kinematics kin_;
  EstimatorParams params_;
  Vec3 position_W_ = Vec3::Zero();
  Vec3 velocity_W_ = Vec3::Zero();
  Eigen::Quaterniond lastQuat_ = Eigen::Quaterniond::Identity();
  std::array<Vec3, kNumLegs> footAnchor_W_;
  std::array<bool, kNumLegs> anchorValid_{};
  std::array<double, kNumLegs> contactAge_{};
  double lastTime_ = 0.0;
  bool haveTime_ = false;
  bool haveQuat_ = false;
  bool initialized_ = false;
  int supportingFeet_ = 0;
};

}  // namespace go1
