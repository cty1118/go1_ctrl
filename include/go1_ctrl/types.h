#pragma once
// Shared data structures (Members 1 + 2).
// Every module uses these structs (see the team rules in the README before changing them).

#include <array>
#include <Eigen/Dense>

namespace go1 {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Mat3 = Eigen::Matrix3d;

constexpr int kNumLegs = 4;
constexpr int kJointsPerLeg = 3;
constexpr int kNumJoints = 12;

// Leg order FR, FL, RR, RL; joint order hip, thigh, calf (same as the Unitree SDK).
enum Leg { FR = 0, FL = 1, RR = 2, RL = 3 };
enum JointId { HIP = 0, THIGH = 1, CALF = 2 };
inline int jointIndex(int leg, int joint) { return kJointsPerLeg * leg + joint; }

enum class GaitType { WALK = 0, TROT = 1, CANTER = 2, GALLOP = 3 };

inline const char* gaitName(GaitType g) {
  switch (g) {
    case GaitType::WALK: return "walk";
    case GaitType::TROT: return "trot";
    case GaitType::CANTER: return "canter";
    case GaitType::GALLOP: return "gallop";
  }
  return "?";
}

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t = 0.0;                    // controller time [s], sim time in Gazebo
  Vec12 q = Vec12::Zero();           // joint position [rad]
  Vec12 dq = Vec12::Zero();          // joint velocity [rad/s]
  Vec12 tau = Vec12::Zero();         // estimated joint torque [Nm]
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();  // body orientation in world (IMU)
  Vec3 gyro = Vec3::Zero();          // body angular velocity, body frame [rad/s]
  Vec3 acc = Vec3::Zero();           // body linear acceleration, body frame [m/s^2]
  Vec4 foot_force = Vec4::Zero();    // foot contact force [N]
  Vec3 pos = Vec3::Zero();           // body position, world frame [m]
  Vec3 vel = Vec3::Zero();           // body velocity, world frame [m/s]
  bool has_ground_truth = false;     // pos/vel came from Gazebo, not the estimator

  Vec3 legQ(int leg) const { return q.segment<3>(kJointsPerLeg * leg); }
  Vec3 legDq(int leg) const { return dq.segment<3>(kJointsPerLeg * leg); }
};

struct LegGait {
  double phase = 0.0;            // gait phase of this leg, [0, 1)
  bool contact = true;           // scheduled to be on the ground
  double swing_progress = 0.0;   // [0, 1] while swinging
  double stance_progress = 0.0;  // [0, 1] while in stance
};

struct GaitState {
  std::array<LegGait, kNumLegs> leg;
  double period = 0.0;               // gait period T [s]
  Vec3 com_shift = Vec3::Zero();     // body CoM reference offset, body frame (Walk, Member 3 -> 5)
};

struct LegCommand {
  Vec3 q = Vec3::Zero();
  Vec3 dq = Vec3::Zero();
  Vec3 kp = Vec3::Zero();
  Vec3 kd = Vec3::Zero();
  Vec3 tau = Vec3::Zero();
};

struct RobotCommand {
  std::array<LegCommand, kNumLegs> leg;
};

struct BodyCmd {
  Vec3 vel = Vec3::Zero();  // desired body velocity, body frame (x forward, y left)
  double yaw_rate = 0.0;    // [rad/s]
  double height = 0.30;     // desired body height [m]
};

inline bool isFinite(const RobotCommand& c) {
  for (const LegCommand& l : c.leg) {
    if (!l.q.allFinite() || !l.dq.allFinite() || !l.kp.allFinite() || !l.kd.allFinite() ||
        !l.tau.allFinite())
      return false;
  }
  return true;
}

inline bool isFinite(const RobotState& s) {
  return s.q.allFinite() && s.dq.allFinite() && s.quat.coeffs().allFinite() &&
         s.gyro.allFinite() && s.acc.allFinite();
}

}  // namespace go1
