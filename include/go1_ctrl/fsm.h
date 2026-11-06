#pragma once
// Member 1: state machine (Passive -> Stand -> Gait) and safety checks.

#include <string>
#include "go1_ctrl/types.h"

namespace go1 {

enum class Mode { PASSIVE, STAND, GAIT };
const char* modeName(Mode m);

// Values on the go1_ctrl/mode_cmd topic (std_msgs/Int8). Keyboard: space, 1, 2, 3..6.
enum ModeCmd {
  CMD_STOP = 0,
  CMD_PASSIVE = 1,
  CMD_STAND = 2,
  CMD_WALK = 3,
  CMD_TROT = 4,
  CMD_CANTER = 5,
  CMD_GALLOP = 6,
};

struct FsmParams {
  Vec3 stand_q = Vec3(0.0, 0.67, -1.3);  // standing pose per leg (hip, thigh, calf)
  double stand_time = 2.0;               // interpolation time to the standing pose [s]
  double stand_kp = 180.0;
  double stand_kd = 8.0;
  double passive_kd = 8.0;
};

class FSM {
 public:
  explicit FSM(const FsmParams& p) : p_(p) { q_start_.setZero(); }

  void request(int cmd, const RobotState& s);
  void forcePassive(const std::string& reason);

  Mode mode() const { return mode_; }
  GaitType gait() const { return gait_; }
  // True once after a new gait was selected, so the scheduler can be (re)started.
  bool takeGaitChanged();

  void passiveCommand(RobotCommand& cmd) const;
  void standCommand(const RobotState& s, RobotCommand& cmd) const;

 private:
  void enterStand(const RobotState& s);
  bool standFinished(double t) const;

  FsmParams p_;
  Mode mode_ = Mode::PASSIVE;
  GaitType gait_ = GaitType::TROT;
  bool gait_changed_ = false;
  Vec12 q_start_;
  double t_start_ = 0.0;
};

struct SafetyParams {
  Vec3 q_min = Vec3(-1.047, -0.663, -2.721);  // Go1 joint limits (go1_description / SDK)
  Vec3 q_max = Vec3(1.047, 2.966, -0.837);
  Vec3 tau_max = Vec3(23.7, 23.7, 35.55);
  double fall_angle = 1.047;  // body tilt that counts as a fall [rad]
  double min_height = 0.12;   // body height that counts as a fall [m]
  bool check_height = false;  // only meaningful with ground truth on the ground
};

class Safety {
 public:
  explicit Safety(const SafetyParams& p) : p_(p) {}
  // Empty string if the state is fine, otherwise the fault reason.
  std::string checkState(const RobotState& s, Mode m) const;
  // Clamp position targets to joint limits and the predicted joint torque to the torque limits.
  void limit(const RobotState& s, RobotCommand& cmd) const;

 private:
  SafetyParams p_;
};

}  // namespace go1
