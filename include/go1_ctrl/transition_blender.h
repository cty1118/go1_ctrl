#pragma once
// Owner: Member 6. Blending between swing and stance commands at touchdown and lift-off.
// LOCAL TEST VERSION (Member 4): picks the command from the scheduled AND the detected
// contact, and ramps the stance command in over ramp_time.
// main.cpp calls this interface: keep it.
#include <array>
#include "go1_ctrl/types.h"

namespace go1 {
class TransitionBlender {
 public:
  // Per leg: mix swing and stance commands around touchdown and liftoff.
  //  - late touchdown / bounce: scheduled stance, no detected contact, stance_progress
  //    below late_window and for at most late_time -> keep the swing command;
  //  - early touchdown: scheduled swing, detected contact, swing_progress above
  //    early_window -> stance command;
  //  - the stance command is ramped in from the swing command over ramp_time.
  // Timing uses the control period dt, not the gait phase: the phase stops while the
  // Walk scheduler waits for support, and the blender must not wait with it.
  void blend(const GaitState& g, const std::array<bool, kNumLegs>& contact,
             const RobotCommand& swing, const RobotCommand& stance, RobotCommand& out);

  double late_window = 0.25;   // [stance progress]
  double late_time = 0.08;     // [s] longest wait for a late touchdown
  double early_window = 0.75;  // [swing progress]
  double ramp_time = 0.04;     // [s]
  double dt = 0.002;           // control period [s] (500 Hz)

 private:
  std::array<double, kNumLegs> w_{};     // stance weight per leg, 0..1
  std::array<double, kNumLegs> wait_{};  // time in scheduled stance without detected contact
};
}  // namespace go1
