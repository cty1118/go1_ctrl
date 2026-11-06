#pragma once
// OWNER: Member 3 (Luciana C.). PLACEHOLDER written by Member 1 so the loop runs.
// Proposed interface, confirm at the freeze meeting. Replace the body in src/gait_scheduler.cpp.
#include "go1_ctrl/types.h"

namespace go1 {
class GaitScheduler {
 public:
  // Restart the schedule for gait g at time t (called on every gait change).
  void start(GaitType g, double t);
  // Phase, contact plan and swing/stance progress for every leg at time t.
  GaitState update(double t);

 private:
  GaitType gait_ = GaitType::TROT;
  double t0_ = 0.0;
};
}  // namespace go1
