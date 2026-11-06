// OWNER: Member 3. PLACEHOLDER: every gait runs as a slow trot. Replace with the real scheduler.
#include "go1_ctrl/gait_scheduler.h"

#include <cmath>

namespace go1 {
void GaitScheduler::start(GaitType g, double t) {
  gait_ = g;
  t0_ = t;
}

GaitState GaitScheduler::update(double t) {
  const double T = 0.5, duty = 0.5;
  const double offset[kNumLegs] = {0.0, 0.5, 0.5, 0.0};  // FR+RL, then FL+RR
  GaitState g;
  g.period = T;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    double ph = std::fmod((t - t0_) / T + offset[leg], 1.0);
    if (ph < 0) ph += 1.0;
    LegGait& l = g.leg[leg];
    l.phase = ph;
    l.contact = ph < duty;
    l.stance_progress = l.contact ? ph / duty : 0.0;
    l.swing_progress = l.contact ? 0.0 : (ph - duty) / (1.0 - duty);
  }
  return g;
}
}  // namespace go1
