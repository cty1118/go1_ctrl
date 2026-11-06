// OWNER: Member 6. PLACEHOLDER: hard switch on the scheduled contact. Replace with the ramped blend.
#include "go1_ctrl/transition_blender.h"

namespace go1 {
void TransitionBlender::blend(const GaitState& g, const std::array<bool, kNumLegs>& /*contact*/,
                              const RobotCommand& swing, const RobotCommand& stance,
                              RobotCommand& out) {
  for (int leg = 0; leg < kNumLegs; ++leg)
    out.leg[leg] = g.leg[leg].contact ? stance.leg[leg] : swing.leg[leg];
}
}  // namespace go1
