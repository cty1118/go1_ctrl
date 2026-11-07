#include "go1_ctrl/fsm.h"

#include <algorithm>
#include <cmath>
#include <ros/console.h>

namespace go1 {

const char* modeName(Mode m) {
  switch (m) {
    case Mode::PASSIVE: return "PASSIVE";
    case Mode::STAND: return "STAND";
    case Mode::GAIT: return "GAIT";
  }
  return "?";
}

void FSM::request(int cmd, const RobotState& s) {
  switch (cmd) {
    case CMD_PASSIVE:
      if (mode_ != Mode::PASSIVE) {
        mode_ = Mode::PASSIVE;
        ROS_INFO("[fsm] -> PASSIVE");
      }
      break;
    case CMD_STAND:
      if (mode_ != Mode::STAND) enterStand(s);
      break;
    case CMD_STOP:
      if (mode_ == Mode::GAIT) enterStand(s);
      break;
    case CMD_WALK:
    case CMD_TROT:
    case CMD_CANTER:
    case CMD_GALLOP: {
      const GaitType g = static_cast<GaitType>(cmd - CMD_WALK);
      if (mode_ == Mode::GAIT) {
        if (g != gait_) {
          gait_ = g;
          gait_changed_ = true;
          ROS_INFO("[fsm] switch gait -> %s", gaitName(g));
        }
      } else if (mode_ == Mode::STAND && standFinished(s.t)) {
        mode_ = Mode::GAIT;
        gait_ = g;
        gait_changed_ = true;
        ROS_INFO("[fsm] -> GAIT (%s)", gaitName(g));
      } else {
        ROS_WARN("[fsm] stand up first (key 2) and wait %.1f s before starting a gait",
                 p_.stand_time);
      }
      break;
    }
    default:
      ROS_WARN("[fsm] unknown mode command %d", cmd);
  }
}

void FSM::forcePassive(const std::string& reason) {
  if (mode_ == Mode::PASSIVE) return;
  mode_ = Mode::PASSIVE;
  ROS_ERROR("[fsm] -> PASSIVE: %s", reason.c_str());
}

bool FSM::takeGaitChanged() {
  const bool changed = gait_changed_;
  gait_changed_ = false;
  return changed;
}

void FSM::enterStand(const RobotState& s) {
  q_start_ = s.q;
  t_start_ = s.t;
  mode_ = Mode::STAND;
  ROS_INFO("[fsm] -> STAND (%.1f s interpolation)", p_.stand_time);
}

bool FSM::standFinished(double t) const { return t - t_start_ >= p_.stand_time; }

void FSM::passiveCommand(RobotCommand& cmd) const {
  for (LegCommand& l : cmd.leg) {
    l.q.setZero();
    l.dq.setZero();
    l.kp.setZero();
    l.kd.setConstant(p_.passive_kd);
    l.tau.setZero();
  }
}

void FSM::standCommand(const RobotState& s, RobotCommand& cmd) const {
  // Smoothstep from the pose at entry to the standing pose: zero velocity at both ends.
  const double T = std::max(p_.stand_time, 1e-3);
  const double a = std::min(std::max((s.t - t_start_) / T, 0.0), 1.0);
  const double w = a * a * (3.0 - 2.0 * a);
  const double dw = 6.0 * a * (1.0 - a) / T;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 q0 = q_start_.segment<3>(kJointsPerLeg * leg);
    LegCommand& l = cmd.leg[leg];
    l.q = q0 + w * (p_.stand_q - q0);
    l.dq = dw * (p_.stand_q - q0);
    l.kp.setConstant(p_.stand_kp);
    l.kd.setConstant(p_.stand_kd);
    l.tau.setZero();
  }
}

std::string Safety::checkState(const RobotState& s, Mode m) const {
  if (!isFinite(s)) return "non-finite robot state";
  if (m == Mode::PASSIVE) return "";

  // z-axis of the body expressed in world: R(2,2) = cos(roll) * cos(pitch).
  const double up = s.quat.normalized().toRotationMatrix()(2, 2);
  if (up < std::cos(p_.fall_angle)) {
    return "fall detected (tilt " + std::to_string(std::acos(std::max(-1.0, std::min(1.0, up))) *
                                                   180.0 / M_PI) +
           " deg)";
  }
  if (m == Mode::GAIT && p_.check_height && s.pos.z() < p_.min_height) {
    return "fall detected (body height " + std::to_string(s.pos.z()) + " m)";
  }
  return "";
}

void Safety::limit(const RobotState& s, RobotCommand& cmd) const {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    LegCommand& l = cmd.leg[leg];
    for (int j = 0; j < kJointsPerLeg; ++j) {
      const int i = jointIndex(leg, j);
      l.q[j] = std::min(std::max(l.q[j], p_.q_min[j]), p_.q_max[j]);
      const double lim = p_.tau_max[j];
      l.tau[j] = std::min(std::max(l.tau[j], -lim), lim);
      // The motor applies tau + Kp(q_des - q) + Kd(dq_des - dq); keep that total within the limit.
      const double pd = l.kp[j] * (l.q[j] - s.q[i]) + l.kd[j] * (l.dq[j] - s.dq[i]);
      const double total = pd + l.tau[j];
      const double clamped = std::min(std::max(total, -lim), lim);
      l.tau[j] += clamped - total;
    }
  }
}

}  // namespace go1
