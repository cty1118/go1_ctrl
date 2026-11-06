// Member 1: RealIO. Talks to the physical Go1 through unitree_legged_sdk (v3.8.x, low level).
// Communication only: the SDK is used to send LowCmd and receive LowState, nothing else.
//
// Network: the control PC must be at 192.168.123.162 (Member 6 sets this up), the Go1 low-level
// controller answers at 192.168.123.10:8007.
//
// Every command passes through the SDK's PositionLimit and PowerProtect before it is sent.
// PowerProtect(factor) caps motor power at factor * 10 %; it is a ROS parameter and starts at 1.

#include "go1_ctrl/io_interface.h"

#ifdef GO1_WITH_SDK

#include <cmath>
#include <string>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace go1 {
namespace {

namespace sdk = UNITREE_LEGGED_SDK;
constexpr uint8_t kModeServo = 0x0A;

class RealIO : public IOInterface {
 public:
  RealIO(ros::NodeHandle& /*nh*/, ros::NodeHandle& pnh)
      : udp_(sdk::LOWLEVEL, 8090, "192.168.123.10", 8007), safe_(sdk::LeggedType::Go1) {
    pnh.param("power_protect", power_protect_, 1);
    if (power_protect_ < 1 || power_protect_ > 10) {
      ROS_WARN("[real_io] power_protect must be 1..10, using 1");
      power_protect_ = 1;
    }
    pnh.param("foot_force_scale", foot_force_scale_, 1.0);  // raw SDK units -> N, calibrate
    udp_.InitCmdData(cmd_);
  }

  bool init() override {
    ROS_WARN("[real_io] REAL ROBOT. PowerProtect level %d. Keep the e-stop in reach.",
             power_protect_);
    return true;
  }

  void read(RobotState& s) override {
    udp_.Recv();
    udp_.GetRecv(state_);
    for (int i = 0; i < kNumJoints; ++i) {
      s.q[i] = state_.motorState[i].q;
      s.dq[i] = state_.motorState[i].dq;
      s.tau[i] = state_.motorState[i].tauEst;
    }
    const auto& imu = state_.imu;
    s.quat = Eigen::Quaterniond(imu.quaternion[0], imu.quaternion[1], imu.quaternion[2],
                                imu.quaternion[3]);  // SDK order is w, x, y, z
    s.gyro = Vec3(imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
    s.acc = Vec3(imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
    for (int leg = 0; leg < kNumLegs; ++leg) s.foot_force[leg] = foot_force_scale_ * state_.footForce[leg];
    s.has_ground_truth = false;  // pos/vel must come from Member 2's estimator

    // The robot reports a zero quaternion until the first real packet arrives.
    if (std::abs(s.quat.norm() - 1.0) < 0.1) got_packet_ = true;
  }

  void write(const RobotCommand& c) override {
    cmd_.levelFlag = sdk::LOWLEVEL;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      for (int j = 0; j < kJointsPerLeg; ++j) {
        auto& m = cmd_.motorCmd[jointIndex(leg, j)];
        m.mode = kModeServo;
        m.q = static_cast<float>(c.leg[leg].q[j]);
        m.dq = static_cast<float>(c.leg[leg].dq[j]);
        m.Kp = static_cast<float>(c.leg[leg].kp[j]);
        m.Kd = static_cast<float>(c.leg[leg].kd[j]);
        m.tau = static_cast<float>(c.leg[leg].tau[j]);
      }
    }
    safe_.PositionLimit(cmd_);
    const int ret = safe_.PowerProtect(cmd_, state_, power_protect_);
    if (ret < 0) ROS_ERROR_THROTTLE(1.0, "[real_io] PowerProtect tripped (%d)", ret);
    udp_.SetSend(cmd_);
    udp_.Send();
  }

  bool ready() const override { return got_packet_; }

 private:
  sdk::UDP udp_;
  sdk::Safety safe_;
  sdk::LowCmd cmd_ = {};
  sdk::LowState state_ = {};
  int power_protect_ = 1;
  double foot_force_scale_ = 1.0;
  bool got_packet_ = false;
};

}  // namespace

std::unique_ptr<IOInterface> makeRealIO(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  return std::unique_ptr<IOInterface>(new RealIO(nh, pnh));
}

}  // namespace go1

#else  // built without the SDK

namespace go1 {
std::unique_ptr<IOInterface> makeRealIO(ros::NodeHandle&, ros::NodeHandle&) { return nullptr; }
}  // namespace go1

#endif
