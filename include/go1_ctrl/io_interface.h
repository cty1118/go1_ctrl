#pragma once
// Member 1: hardware abstraction. SimIO talks to Gazebo (unitree_ros), RealIO to the Go1 via the SDK.
// The control loop only sees this interface, so every module runs unchanged in sim and on the robot.

#include <memory>
#include <ros/ros.h>
#include "go1_ctrl/types.h"

namespace go1 {

class IOInterface {
 public:
  virtual ~IOInterface() = default;
  virtual bool init() = 0;
  // Copy the latest measurements into s (s.t is set by the caller).
  virtual void read(RobotState& s) = 0;
  // Send one command for all 12 joints.
  virtual void write(const RobotCommand& c) = 0;
  // True once every required measurement has arrived at least once.
  virtual bool ready() const = 0;
};

std::unique_ptr<IOInterface> makeSimIO(ros::NodeHandle& nh, ros::NodeHandle& pnh);
// Returns nullptr if the package was built without the Unitree SDK.
std::unique_ptr<IOInterface> makeRealIO(ros::NodeHandle& nh, ros::NodeHandle& pnh);

}  // namespace go1
