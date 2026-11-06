// Member 1: SimIO. Talks to the Go1 in Gazebo through unitree_ros.
//
//   in : /<robot>_gazebo/<joint>_controller/state   unitree_legged_msgs/MotorState  (12)
//        /trunk_imu                                 sensor_msgs/Imu
//        /visual/<leg>_foot_contact/the_force       geometry_msgs/WrenchStamped     (4)
//        /gazebo/model_states                       gazebo_msgs/ModelStates (ground truth)
//   out: /<robot>_gazebo/<joint>_controller/command unitree_legged_msgs/MotorCmd    (12)
//
// Joint index i = 3 * leg + joint, with legs FR, FL, RR, RL and joints hip, thigh, calf,
// which is the same order as the controller names below, so no remapping table is needed.

#include <array>
#include <cmath>
#include <mutex>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>

#include "go1_ctrl/io_interface.h"

namespace go1 {
namespace {

const std::array<const char*, kNumLegs> kLegNames = {"FR", "FL", "RR", "RL"};
const std::array<const char*, kJointsPerLeg> kJointNames = {"hip", "thigh", "calf"};
constexpr uint8_t kModeServo = 0x0A;  // PMSM servo mode in unitree_legged_control

class SimIO : public IOInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // holds a RobotState (Vec4, Quaterniond) and is created with new

  SimIO(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    pnh.param<std::string>("robot_name", robot_, "go1");
    pnh.param<std::string>("gazebo_model", model_, robot_ + "_gazebo");
    pnh.param<std::string>("imu_topic", imu_topic_, "/trunk_imu");
  }

  bool init() override {
    const std::string ns = "/" + robot_ + "_gazebo/";
    for (int leg = 0; leg < kNumLegs; ++leg) {
      for (int j = 0; j < kJointsPerLeg; ++j) {
        const int i = jointIndex(leg, j);
        const std::string ctrl =
            ns + std::string(kLegNames[leg]) + "_" + kJointNames[j] + "_controller/";
        cmd_pub_[i] = nh_.advertise<unitree_legged_msgs::MotorCmd>(ctrl + "command", 1);
        joint_sub_[i] = nh_.subscribe<unitree_legged_msgs::MotorState>(
            ctrl + "state", 1, [this, i](const unitree_legged_msgs::MotorState::ConstPtr& m) {
              std::lock_guard<std::mutex> lk(mtx_);
              s_.q[i] = m->q;
              s_.dq[i] = m->dq;
              s_.tau[i] = m->tauEst;
              joint_seen_[i] = true;
            });
      }
      const std::string foot = "/visual/" + std::string(kLegNames[leg]) + "_foot_contact/the_force";
      foot_sub_[leg] = nh_.subscribe<geometry_msgs::WrenchStamped>(
          foot, 1, [this, leg](const geometry_msgs::WrenchStamped::ConstPtr& m) {
            std::lock_guard<std::mutex> lk(mtx_);
            const auto& f = m->wrench.force;
            s_.foot_force[leg] = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
          });
    }
    imu_sub_ = nh_.subscribe(imu_topic_, 1, &SimIO::imuCb, this);
    model_sub_ = nh_.subscribe("/gazebo/model_states", 1, &SimIO::modelCb, this);
    ROS_INFO("[sim_io] robot '%s', controllers under %s, ground truth from model '%s'",
             robot_.c_str(), ns.c_str(), model_.c_str());
    return true;
  }

  void read(RobotState& s) override {
    std::lock_guard<std::mutex> lk(mtx_);
    const double t = s.t;
    s = s_;
    s.t = t;
  }

  void write(const RobotCommand& c) override {
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const LegCommand& l = c.leg[leg];
      for (int j = 0; j < kJointsPerLeg; ++j) {
        unitree_legged_msgs::MotorCmd m;
        m.mode = kModeServo;
        m.q = l.q[j];
        m.dq = l.dq[j];
        m.Kp = l.kp[j];
        m.Kd = l.kd[j];
        m.tau = l.tau[j];
        cmd_pub_[jointIndex(leg, j)].publish(m);
      }
    }
  }

  bool ready() const override {
    std::lock_guard<std::mutex> lk(mtx_);
    for (bool b : joint_seen_)
      if (!b) return false;
    return imu_seen_;  // ground truth is optional: the estimator can replace it
  }

 private:
  void imuCb(const sensor_msgs::Imu::ConstPtr& m) {
    std::lock_guard<std::mutex> lk(mtx_);
    s_.quat = Eigen::Quaterniond(m->orientation.w, m->orientation.x, m->orientation.y,
                                 m->orientation.z);
    s_.gyro = Vec3(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    s_.acc = Vec3(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    imu_seen_ = true;
  }

  void modelCb(const gazebo_msgs::ModelStates::ConstPtr& m) {
    for (size_t k = 0; k < m->name.size(); ++k) {
      if (m->name[k] != model_) continue;
      std::lock_guard<std::mutex> lk(mtx_);
      const auto& p = m->pose[k].position;
      const auto& v = m->twist[k].linear;
      s_.pos = Vec3(p.x, p.y, p.z);
      s_.vel = Vec3(v.x, v.y, v.z);  // gazebo_msgs twists are in the world frame
      s_.has_ground_truth = true;
      return;
    }
  }

  ros::NodeHandle nh_;
  std::string robot_, model_, imu_topic_;
  std::array<ros::Publisher, kNumJoints> cmd_pub_;
  std::array<ros::Subscriber, kNumJoints> joint_sub_;
  std::array<ros::Subscriber, kNumLegs> foot_sub_;
  ros::Subscriber imu_sub_, model_sub_;

  mutable std::mutex mtx_;
  RobotState s_;
  std::array<bool, kNumJoints> joint_seen_{};
  bool imu_seen_ = false;
};

}  // namespace

std::unique_ptr<IOInterface> makeSimIO(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  return std::unique_ptr<IOInterface>(new SimIO(nh, pnh));
}

}  // namespace go1
