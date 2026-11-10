// Member 1: main control loop (500 Hz).
//
//   1 read sensors -> 2 estimate -> 3 schedule -> 4a swing / 4b stance -> 5 blend
//   -> safety -> 6 write motors
//
// In simulation the loop runs on Gazebo time (/use_sim_time = true, set in sim.launch), so
// ros::Rate and ros::Time::now() follow the simulator, also when it runs slower than real time.
//
// Topics:
//   sub go1_ctrl/mode_cmd  std_msgs/Int8        0 stop, 1 passive, 2 stand, 3..6 walk..gallop
//   sub go1_ctrl/cmd_vel   geometry_msgs/Twist  body velocity command (x, y, yaw rate)
//   pub go1_ctrl/mode      std_msgs/String      current mode / gait, latched

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include "go1_ctrl/contact_detector.h"
#include "go1_ctrl/estimator.h"
#include "go1_ctrl/fsm.h"
#include "go1_ctrl/gait_scheduler.h"
#include "go1_ctrl/io_interface.h"
#include "go1_ctrl/stance_controller.h"
#include "go1_ctrl/swing_controller.h"
#include "go1_ctrl/transition_blender.h"

using namespace go1;

namespace {

Vec3 paramVec3(ros::NodeHandle& pnh, const std::string& name, const Vec3& def) {
  std::vector<double> v;
  if (!pnh.getParam(name, v)) return def;
  if (v.size() != 3) {
    ROS_WARN("param %s must have 3 values, using the default", name.c_str());
    return def;
  }
  return Vec3(v[0], v[1], v[2]);
}

struct Inputs {
  std::atomic<int> mode_cmd{-1};
  BodyCmd body;
  ros::Time last_vel;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go1_ctrl");
  ros::NodeHandle nh, pnh("~");

  // ---- parameters (config/fsm.yaml) ----
  std::string io_type;
  double rate_hz, vel_timeout;
  pnh.param<std::string>("io", io_type, "sim");
  pnh.param("rate", rate_hz, 500.0);
  pnh.param("cmd_vel_timeout", vel_timeout, 0.5);

  FsmParams fp;
  fp.stand_q = paramVec3(pnh, "stand_q", fp.stand_q);
  pnh.param("stand_time", fp.stand_time, fp.stand_time);
  pnh.param("stand_kp", fp.stand_kp, fp.stand_kp);
  pnh.param("stand_kd", fp.stand_kd, fp.stand_kd);
  pnh.param("passive_kd", fp.passive_kd, fp.passive_kd);

  SafetyParams sp;
  sp.q_min = paramVec3(pnh, "q_min", sp.q_min);
  sp.q_max = paramVec3(pnh, "q_max", sp.q_max);
  sp.tau_max = paramVec3(pnh, "tau_max", sp.tau_max);
  pnh.param("fall_angle", sp.fall_angle, sp.fall_angle);
  pnh.param("min_height", sp.min_height, sp.min_height);
  pnh.param("check_height", sp.check_height, io_type == "sim");

  // ---- IO ----
  std::unique_ptr<IOInterface> io;
  if (io_type == "sim") {
    io = makeSimIO(nh, pnh);
  } else if (io_type == "real") {
    io = makeRealIO(nh, pnh);
    if (!io) {
      ROS_FATAL("io:=real but go1_ctrl was built without unitree_legged_sdk (see CMakeLists.txt)");
      return 1;
    }
  } else {
    ROS_FATAL("unknown io '%s' (use sim or real)", io_type.c_str());
    return 1;
  }
  if (!io->init()) return 1;

  // ---- modules ----
  FSM fsm(fp);
  Safety safety(sp);
  Estimator estimator;
  GaitScheduler scheduler;
  SwingController swing;
  StanceController stance;
  ContactDetector contact_det;
  TransitionBlender blender;

  // ---- operator inputs ----
  Inputs in;
  ros::Subscriber mode_sub = nh.subscribe<std_msgs::Int8>(
      "go1_ctrl/mode_cmd", 10, [&in](const std_msgs::Int8::ConstPtr& m) { in.mode_cmd = m->data; });
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>(
      "go1_ctrl/cmd_vel", 1, [&in](const geometry_msgs::Twist::ConstPtr& m) {
        in.body.vel = Vec3(m->linear.x, m->linear.y, 0.0);
        in.body.yaw_rate = m->angular.z;
        in.last_vel = ros::Time::now();
      });
  ros::Publisher mode_pub = nh.advertise<std_msgs::String>("go1_ctrl/mode", 1, true);
  ros::Publisher swing_debug_pub = nh.advertise<std_msgs::Float64MultiArray>("go1_ctrl/swing_debug", 10);
  // ---- wait for the first sensor data (Gazebo may still be loading) ----
  ROS_INFO("[main] waiting for sensor data (io=%s) ...", io_type.c_str());
  ros::Rate wait(50);
  while (ros::ok() && !io->ready()) {
    ros::spinOnce();
    RobotState tmp;
    RobotCommand damp;
    io->read(tmp);
    fsm.passiveCommand(damp);
    io->write(damp);
    ROS_INFO_THROTTLE(2.0, "[main] still waiting for joint states and IMU");
    wait.sleep();
  }
  if (!ros::ok()) return 0;
  ROS_INFO("[main] sensors ready, %.0f Hz loop. Mode PASSIVE: press 2 to stand.", rate_hz);

  RobotState s;
  RobotCommand cmd, cmd_swing, cmd_stance;
  std::string last_status;
  ros::Rate rate(rate_hz);
  const double t_begin = ros::Time::now().toSec();
  long n_overrun = 0;

  while (ros::ok()) {
    ros::spinOnce();
    s.t = ros::Time::now().toSec() - t_begin;

    // 1. read sensors
    io->read(s);

    // operator command
    const int mc = in.mode_cmd.exchange(-1);
    if (mc >= 0) fsm.request(mc, s);
    BodyCmd body = in.body;
    if ((ros::Time::now() - in.last_vel).toSec() > vel_timeout) {
      body.vel.setZero();  // stop if the teleop goes quiet
      body.yaw_rate = 0.0;
    }

    // safety on the measured state
    const std::string fault = safety.checkState(s, fsm.mode());
    if (!fault.empty()) fsm.forcePassive(fault);

    switch (fsm.mode()) {
      case Mode::PASSIVE:
        fsm.passiveCommand(cmd);
        break;

      case Mode::STAND:
        fsm.standCommand(s, cmd);
        break;

      case Mode::GAIT: {
        if (fsm.takeGaitChanged()) scheduler.start(fsm.gait(), s.t);
        // 2. estimate body state (pass-through of ground truth until Member 2 is done)
        const std::array<bool, kNumLegs> contact = contact_det.update(s);
        estimator.update(s, contact);
        // 3. gait schedule
        const GaitState g = scheduler.update(s.t);
        // 4a/4b. swing and stance commands
        swing.compute(s, g, body, cmd_swing);
        // Member 7: Publish swing foot tracking data.
        // Leg order: FR, FL, RR, RL.
        // Each leg: p_des_x, p_des_y, p_des_z, p_x, p_y, p_z.
        std_msgs::Float64MultiArray debug_msg;
        debug_msg.data.reserve(24);

        for (int leg = 0; leg < kNumLegs; ++leg) {
          const auto& target = swing.lastTarget(leg);
          const auto& measured = swing.lastMeasured(leg);

          for (int axis = 0; axis < 3; ++axis) {
            debug_msg.data.push_back(target.p(axis));
          }
          for (int axis = 0; axis < 3; ++axis) {
            debug_msg.data.push_back(measured.p(axis));
          }
        }

        swing_debug_pub.publish(debug_msg);
        stance.compute(s, g, body, cmd_stance);
        // 5. blend at touchdown / liftoff
        blender.blend(g, contact, cmd_swing, cmd_stance, cmd);
        break;
      }
    }

    // final safety: no NaN to the motors, joint and torque limits
    if (!isFinite(cmd)) {
      fsm.forcePassive("non-finite command");
      fsm.passiveCommand(cmd);
    }
    if (fsm.mode() != Mode::PASSIVE) safety.limit(s, cmd);

    // 6. write motor commands
    io->write(cmd);

    std::string status = modeName(fsm.mode());
    if (fsm.mode() == Mode::GAIT) status += std::string(" ") + gaitName(fsm.gait());
    if (status != last_status) {
      std_msgs::String m;
      m.data = status;
      mode_pub.publish(m);
      last_status = status;
    }

    if (!rate.sleep()) {
      ++n_overrun;
      ROS_WARN_THROTTLE(5.0, "[main] loop overran %ld times (cycle %.2f ms)", n_overrun,
                        rate.cycleTime().toSec() * 1e3);
    }
  }

  // leave the motors damped on shutdown
  fsm.passiveCommand(cmd);
  io->write(cmd);
  return 0;
}
