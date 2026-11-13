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

// Leg geometry from config/robot.yaml (Member 2). Missing keys keep defaultGo1Geometry().
std::array<LegGeometry, kNumLegs> loadGeometry(ros::NodeHandle& pnh) {
  std::array<LegGeometry, kNumLegs> g = defaultGo1Geometry();
  double hx = g[FL].hipPosition_B.x(), hy = g[FL].hipPosition_B.y(), hz = g[FL].hipPosition_B.z();
  double thigh = g[FL].thighLength, calf = g[FL].calfLength, lat = g[FL].lateralOffset;
  pnh.param("hip_x", hx, hx);
  pnh.param("hip_y", hy, hy);
  pnh.param("hip_z", hz, hz);
  pnh.param("thigh_length", thigh, thigh);
  pnh.param("calf_length", calf, calf);
  pnh.param("lateral_offset", lat, lat);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const bool front = (leg == FR || leg == FL);
    const bool left = (leg == FL || leg == RL);
    g[leg].hipPosition_B = Vec3(front ? hx : -hx, left ? hy : -hy, hz);
    g[leg].lateralOffset = left ? std::abs(lat) : -std::abs(lat);
    g[leg].thighLength = thigh;
    g[leg].calfLength = calf;
  }
  return g;
}

// Estimator tuning from the estimator: block of config/robot.yaml.
EstimatorParams loadEstimatorParams(ros::NodeHandle& pnh) {
  EstimatorParams p;
  pnh.param("estimator/gravity", p.gravity, p.gravity);
  pnh.param("estimator/max_dt", p.maxDt, p.maxDt);
  pnh.param("estimator/min_contact_time", p.minContactTime, p.minContactTime);
  pnh.param("estimator/position_time_constant", p.positionTimeConstant, p.positionTimeConstant);
  pnh.param("estimator/velocity_time_constant", p.velocityTimeConstant, p.velocityTimeConstant);
  pnh.param("estimator/slip_threshold", p.slipThreshold, p.slipThreshold);
  pnh.param("estimator/max_imu_acceleration", p.maxImuAcceleration, p.maxImuAcceleration);
  pnh.param("estimator/max_contact_speed", p.maxContactSpeed, p.maxContactSpeed);
  pnh.param("estimator/use_ground_truth", p.useGroundTruth, p.useGroundTruth);
  return p;
}

// Gait table from config/gaits.yaml (Members 3 and 4). Must run before the first start().
void configureScheduler(ros::NodeHandle& pnh, GaitScheduler& scheduler) {
  const GaitType gaits[] = {GaitType::WALK, GaitType::TROT, GaitType::CANTER, GaitType::GALLOP};
  for (GaitType gt : gaits) {
    const std::string key = std::string("gaits/") + gaitName(gt);
    GaitParameters p = GaitScheduler::defaults(gt);
    std::vector<double> offset, duty;
    if (!pnh.hasParam(key)) {
      ROS_WARN("[main] %s not set, using the built-in %s parameters", key.c_str(), gaitName(gt));
      continue;
    }
    pnh.param(key + "/period", p.period, p.period);
    if (pnh.getParam(key + "/offset", offset) && offset.size() == 4)
      for (int i = 0; i < 4; ++i) p.offset[i] = offset[i];
    if (pnh.getParam(key + "/duty", duty) && duty.size() == 4)
      for (int i = 0; i < 4; ++i) p.duty[i] = duty[i];
    try {
      scheduler.configure(gt, p);
    } catch (const std::exception& e) {
      ROS_ERROR("[main] %s rejected (%s), using the built-in parameters", key.c_str(), e.what());
    }
  }
  double transition_time;
  if (pnh.getParam("gaits/transition_time", transition_time)) {
    try {
      scheduler.setTransitionTime(transition_time);
    } catch (const std::exception& e) {
      ROS_ERROR("[main] gaits/transition_time rejected (%s)", e.what());
    }
  }
  bool ground_walk = true, hang = false;
  pnh.param("gaits/ground_walk", ground_walk, ground_walk);
  pnh.param("hang", hang, hang);
  if (hang && ground_walk) {
    ROS_INFO("[main] robot is hanging: Walk runs without the support check");
    ground_walk = false;  // feet never touch the ground on the rig
  }
  scheduler.setGroundWalk(ground_walk);
}

// Feet, body position and orientation in the world frame for the Walk support check.
WalkFeedback walkFeedback(const RobotState& s, const Kinematics& kin,
                          const std::array<bool, kNumLegs>& contact) {
  WalkFeedback f;
  const Mat3 R = s.quat.normalized().toRotationMatrix();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 q(s.q(jointIndex(leg, 0)), s.q(jointIndex(leg, 1)), s.q(jointIndex(leg, 2)));
    f.feet[leg] = s.pos + R * kin.forwardKinematics(leg, q);
  }
  f.com = s.pos;
  f.world_R_body = R;
  f.contact = contact;
  f.stamp = s.t;
  return f;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go1_ctrl");
  ros::NodeHandle nh, pnh("~");

  // ---- parameters (config/fsm.yaml; robot.yaml and gaits.yaml are read further down) ----
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
  const std::array<LegGeometry, kNumLegs> geometry = loadGeometry(pnh);  // config/robot.yaml
  const Kinematics kin(geometry);
  Estimator estimator(geometry, loadEstimatorParams(pnh));
  GaitScheduler scheduler;
  configureScheduler(pnh, scheduler);  // config/gaits.yaml
  bool scheduler_running = false;      // false until GAIT is entered (again)
  SwingController swing(SwingParams(), geometry);
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
ros::Publisher gait_debug_pub = nh.advertise<std_msgs::Float64MultiArray>("go1_ctrl/gait_debug", 10);
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
// Member 7: Preserve gait data for post-safety debugging.
GaitState debug_gait;
std::array<bool, kNumLegs> debug_contact{};
bool debug_gait_valid = false;

    if (fsm.mode() != Mode::GAIT) scheduler_running = false;  // next GAIT entry restarts it

    switch (fsm.mode()) {
      case Mode::PASSIVE:
        fsm.passiveCommand(cmd);
        break;

      case Mode::STAND:
        fsm.standCommand(s, cmd);
        break;

      case Mode::GAIT: {
        fsm.takeGaitChanged();
        // 2. estimate body state
        const std::array<bool, kNumLegs> contact = contact_det.update(s);
        estimator.update(s, contact);
        // 3. gait schedule: start() when GAIT is entered, request() for a smooth
        //    switch while already walking (the scheduler blends at a cycle boundary)
        if (!scheduler_running) {
          scheduler.start(fsm.gait(), s.t);
          scheduler_running = true;
        } else if (fsm.gait() != scheduler.activeGait()) {
          if (scheduler.walkBlocked()) {
            scheduler.start(fsm.gait(), s.t);  // Walk is holding for support: switch directly
          } else if (!scheduler.transitioning()) {
            scheduler.request(fsm.gait());
          }
        }
        scheduler.setWalkFeedback(walkFeedback(s, kin, contact));
        const GaitState g = scheduler.update(s.t);
debug_gait = g;
debug_contact = contact;
debug_gait_valid = true;
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
    // Member 7: Publish gait phase, contact, force, and torque data.
    // Leg order: FR, FL, RR, RL.
    // Per leg: phase, scheduled_contact, detected_contact,
    // foot_force, commanded_tau[3], estimated_tau[3].
    if (debug_gait_valid && fsm.mode() == Mode::GAIT) {
      std_msgs::Float64MultiArray gait_msg;
      gait_msg.data.reserve(40);
    
      for (int leg = 0; leg < kNumLegs; ++leg) {
        gait_msg.data.push_back(debug_gait.leg[leg].phase);
        gait_msg.data.push_back(debug_gait.leg[leg].contact ? 1.0 : 0.0);
        gait_msg.data.push_back(debug_contact[leg] ? 1.0 : 0.0);
        gait_msg.data.push_back(s.foot_force(leg));
    
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
          gait_msg.data.push_back(cmd.leg[leg].tau(joint));
        }
    
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
          gait_msg.data.push_back(s.tau(jointIndex(leg, joint)));
        }
      }
    
      gait_debug_pub.publish(gait_msg);
    }

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
