// Member 1: keyboard teleop. Run in its own terminal:  rosrun go1_ctrl keyboard
//
//   1 passive   2 stand   3 walk   4 trot   5 canter   6 gallop   space stop (back to stand)
//   w/s forward/back   a/d left/right   q/e turn   x zero velocity   Ctrl-C quit

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>

namespace {

struct RawTerminal {
  termios old_{};
  RawTerminal() {
    tcgetattr(STDIN_FILENO, &old_);
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;  // read() returns after 0.1 s without a key
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &old_); }
};

double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go1_keyboard");
  ros::NodeHandle nh, pnh("~");
  double step_v, step_w, max_v, max_w;
  pnh.param("step_v", step_v, 0.05);
  pnh.param("step_w", step_w, 0.1);
  pnh.param("max_v", max_v, 0.6);
  pnh.param("max_w", max_w, 1.0);

  ros::Publisher mode_pub = nh.advertise<std_msgs::Int8>("go1_ctrl/mode_cmd", 10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("go1_ctrl/cmd_vel", 1);

  std::printf(
      "Go1 keyboard\n"
      "  1 passive  2 stand  3 walk  4 trot  5 canter  6 gallop  space stop\n"
      "  w/s vx  a/d vy  q/e yaw  x zero velocity  Ctrl-C quit\n");

  RawTerminal term;
  geometry_msgs::Twist tw;
  ros::Rate rate(20);
  while (ros::ok()) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      int mode = -1;
      switch (c) {
        case ' ': mode = 0; break;
        case '1': case '2': case '3': case '4': case '5': case '6': mode = c - '0'; break;
        case 'w': tw.linear.x += step_v; break;
        case 's': tw.linear.x -= step_v; break;
        case 'a': tw.linear.y += step_v; break;
        case 'd': tw.linear.y -= step_v; break;
        case 'q': tw.angular.z += step_w; break;
        case 'e': tw.angular.z -= step_w; break;
        case 'x': tw = geometry_msgs::Twist(); break;
        default: break;
      }
      if (mode == 0 || mode == 1) tw = geometry_msgs::Twist();  // stopping also zeros velocity
      tw.linear.x = clamp(tw.linear.x, max_v);
      tw.linear.y = clamp(tw.linear.y, max_v);
      tw.angular.z = clamp(tw.angular.z, max_w);
      if (mode >= 0) {
        std_msgs::Int8 m;
        m.data = static_cast<int8_t>(mode);
        mode_pub.publish(m);
      }
      std::printf("\r mode key %c | vx %+.2f vy %+.2f wz %+.2f   ", c, tw.linear.x, tw.linear.y,
                  tw.angular.z);
      std::fflush(stdout);
    }
    vel_pub.publish(tw);  // keep publishing so the controller's timeout does not zero it
    rate.sleep();
  }
  std::printf("\n");
  return 0;
}
