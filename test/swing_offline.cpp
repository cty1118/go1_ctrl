// Owner: Member 4. Offline check of the swing trajectory, foot placement and the
// swing controller with Member 2's kinematics. Needs no ROS. Build and run from
// the package root:
//   g++ -std=c++14 -O2 -Wall -I include -I /usr/include/eigen3 test/swing_offline.cpp
//       src/swing_controller.cpp src/foot_planner.cpp src/kinematics.cpp
//       src/estimator.cpp -o /tmp/swing_offline
//   /tmp/swing_offline > /tmp/swing.csv
//   python3 test/plot_swing.py /tmp/swing.csv
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "go1_ctrl/estimator.h"  // defaultGo1Geometry()
#include "go1_ctrl/foot_planner.h"
#include "go1_ctrl/kinematics.h"
#include "go1_ctrl/swing_controller.h"

using namespace go1;

namespace {

// Same timing as the placeholder scheduler: trot, T = 0.5 s, duty 0.5.
GaitState trot(double t) {
  const double T = 0.5, duty = 0.5;
  const double offset[kNumLegs] = {0.0, 0.5, 0.5, 0.0};
  GaitState g;
  g.period = T;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    double ph = std::fmod(t / T + offset[leg], 1.0);
    if (ph < 0) ph += 1.0;
    LegGait& l = g.leg[leg];
    l.phase = ph;
    l.contact = ph < duty;
    l.stance_progress = l.contact ? ph / duty : 0.0;
    l.swing_progress = l.contact ? 0.0 : (ph - duty) / (1.0 - duty);
  }
  return g;
}

}  // namespace

int main() {
  const double dt = 0.002;  // 500 Hz
  int failures = 0;
  auto check = [&failures](bool ok, const char* what, double value) {
    std::fprintf(stderr, "[%s] %-36s %.2e\n", ok ? " ok " : "FAIL", what, value);
    if (!ok) ++failures;
  };

  const std::array<LegGeometry, kNumLegs> geometry = defaultGo1Geometry();
  const Kinematics kin(geometry);

  // ---- 1. Trajectory + foothold sweep for the plot (FR leg) ----
  const double t_swing = 0.25, t_stance = 0.25, body_height = 0.32, step_height = 0.08;
  const Vec3 v_cmd(0.5, 0.0, 0.0);
  const Vec3 p_nominal = nominalFoot(geometry[FR]);
  const Vec3 p_liftoff(p_nominal.x() - v_cmd.x() * t_stance / 2.0, p_nominal.y(), -body_height);
  FootPlanner planner;
  const int steps = static_cast<int>(std::round(t_swing / dt));

  std::printf("t,s,px,py,pz,vx,vy,vz,land_x,land_y,land_z,v_body_x,v_filt_x\n");
  Vec3 p_land_prev = Vec3::Zero();
  for (int i = 0; i <= steps; ++i) {
    const double s = static_cast<double>(i) / steps;
    // Measured body velocity ripples around the command, about once per swing as
    // in a trot. The planner low-passes it; the foothold follows the filtered
    // velocity until freeze_progress, then stays fixed.
    const Vec3 v(v_cmd.x() + 0.1 * std::sin(2.0 * M_PI * s), 0.0, 0.0);
    const Vec3 v_filt = planner.filterVelocity(v, dt);
    const Vec3 p_land = planner.update(FR, s, p_nominal, v_filt, v_cmd, t_stance, -body_height);
    Vec3 land_rate = Vec3::Zero();
    if (i > 0) land_rate = (p_land - p_land_prev) / dt;
    p_land_prev = p_land;
    const SwingTarget tg = cycloidSwing(p_liftoff, p_land, step_height, s, t_swing, land_rate);
    std::printf("%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f\n", i * dt, s,
                tg.p.x(), tg.p.y(), tg.p.z(), tg.v.x(), tg.v.y(), tg.v.z(), p_land.x(),
                p_land.y(), p_land.z(), v.x(), v_filt.x());
  }

  // ---- 2. Cycloid properties ----
  const Vec3 pf = planner.foothold(p_nominal, v_cmd, v_cmd, t_stance, -body_height);
  double max_v_err = 0.0;
  for (int i = 1; i < steps; ++i) {
    const double s = static_cast<double>(i) / steps, ds = 1e-6;
    const SwingTarget a = cycloidSwing(p_liftoff, pf, step_height, s - ds, t_swing);
    const SwingTarget b = cycloidSwing(p_liftoff, pf, step_height, s + ds, t_swing);
    const SwingTarget c = cycloidSwing(p_liftoff, pf, step_height, s, t_swing);
    max_v_err = std::max(max_v_err, ((b.p - a.p) / (2.0 * ds * t_swing) - c.v).norm());
  }
  const SwingTarget s0 = cycloidSwing(p_liftoff, pf, step_height, 0.0, t_swing);
  const SwingTarget s5 = cycloidSwing(p_liftoff, pf, step_height, 0.5, t_swing);
  const SwingTarget s1 = cycloidSwing(p_liftoff, pf, step_height, 1.0, t_swing);
  std::fprintf(stderr, "step length %.3f m, apex %.3f m\n", pf.x() - p_liftoff.x(),
               s5.p.z() - p_liftoff.z());
  check(std::abs(s5.p.z() - p_liftoff.z() - step_height) < 1e-9, "apex height = step_height",
        std::abs(s5.p.z() - p_liftoff.z() - step_height));
  check(s0.v.norm() < 1e-9 && s1.v.norm() < 1e-9, "zero velocity at s = 0 and 1",
        std::max(s0.v.norm(), s1.v.norm()));
  check((s1.p - pf).norm() < 1e-9, "ends on the foothold", (s1.p - pf).norm());
  check(max_v_err < 1e-4, "v_des matches finite difference", max_v_err);

  // ---- 3. Member 2's kinematics as used here (their own tests: test_kinematics) ----
  // IK started from a nearby q must return q; the Jacobian must match FK.
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u1(-0.5, 0.5), u2(0.0, 1.5), u3(-2.5, -0.9);
  double ik_err = 0.0, jac_err = 0.0;
  int ik_fail = 0;
  for (int k = 0; k < 1000; ++k) {
    const int leg = k % kNumLegs;
    const Vec3 q(u1(rng), u2(rng), u3(rng));
    Vec3 q_ik = Vec3::Zero();
    if (kin.inverseKinematics(leg, kin.forwardKinematics(leg, q), q + Vec3::Constant(0.05), q_ik))
      ik_err = std::max(ik_err, (q_ik - q).norm());
    else
      ++ik_fail;
    Mat3 Jfd;
    for (int j = 0; j < 3; ++j) {
      Vec3 dq = Vec3::Zero();
      dq[j] = 1e-6;
      Jfd.col(j) = (kin.forwardKinematics(leg, q + dq) - kin.forwardKinematics(leg, q - dq)) / 2e-6;
    }
    jac_err = std::max(jac_err, (Jfd - kin.jacobian(leg, q)).norm());
  }
  const Vec3 stand_q(0.0, 0.67, -1.3);
  const Vec3 stand_p = kin.forwardKinematics(FR, stand_q);
  std::fprintf(stderr, "FR foot at the stand pose: (%.4f, %.4f, %.4f) m\n", stand_p.x(),
               stand_p.y(), stand_p.z());
  check(ik_fail == 0, "IK finds a solution, 1000 random q", ik_fail);
  check(ik_err < 1e-6, "IK(FK(q)) = q, 1000 random q", ik_err);
  check(jac_err < 1e-5, "Jacobian matches finite difference", jac_err);

  // ---- 4. SwingController in the loop with perfect joint tracking ----
  // q follows q_des exactly, so FK(q) must follow p_des: checks timing, IK and dq_des.
  SwingController swing;
  RobotState st;
  for (int leg = 0; leg < kNumLegs; ++leg) st.q.segment<3>(3 * leg) = stand_q;
  st.vel = Vec3(0.3, 0.0, 0.0);
  BodyCmd cmd;
  cmd.vel = Vec3(0.3, 0.0, 0.0);
  RobotCommand out;
  double track_err = 0.0, dq_err = 0.0;
  for (int i = 0; i < 1000; ++i) {  // 2 s = 4 gait periods
    st.t = i * dt;
    const GaitState g = trot(st.t);
    swing.compute(st, g, cmd, out);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (g.leg[leg].contact) continue;
      const Vec3 q_next = out.leg[leg].q;
      const Vec3 p_cmd = kin.forwardKinematics(leg, q_next);
      track_err = std::max(track_err, (p_cmd - swing.lastTarget(leg).p).norm());
      // J * dq_des must reproduce v_des (up to DLS damping and one step of J lag).
      const Vec3 v_from_dq = kin.jacobian(leg, q_next) * out.leg[leg].dq;
      if (i > 0) dq_err = std::max(dq_err, (v_from_dq - swing.lastTarget(leg).v).norm());
      st.dq.segment<3>(3 * leg) = out.leg[leg].dq;
      st.q.segment<3>(3 * leg) = q_next;
    }
  }
  check(swing.ikFailures() == 0, "no IK failures in the trot", swing.ikFailures());
  check(track_err < 1e-6, "FK(q_des) = p_des over 4 periods", track_err);
  check(dq_err < 0.1, "J dq_des ~ v_des [m/s]", dq_err);
  check(std::all_of(out.leg.begin(), out.leg.end(),
                    [](const LegCommand& l) { return l.q.allFinite() && l.tau.allFinite(); }),
        "commands are finite", 0.0);

  std::fprintf(stderr, failures ? "%d check(s) FAILED\n" : "all checks passed\n", failures);
  return failures ? 1 : 0;
}
