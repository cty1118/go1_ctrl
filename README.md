# go1_ctrl

Self-written gait controller for the Unitree Go1 (EEL 5669, Class Project Part 2).

Goal: walk, trot, canter and gallop, first in Gazebo/RViz and then on the real robot, using a combination of position control and force/impedance control.

## Setup (Part 1 VM: Ubuntu 20.04, ROS Noetic, Gazebo 11)

```bash
cd ~/unitree_ws/src
git clone https://github.com/cty1118/go1_ctrl.git
cd ~/unitree_ws && catkin_make && source devel/setup.bash
```

`unitree_ros` and `unitree_legged_msgs` must already be in `~/unitree_ws/src` (they are in the Part 1 VM).

## Status

This repository only contains the directory structure. Every file holds a one-line note naming its owner; the owner writes its contents. `package.xml` and `CMakeLists.txt` are kept minimal so `catkin_make` still succeeds for the whole workspace.

## Layout

| Path | Owner | Contents |
| --- | --- | --- |
| `include/go1_ctrl/types.h` | Members 1 + 2 | Shared data structures and leg order |
| `io_interface.h`, `src/sim_io.cpp`, `src/real_io.cpp`, `src/fsm.cpp`, `src/main.cpp`, `launch/`, `package.xml`, `CMakeLists.txt` | Member 1 | IO, state machine, safety, main loop, build |
| `kinematics.h`, `estimator.h`, `config/robot.yaml`, `test/` | Member 2 | Kinematics and state estimation |
| `gait_scheduler.h`, `config/gaits.yaml` | Member 3 | Gait scheduler, Walk and Trot |
| `swing_controller.h`, `foot_planner.h` | Member 4 | Swing leg, foot placement, Canter and Gallop |
| `stance_controller.h`, `config/stance.yaml` | Member 5 | Stance leg impedance control |
| `contact_detector.h`, `transition_blender.h`, `config/transition.yaml` | Member 6 | Transitions, contact detection, real robot |
| `scripts/`, `docs/experiments.md` | Member 7 | Logging, plots, experiments |

## Team rules

- Leg order everywhere: FR = 0, FL = 1, RR = 2, RL = 3; joints per leg: hip, thigh, calf.
- Only edit the files you own. Ask before changing someone else's interface.
- Work on your own branch, `feature/<module>`, and open a pull request into `main`.
- Commit under your own GitHub account so each contribution is visible.
- The control loop runs on simulation time (`use_sim_time`), never wall-clock time.
- Do not run unitree_guide's `junior_ctrl` at the same time as our controller.
