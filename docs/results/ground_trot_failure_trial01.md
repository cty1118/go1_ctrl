# Ground-Supported Trot Failure — Trial 01

**Project:** EEL 5669 Unitree Go1 Gait Controller
**Evaluator:** Member 7 — Testing & Evaluation
**Date:** November 14, 2026
**Environment:** ROS Noetic, Gazebo 11, ground-supported Go1
**Controller version:** main, commit 030c250

## Objective

Evaluate ground-supported Trot stability using the newly integrated stance controller.

## Experimental procedure

1. Launch `go1_ctrl sim.launch hang:=false`.
2. Start recording robot state, IMU, foot forces, gait diagnostics, and controller mode.
3. Command STAND and allow the robot to stabilize.
4. Command TROT at zero requested velocity.
5. Observe the fall and record the transition to PASSIVE.

## Results

| Event | Simulation time |
|---|---:|
| STAND activated | 17.2720 s |
| TROT activated | 23.5686 s |
| PASSIVE activated | 27.6224 s |
| Duration in TROT | 4.0538 s |

### Body stability

| Time after TROT | Height (m) | Roll (deg) | Pitch (deg) |
|---|---:|---:|---:|
| 0.00 s | 0.352 | -0.01 | 0.07 |
| 1.00 s | 0.331 | -2.79 | -0.27 |
| 2.00 s | 0.323 | -13.05 | 1.51 |
| 3.00 s | 0.316 | -1.98 | 8.10 |
| 3.90 s | 0.313 | 49.14 | -7.26 |
| 4.05 s | 0.292 | 57.57 | -18.17 |

### Critical contact observations

| Time after TROT | Scheduled stance | Detected contact |
|---|---|---|
| 3.60 s | FL + RR | FR + RR + RL |
| 3.70 s | FR + RL | RL only |
| 3.80 s | FR + RL | None |
| 3.90 s | FR + RL | FR + RR |

At approximately 3.80 s, neither scheduled stance leg had detected
contact, while body roll reached approximately 40 degrees.

## Interpretation

- The robot successfully stood before Trot activation.
- Ground-supported Trot ended in a fall and PASSIVE transition.
- The instability was predominantly lateral (roll).
- Scheduled stance and measured ground contact diverged.
- The measurements are consistent with loss of effective support
  accompanying the rapid body roll.

## Limitations

- This is one fully instrumented ground-supported Trot trial.
- Contact detection uses a 20 N threshold; below-threshold forces
  do not necessarily imply zero physical contact.
- Sampled contact and body-orientation values are observational.
- The specific root cause has not been established.
- Stance control, foot placement, contact detection, and transition
  blending require further investigation.
- Previous suspended-gait tracking results do not prove that the
  swing controller is fault-free under ground-supported conditions.

## Evidence

- Diagnostic figure: `ground_trot_failure_trial01.png`
- Raw ROS bag (retained locally):
  `~/go1_experiments/trot_ground_failure_trial01.bag`
- ROS bag duration: 19.9 s
- ROS bag messages: 126,392

## Conclusion

Ground-supported Trot is not yet stable with the tested controller
configuration. Loss of effective ground support coincides with a
rapid increase in body roll before the controller enters PASSIVE.
Further controlled experiments are required to establish causality.
