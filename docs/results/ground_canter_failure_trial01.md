# Ground-Supported Canter Failure — Trial 01

**Project:** EEL 5669 — Unitree Go1 Gait Controller  
**Evaluator:** Member 7 — Testing and Evaluation  
**Date:** November 14, 2026  
**Test Environment:** ROS Noetic, Gazebo 11  
**Controller Baseline:** `030c250` (integrated team controller)

## 1. Objective

Evaluate the ground-supported stability of the integrated Unitree Go1 Canter controller and record quantitative evidence of the observed failure.

## 2. Experimental Setup

- Simulation launched using `roslaunch go1_ctrl sim.launch hang:=false`.
- Robot initialized in PASSIVE mode.
- STAND mode activated before Canter.
- CANTER activated through `/go1_ctrl/mode_cmd` with `data: 5`.
- No intentional translational velocity command was applied.
- Robot body states, IMU measurements, gait diagnostics, mode transitions, and foot-contact forces were recorded.

**Raw ROS bag (stored locally):** `~/go1_experiments/canter_ground_trial01.bag`

## 3. Observed Results

The robot successfully entered STAND mode and remained upright before Canter activation.

After entering CANTER, the robot lost stability and fell. The controller subsequently transitioned to PASSIVE.

### Recorded Mode Transitions

| Event | Simulation Time (s) |
|---|---:|
| PASSIVE | 10.6314 |
| STAND | 14.7868 |
| GAIT canter | 21.7120 |
| PASSIVE (after failure) | 23.6262 |

**Time in CANTER before PASSIVE: 1.9142 seconds.**

### Recorded Data

- Total ROS bag duration: approximately 18.3 seconds
- ROS bag size: 44.7 MB
- Total recorded messages: 115,367
- Body-state samples analyzed during Canter: 9,465
- Gait diagnostic samples analyzed during Canter: 957
- Foot-contact forces recorded for all four legs

## 4. Diagnostic Evidence

The analysis generated the following evidence:

1. **Body roll and pitch:** Orientation measurements during Canter.
2. **Body height:** Changes in the robot's vertical position.
3. **Foot-contact forces:** Contact-force measurements for FR, FL, RR, and RL legs.
4. **CSV datasets:** Numerical measurements for further investigation.

The measurements provide a basis for investigating body instability and foot-contact behavior. However, the exact mechanical or control-related failure mechanism has not yet been determined.

## 5. Comparison with Other Ground-Supported Gaits

| Metric | Walk | Trot | Canter |
|---|---:|---:|---:|
| Time before PASSIVE (s) | 5.1972 | 4.0538 | 1.9142 |
| Observed outcome | Fell | Fell | Fell |

Canter remained active for less time than Walk and Trot in these individual trials.

**Important:** These results represent one instrumented trial per gait. They do not establish a statistically reliable ranking of gait stability.

## 6. Evidence Files

- `ground_canter_failure_trial01.png` — Diagnostic visualization
- `ground_canter_body_trial01.csv` — Body height and orientation data
- `ground_canter_contacts_trial01.csv` — Scheduled/detected contacts and foot forces
- `scripts/evaluation/analyze_ground_canter_failure.py` — Reproducible analysis script

## 7. Limitations

- Only one ground-supported Canter trial was analyzed.
- The precise root cause of the fall has not been established.
- No quantitative claims about maximum roll, pitch, or contact mismatch are made without further numerical inspection.
- The raw ROS bag remains stored locally and is not included in the Git repository.

## 8. Conclusion

The integrated controller successfully transitions from STAND to CANTER but fails to maintain ground-supported stability.

The controller transitions to PASSIVE approximately **1.9142 seconds** after Canter activation.

The recorded diagnostic data and visualization can support further investigation of stance control, contact behavior, and gait transitions.
