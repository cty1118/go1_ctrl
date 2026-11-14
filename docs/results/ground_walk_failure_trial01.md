# Ground-Supported Walk Failure — Trial 01

**Project:** EEL 5669 — Unitree Go1 Gait Controller  
**Evaluator:** Member 7 — Testing and Evaluation  
**Date:** November 14, 2026  
**Test Environment:** ROS Noetic, Gazebo 11  
**Controller Baseline:** `030c250` (integrated team controller)

## 1. Objective

Evaluate whether the integrated Unitree Go1 controller can maintain stability during ground-supported Walk operation, and collect quantitative evidence of any failure.

## 2. Experimental Setup

- Simulation launched using `roslaunch go1_ctrl sim.launch hang:=false`.
- Robot initialized in PASSIVE mode.
- STAND mode activated before Walk.
- WALK mode activated using `/go1_ctrl/mode_cmd` with `data: 3`.
- No intentional translational velocity command was applied.
- Robot body states, IMU, gait diagnostics, mode transitions, and four foot-force topics were recorded.

**Raw ROS bag (stored locally):** `~/go1_experiments/walk_ground_trial01.bag`

## 3. Observed Results

The robot successfully entered STAND mode and remained upright before the Walk command.

After entering WALK, the robot subsequently fell and the controller returned to PASSIVE.

### Recorded Mode Transitions

| Event | Simulation Time (s) |
|---|---:|
| PASSIVE | 5.5306 |
| STAND | 8.4586 |
| GAIT walk | 14.3522 |
| PASSIVE (after failure) | 19.5494 |

**Time in WALK before PASSIVE: 5.1972 seconds.**

### Recorded Data

- Total ROS bag duration: approximately 16.0 seconds
- ROS bag size: 39.7 MB
- Total recorded messages: 102,564
- Body-state samples analyzed: 25,756
- Gait diagnostic samples analyzed: 2,597
- Foot-force measurements recorded for all four legs

## 4. Diagnostic Evidence

The analysis script generates:

1. **Body roll and pitch:** Orientation changes during WALK.
2. **Body height:** Vertical displacement during the test.
3. **Foot contact forces:** Contact-force measurements for FR, FL, RR, and RL legs.
4. **CSV datasets:** Numerical body-state and contact data for additional analysis.

The available measurements support further investigation of body instability and foot-contact behavior. The exact failure mechanism has not yet been established.

## 5. Comparison with Ground-Supported Trot

| Metric | Walk | Trot |
|---|---:|---:|
| Time before PASSIVE | 5.1972 s | 4.0538 s |
| Observed outcome | Fell | Fell |

Walk remained active approximately 1.1434 seconds longer than Trot in these individual trials.

**Caution:** These are separate trials, not repeated statistical evaluations. The duration difference alone does not establish that Walk is more stable than Trot.

## 6. Evidence Files

- `ground_walk_failure_trial01.png` — Diagnostic visualization
- `ground_walk_body_trial01.csv` — Body height and orientation data
- `ground_walk_contacts_trial01.csv` — Scheduled/detected contacts and foot forces
- `scripts/evaluation/analyze_ground_walk_failure.py` — Reproducible analysis script

## 7. Limitations

- Only one ground-supported Walk trial is documented.
- The precise root cause of the fall has not been identified.
- No quantitative claim about maximum roll, pitch, or contact mismatch is made without further numerical inspection.
- The raw ROS bag is stored locally and is not included in the Git repository.

## 8. Conclusion

The integrated controller successfully transitions from STAND to WALK but fails to maintain ground-supported stability throughout the trial.

The controller returns to PASSIVE after approximately **5.1972 seconds** in WALK.

The recorded body-state and foot-contact datasets provide a basis for identifying the mechanisms responsible for the failure and evaluating future controller improvements.
