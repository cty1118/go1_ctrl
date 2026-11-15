# Ground-Supported Gallop Failure — Trial 01

**Project:** EEL 5669 — Unitree Go1 Gait Controller  
**Evaluator:** Member 7 — Testing and Evaluation  
**Date:** November 14, 2026  
**Test Environment:** ROS Noetic, Gazebo 11  
**Controller Baseline:** `030c250` (integrated team controller)

## 1. Objective

Evaluate the stability of the integrated Unitree Go1 controller during ground-supported Gallop operation and document the observed failure using simulation data.

## 2. Experimental Setup

- Simulation launched using `roslaunch go1_ctrl sim.launch hang:=false`.
- Robot initialized in PASSIVE mode.
- STAND mode activated before Gallop.
- GALLOP mode activated through `/go1_ctrl/mode_cmd` with `data: 6`.
- No intentional translational velocity command was applied.
- Body states, IMU measurements, gait diagnostics, mode transitions, and foot-contact forces were recorded.

**Raw ROS bag (stored locally):** `~/go1_experiments/gallop_ground_trial01.bag`

## 3. Observed Results

The robot successfully entered STAND mode and remained upright before Gallop activation.

After entering GALLOP, the robot lost stability and fell. The controller subsequently transitioned to PASSIVE.

### Recorded Mode Transitions

| Event | Simulation Time (s) |
|---|---:|
| PASSIVE | 6.1916 |
| STAND | 19.0890 |
| GAIT gallop | 27.6754 |
| PASSIVE (after failure) | 31.2918 |

**Time in GALLOP before PASSIVE: 3.6164 seconds.**

### Recorded Data

- Total ROS bag duration: approximately 26.4 seconds
- ROS bag size: 64.9 MB
- Total recorded messages: 167,516
- Body-state samples analyzed during Gallop: 17,833
- Gait diagnostic samples analyzed during Gallop: 1,806
- Foot-contact measurements recorded for all four legs

## 4. Diagnostic Evidence

The analysis generated:

1. **Body roll and pitch:** Orientation measurements during Gallop.
2. **Body height:** Changes in the robot's vertical position.
3. **Foot-contact forces:** Measurements for FR, FL, RR, and RL legs.
4. **CSV datasets:** Numerical body-state and contact data for further analysis.

These measurements can help investigate the relationship between body instability, foot contact, and gait transitions. They do not establish the precise cause of the failure.

## 5. Comparison with All Four Ground-Supported Gaits

| Metric | Walk | Trot | Canter | Gallop |
|---|---:|---:|---:|---:|
| Time before PASSIVE (s) | 5.1972 | 4.0538 | 1.9142 | 3.6164 |
| Observed outcome | Fell | Fell | Fell | Fell |

All four gaits failed to maintain ground-supported stability during their respective trials.

Walk had the longest time before PASSIVE and Canter had the shortest, but these results do not establish a statistically reliable stability ranking.

## 6. Evidence Files

- `ground_gallop_failure_trial01.png` — Diagnostic visualization
- `ground_gallop_body_trial01.csv` — Body height and orientation data
- `ground_gallop_contacts_trial01.csv` — Scheduled/detected contacts and foot forces
- `scripts/evaluation/analyze_ground_gallop_failure.py` — Reproducible analysis script

## 7. Limitations

- Only one instrumented ground-supported Gallop trial was analyzed.
- The exact root cause of the fall has not been established.
- No quantitative conclusions about maximum roll, pitch, or contact mismatch are drawn without further numerical inspection.
- Comparisons across gaits are descriptive rather than statistical.
- The raw ROS bag remains stored locally and is not included in the Git repository.

## 8. Conclusion

The integrated Unitree Go1 controller successfully transitions from STAND to GALLOP but does not maintain ground-supported stability.

The controller transitions to PASSIVE approximately **3.6164 seconds** after Gallop activation.

Together with the Walk, Trot, and Canter results, this trial provides evidence that all four currently implemented gaits require additional ground-stability validation and controller improvements.
