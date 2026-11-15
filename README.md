# ME 319 Assignment 2 — Mumbai 2044: Predictive Tracking and Turret Control
### Lag-Matched Predictive Fire Control for a Laser Turret Engaging a Maneuvering Aerial Target

**Author:** Advika Raverkar, Department of Mechanical Engineering, IIT Bombay  
**Course:** ME 319 Control Systems (Autumn 2026, S3)

This repository contains the complete MATLAB implementation of a predictive feedback fire-control system for the elevation axis of a laser turret. It estimates a maneuvering target's motion from noisy 20 Hz radar data, predicts the target's position 0.5 s ahead, drives the turret with a linear time-invariant compensator, and fires laser pulses. It also handles temporary radar loss. Every number and figure in the paper can be reproduced with the scripts below.

---

## Design summary

| Block | What it does | Key parameters |
|---|---|---|
| **Estimator** | Radar (r, θ) is converted to Cartesian (x, y). One constant-acceleration filter runs per axis. It starts as a recursive least-squares fit and switches to a fading-memory α-β-γ filter. During a blackout it coasts. On recovery it re-opens its memory only if the innovation exceeds 5 m. Estimates are clipped to the physical limits. | memory θ = 0.85, gate 5 m, \|v\| ≤ 150 m/s, \|a\| ≤ 20 m/s² |
| **Predictor** | Propagates the estimate to t + tp and sets θ_ref = atan2(y, x), clipped to −10°…85°. | tp = 0.5 s |
| **Controller** | Gc(s) = Kp / (τs + 1), derived so that the closed loop ≈ e^(−s·tp). The 0.5 s loop lag then cancels the 0.5 s prediction lead, and the turret points at the target's current position. | Kp = 2003 V/rad, τ = 0.25 s, Kv = 1/tp = 2 s⁻¹, ζ = 0.707, ωn = 2.83 rad/s |
| **Fire gate** | Fires one pulse (a 0→1 edge) when the predicted miss distance at the current time is at most κ·R_eng, the laser has recharged, and the track is valid. | κ = 0.75, recharge 1 s |

**Plant** (derived from first principles in `turret_plant.m`):
Gp(s) = 1000 / [s(320s² + 3350s + 1.0015×10⁶)]. It has an integrator and a lightly damped mode at 55.9 rad/s (ζ = 0.094).

---

## Repository structure

```
├── README.md
├── turret_plant.m              REQUIRED  plant dynamics (states: θ, θ̇, i)
├── target_motion_generator.m   REQUIRED  target truth + 20 Hz radar model
├── turret_controller.m         REQUIRED  estimator → predictor → Gc(s) → fire gate
│
├── turret_params.m             physical parameters (Section 2 of the brief)
├── design_params.m             all design parameters: ONE finalized design
├── target_estimator.m          one radar tick of the estimator
├── propagate_target.m          pushes the estimate forward in time
├── simulate_engagement.m       closed-loop simulation (RK4, 1 ms, hard stops, true-hit scoring)
├── random_case.m               random admissible engagements (generalization test)
├── score_engagement.m          interception / first-hit / accuracy metrics
├── run_estimator_openloop.m    estimator-only run used in step 3
├── rmsv.m                      toolbox-free RMS
│
├── step1_plant_check.m         plant TF, poles, Bode, ode45-vs-lsim verification
├── step2_target_check.m        trajectory envelope checks, radar noise check
├── step3_estimator_check.m     estimator accuracy, memory sweep, blackout study
├── step4_controller_design.m   margins, closed-loop poles, ramp lag, step response
├── step5_simulate_all.m        all scenarios, Monte Carlo (20 seeds), κ study
├── step6_generalization.m      50 random cases + ablation study
```

### Required function signatures (as specified in the brief)
```matlab
[r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
```
* `turret_controller` keeps its own internal memory, which resets automatically when time restarts (t = 0 or t decreases), so each evaluation case starts fresh.
* It can be called at any rate of 20 Hz or more. A new radar sample is detected when ⌊t / 0.05⌋ increases. Samples with `meas_valid = 0` or NaN values are ignored, and the filter coasts.
* `target_motion_generator` also accepts an optional third argument, `seed`: the radar-noise seed, and for scenario `'R'` the random case number.

### Scenarios
| ID | Description |
|---|---|
| `A` | Nominal: straight, level, inbound at 120 m/s from (1800, 600) m |
| `B` | Maneuvering: smooth x–y weave from t = 1 s (\|a\| ≤ 17.2 m/s²) |
| `C` | Measurement loss: A plus a radar blackout from 1.0 to 1.5 s |
| `D` | Stress test: B plus the blackout |
| `E` | Stress test: fast near-limit weave (\|a\| ≤ 19.2 m/s²) starting at the moment of blackout |
| `R` | Random admissible case (start, speed, heading, maneuver and blackout drawn from `seed`) |

All trajectories are checked against 800 ≤ r ≤ 2000 m, \|v\| ≤ 150 m/s and \|a\| ≤ 20 m/s².

---

## How to reproduce

**Requirements:** MATLAB R2020a or newer. The Control System Toolbox is needed for `step1_plant_check.m` and `step4_controller_design.m`. All other files use base MATLAB and also run in GNU Octave 8.

1. Clone or download this repository and set it as the MATLAB **Current Folder**.
2. Run the scripts in order. Each one prints its results. Only the following files need to be run:

```matlab
step1_plant_check         % ~5 s
step2_target_check        % ~10 s
step3_estimator_check     % ~1 min
step4_controller_design   % ~5 s
step5_simulate_all        % a few minutes (≈160 closed-loop simulations)
step6_generalization      % a few minutes (≈130 closed-loop simulations)
```

3. To simulate a single engagement:
```matlab
D = design_params();                                  % the finalized design
L = simulate_engagement('B', D);                      % scenario B, default noise seed
L.shots                                               % rows: [time, true miss distance, hit]
L = simulate_engagement('R', D, struct('seed', 7));   % random case #7
```

Radar noise is seeded (default seed 319), so every run is reproducible.

> Do **not** run `turret_plant`, `turret_controller` etc. directly. They are functions that need inputs. Run the `step*` scripts instead.

---

## Key results

| Test | Result |
|---|---|
| Stability margins | GM = 38.0 dB, PM = 65.2° |
| Closed-loop dominant poles | −1.99 ± 2.01j (ζ = 0.703, ωn = 2.83 rad/s) |
| Steady-state ramp lag | 0.500 s (design: tp = 0.5 s) |
| Scenarios A–E, 20 noise seeds each | 100 % interception, mean first hit 1.17–2.88 s, hit ratio 0.97–0.99 |
| 50 random admissible cases | 50 / 50 intercepted, first hit mean 1.30 s (worst 2.67 s), first-hit miss mean 1.29 m (worst 1.86 m) |

**Ablation** (scenarios B, C, E plus 20 random cases):

| Variant | Intercept rate | RMS pointing error |
|---|---|---|
| Lag-matched loop + 0.5 s prediction (**this design**) | **1.00** | **1.1 mrad** |
| Same loop, no prediction | 0.65 | 25.7 mrad |
| Fast loop (Kv = 10) + 0.5 s prediction | 0.00 | 21.4 mrad |
| Fast loop, no prediction | 0.13 | 5.3 mrad |

Removing either the prediction or the lag matching breaks interception. That confirms the core idea: the prediction lead and the loop lag must cancel.

---

## Use of AI tools
Claude (Anthropic) was used as a coding and analysis assistant. All scripts were run and every result was checked in MATLAB. Details of what was accepted, modified and rejected are in the AI appendix of the paper.
