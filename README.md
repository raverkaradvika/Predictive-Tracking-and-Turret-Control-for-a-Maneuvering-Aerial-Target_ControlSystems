# Predictive-Tracking-and-Turret-Control-for-a-Maneuvering-Aerial-Target_ControlSystems
# ME319 Assignment 2: Lag-Matched Predictive Fire Control (Guardian turret)

Elevation-axis predictive feedback control of a laser turret engaging a maneuvering
target from noisy 20 Hz radar data with temporary measurement loss.
Runs in MATLAB (R2020a+ with the Control System Toolbox for `step4_controller_design.m`).
All other scripts also run in GNU Octave.

## Design in one paragraph
* **Estimator:** radar (r, θ) is converted to Cartesian coordinates. One constant-acceleration filter runs per axis. It starts as a recursive least-squares fit, then switches to a fading-memory α-β-γ filter (θ = 0.85). During a blackout it coasts; when the radar returns, the memory is re-opened only if the innovation exceeds 5 m. Estimates are clipped to |v| ≤ 150 m/s and |a| ≤ 20 m/s².
* **Prediction:** the state is propagated 0.5 s ahead, and θ_ref = atan2(y, x).
* **Controller:** Gc(s) = Kp/(τs+1), with Kp = 2003 V/rad and τ = 0.25 s. These are derived so that the closed loop ≈ e^{-s·tp}, giving Kv = 1/tp, ζ = 0.707 and ωn = 2.83 rad/s. The loop lag then cancels the prediction lead.
* **Firing:** a pulse is fired when the predicted miss distance at the current time is ≤ 0.75·R_eng and the 1 s recharge has elapsed.

## Files
| File | Purpose |
|---|---|
| `turret_plant.m` | **Required.** First-principles plant: states θ, θ̇, i |
| `target_motion_generator.m` | **Required.** Target truth + radar model; scenarios A, B, C (+ stress tests D, E) |
| `turret_controller.m` | **Required.** Estimator → predictor → Gc(s) → fire gate |
| `turret_params.m` | Physical parameters (Section 2 of the brief) |
| `design_params.m` | All design parameters (single finalized design) |
| `target_estimator.m`, `propagate_target.m` | Estimator update and state propagation |
| `simulate_engagement.m` | Closed-loop simulation (RK4, 1 ms, hard stops, true-hit scoring) |
| `run_estimator_openloop.m`, `rmsv.m` | Helpers for step 3 / RMS without toolboxes |
| `step1_plant_check.m` … `step5_simulate_all.m` | Scripts that reproduce every number and figure in the paper |

## Reproduce the results
Run from this folder, in order. Figures are written to `figs/`.
```matlab
step1_plant_check        % plant TF, poles, Bode, ode45-vs-lsim verification
step2_target_check       % envelope checks for all scenarios, radar model
step3_estimator_check    % estimator accuracy, memory-factor sweep, blackout study
step4_controller_design  % margins, closed-loop poles, 0.5 s ramp lag, step response
step5_simulate_all       % closed-loop engagements, Monte Carlo (20 seeds), kappa study
```
Radar noise is seeded (default seed 319), so results are reproducible.
`target_motion_generator(t, scenario, seed)` accepts an optional third argument for Monte-Carlo runs.

## Key results (MATLAB)
| Scenario | Intercept rate (20 seeds) | Mean / worst first hit [s] | Mean hit ratio |
|---|---|---|---|
| A nominal | 1.00 | 1.17 / 1.18 | 0.99 |
| B maneuver | 1.00 | 1.17 / 1.18 | 0.99 |
| C blackout | 1.00 | 2.61 / 3.55 | 0.99 |
| D maneuver + blackout | 1.00 | 2.51 / 3.44 | 0.98 |
| E fast weave in blackout | 1.00 | 2.88 / 3.83 | 0.97 |

Margins: GM 38.0 dB, PM 65.2°. Steady-state ramp lag 0.500 s.
