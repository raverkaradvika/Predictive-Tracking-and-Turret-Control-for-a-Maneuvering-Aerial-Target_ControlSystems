function D = design_params()
% DESIGN_PARAMS  All design choices (not given in the brief) in one place.
% One finalized design: these are NOT retuned per scenario.

% ---------------- timing ----------------
D.Ts   = 0.05;            % radar period [s]
D.tp   = 0.50;            % prescribed prediction horizon [s]

% ---------------- estimator (Idea 3) ----------------
% Fading-memory alpha-beta-gamma filter, one per Cartesian axis.
% theta = memory factor (0..1): close to 1 -> long memory, smooth, slow.
D.th_nom   = 0.85;        % normal tracking memory: minimax choice from the step-3 sweep
% (smallest WORST-CASE prediction error over scenarios A-E)
D.sig_meas = 1.0;         % per-axis Cartesian measurement noise used in LS start-up [m]
% (range noise 1 m; cross-range r*sig_theta = 0.3-0.7 m)
D.nu_gate  = 5.0;         % innovation gate [m] = theoretical worst-case coast drift
% 0.5*(2*amax)*t_bo^2 = 5 m, typical noise ~1-1.5 m
D.t_bo     = 0.50;        % maximum blackout length used to inflate covariance [s]
D.vmax    = 150;          % physical speed bound [m/s]  -> clamp on estimate
D.amax    = 20;           % physical accel bound [m/s^2] -> clamp on estimate

% ---------------- controller (Idea 1) ----------------
P = turret_params();
D.Kv  = 1/D.tp;           % velocity error constant = 2 1/s  (loop lag = tp)
D.tau = D.tp/2;           % low-pass time constant = 0.25 s  (zeta = 0.707)
D.Kp  = D.Kv / P.K0;      % ~2003 V/rad  so that Kp*K0 = Kv

% ---------------- firing logic (Idea 5) ----------------
D.Reng      = 2.0;        % engagement radius [m]
D.kappa     = 0.75;       % fire if predicted miss <= kappa*Reng
D.t_rech    = 1.0;        % recharge time [s]
D.t_margin  = 1e-3;       % small safety margin on the recharge timer [s]

% ---------------- study hook (NOT used in the finalized design) ----------------
% simulate_engagement can put a modified copy of D in this global variable so
% parameter studies (e.g. kappa sweep) reach turret_controller, whose input
% list is fixed by the brief. Empty by default -> the values above are used.
global DESIGN_OVERRIDE
if isstruct(DESIGN_OVERRIDE), D = DESIGN_OVERRIDE; end
end