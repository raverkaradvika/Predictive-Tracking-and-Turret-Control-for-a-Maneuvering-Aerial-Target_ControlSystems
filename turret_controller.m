function [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
% TURRET_CONTROLLER  Complete predictive-feedback fire-control law (required signature).
%
%   radar (r,theta) -> [Idea 3] estimator -> 0.5 s prediction -> elevation reference
%   -> [Idea 1] lag-matched compensator Gc(s) = Kp/(tau s + 1) -> motor voltage
%   current estimate + turret angle -> [Idea 5] fire gate -> fire_cmd
%
% Can be called at any rate >= 20 Hz (e.g. every 1 ms simulation step).
% A NEW radar sample is detected when floor(t/Ts) increases; the measurement
% is used only if meas_valid = 1 and it is not NaN.
% Internal memory (persistent) resets automatically when time restarts.

persistent S D P

% ---------- (re)initialise at the start of every run ----------
if isempty(S) || t < S.t_last || t == 0
    D = design_params();    % read parameters once per run (fast: not every 1 ms call)
    P = turret_params();
    S.E      = struct('X',zeros(3,2),'P',zeros(3),'mode','LS','init',false,'after_bo',false);
    S.k_last = -1;          % index of last processed radar tick
    S.t_last = t;           % time of previous call
    S.uf     = 0;           % state of the low-pass in Gc(s) (= output voltage)
    S.t_fire = -Inf;        % time of the last laser pulse
    S.valid_last = false;   % was the last radar tick a real measurement?
end
dt = t - S.t_last;  S.t_last = t;

% ================= 1. ESTIMATION: process new radar tick(s) =================
k = floor(t/D.Ts + 1e-9);
while S.k_last < k
    S.k_last = S.k_last + 1;
    ok = (S.k_last == k) && meas_valid && ~isnan(r_meas) && ~isnan(theta_meas);
    z  = [0 0];
    if ok, z = [r_meas*cos(theta_meas), r_meas*sin(theta_meas)]; end   % polar -> Cartesian
    S.E = target_estimator(S.E, z, ok, D);
    S.valid_last = ok;
end

if ~S.E.init                       % no measurement yet: do nothing
    V_m = 0; fire_cmd = 0; return
end
age = t - S.k_last*D.Ts;           % time since the estimate's tick [s] (0 .. 0.05)

% ================= 2. PREDICTION -> elevation reference =================
pp    = propagate_target(S.E.X, age + D.tp);            % target position at t + tp
th_ref = atan2(pp(2), pp(1));                           % future elevation
th_ref = min(max(th_ref, P.th_min), P.th_max);          % respect the mechanical stops

% ================= 3. FEEDBACK: Gc(s) = Kp/(tau s + 1) =================
% Exact discretisation of the first-order lag over the call interval dt:
%   uf <- uf + (1 - exp(-dt/tau)) * (Kp*e - uf)
e    = th_ref - theta_turret;
S.uf = S.uf + (1 - exp(-dt/D.tau)) * (D.Kp*e - S.uf);
V_m  = S.uf;

% ================= 4. FIRING DECISION (Idea 5) =================
fire_cmd = 0;
pn  = propagate_target(S.E.X, age);                     % target position NOW
rn  = norm(pn);
dth = atan2(pn(2), pn(1)) - theta_turret;               % pointing error w.r.t. CURRENT target
miss_hat = rn*abs(sin(dth));                            % perpendicular distance beam-target
in_front = cos(dth) > 0;                                % target in front of the turret
ready    = (t - S.t_fire) >= D.t_rech + D.t_margin;    % laser recharged
trusted  = strcmp(S.E.mode,'FM') && S.valid_last;       % converged & radar currently seeing
if ready && trusted && in_front && miss_hat <= D.kappa*D.Reng
    fire_cmd = 1;                                       % 0 -> 1 edge = one pulse
    S.t_fire = t;
end
end