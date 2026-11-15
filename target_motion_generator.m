function [r_meas, theta_meas, meas_valid, truth] = target_motion_generator(t, scenario, seed)
% TARGET_MOTION_GENERATOR  V-44 target truth + simulated 20 Hz radar (required signature).
%
% Inputs : t        time [s]
%          scenario 'A' nominal | 'B' maneuvering | 'C' measurement loss
%                   'D' (extra stress test) = maneuver of B + blackout of C
%                   'E' (extra stress test) = fast near-limit weave that starts
%                       exactly when the radar goes blind (1.0 s) + blackout
%                   'R' RANDOM admissible case drawn from 'seed' (generalization
%                       test): random start, speed, heading, maneuver and blackout
%          seed     (optional) noise seed for Monte-Carlo runs; default 319
% Outputs: r_meas, theta_meas  latest radar sample (range [m], elevation [rad])
%          meas_valid          1 if the latest radar sample exists, 0 during blackout
%          truth (optional 4th output, for plots/hit checking only — the
%                 controller never sees it): struct with x,y,vx,vy,ax,ay,r,th
%
% Radar model: samples are taken at t_k = k*Ts (Ts = 50 ms) and HELD between
% samples (zero-order hold). The controller detects a NEW sample itself by
% watching floor(t/Ts) increase, so this works whatever rate it is called at.
% During blackout the outputs are NaN, so any accidental use shows up at once.

Ts      = 0.05;                 % radar period [s] (20 Hz)
sig_r   = 1.0;                  % range noise std [m]
sig_th  = deg2rad(0.02);        % elevation noise std [rad]

% ---- fixed, reproducible noise sequence (same every run) ----
if nargin < 3, seed = 319; end
persistent nr nth seed_used
if isempty(nr) || seed ~= seed_used
    old = rng;                     % save caller's random state
    rng(seed, 'twister');          % fixed seed -> identical noise every run
    nr  = randn(1e5, 1);           % enough samples for 5000 s
    nth = randn(1e5, 1);
    rng(old);                      % restore caller's random state (MATLAB and Octave)
    seed_used = seed;
end

% ---- latest radar sample instant at or before t ----
k  = floor(t/Ts + 1e-9);        % sample index (1e-9 guards against round-off at exact multiples)
tk = k*Ts;

% ---- blackout window for this scenario ----
S = scenario_params(scenario, seed);                 % all case parameters
t_bo_on = S.bo_on;  t_bo_len = S.bo_len;
in_blackout = (tk >= t_bo_on) && (tk < t_bo_on + t_bo_len);

if in_blackout
    r_meas = NaN; theta_meas = NaN; meas_valid = 0;   % no information
else
    T = target_truth(tk, S);                    % truth AT the sample instant
    r_meas     = T.r  + sig_r  * nr(k+1);              % additive zero-mean noise
    theta_meas = T.th + sig_th * nth(k+1);
    meas_valid = 1;
end

if nargout > 3
    truth = target_truth(t, S);                 % truth at the CURRENT time
end
end

% =====================================================================
function S = scenario_params(scenario, seed)
% Parameters of one engagement case. Cached, because random cases are
% generated with rejection sampling (checked against the envelope).
persistent cache_key cache_S
key = sprintf('%s_%d', upper(char(scenario)), seed);
if ischar(cache_key) && strcmp(key, cache_key), S = cache_S; return; end

sc = upper(char(scenario));
% Default = scenario A: straight, level, inbound at 120 m/s (V-1-like run)
S.p0 = [1800; 600];  S.v0 = [-120; 0];          % start (r0 ~ 1897 m, el 18.4 deg)
S.Ax = 0; S.wx = 1; S.Ay = 0; S.wy = 1; S.tm = 1.0;   % no maneuver
S.bo_on = Inf; S.bo_len = 0;                    % no blackout
switch sc
    case {'B','D'}, S.Ax = 10; S.wx = 1.2; S.Ay = 14; S.wy = 1.8;  % |a| <= 17.2 m/s^2
    case 'E',       S.Ax = 12; S.wx = 3.0; S.Ay = 15; S.wy = 4.0;  % |a| <= 19.2, fast weave
    case 'R',       S = random_case(seed);                         % generalization test
end
if any(sc == 'CDE'), S.bo_on = 1.00; S.bo_len = 0.50; end           % blackout 1.0-1.5 s
cache_key = key;  cache_S = S;
end

% =====================================================================
function T = target_truth(t, S)
% Analytic target trajectory in the elevation plane (turret at origin,
% x horizontal, y up). Analytic => exactly smooth; velocity and acceleration
% are known exactly, so the envelope can be verified.
%   base: p = p0 + v0 t
%   maneuver from t = tm:  a = [Ax sin(wx tau); Ay sin(wy tau)], tau = t - tm
%   (acceleration starts at 0 -> no jump at onset). Integrating twice:
%   dv = (A/w)(1 - cos(w tau)),  dp = (A/w)(tau - sin(w tau)/w)
p = S.p0 + S.v0*t;  v = S.v0;  a = [0; 0];
if t > S.tm && (S.Ax ~= 0 || S.Ay ~= 0)
    tau = t - S.tm;
    a = a + [S.Ax*sin(S.wx*tau); S.Ay*sin(S.wy*tau)];
    v = v + [S.Ax/S.wx*(1-cos(S.wx*tau)); S.Ay/S.wy*(1-cos(S.wy*tau))];
    p = p + [S.Ax/S.wx*(tau - sin(S.wx*tau)/S.wx); S.Ay/S.wy*(tau - sin(S.wy*tau)/S.wy)];
end
T.x = p(1);  T.y = p(2);  T.vx = v(1);  T.vy = v(2);  T.ax = a(1);  T.ay = a(2);
T.r  = hypot(p(1), p(2));       % 2-D range [m]
T.th = atan2(p(2), p(1));       % elevation [rad]
end