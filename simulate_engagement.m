function L = simulate_engagement(scenario, D, opts)
% SIMULATE_ENGAGEMENT  Closed-loop simulation of one engagement (0..tend).
% Fixed-step RK4 (dt = 1 ms) on turret_plant, controller called every step,
% voltage held constant over a step (zero-order hold). Returns a log struct L.
if nargin < 3, opts = struct(); end
if ~isfield(opts,'tend'), opts.tend = 10; end
if ~isfield(opts,'dt'),   opts.dt   = 1e-3; end
if ~isfield(opts,'th0'),  opts.th0  = 0; end      % initial turret elevation [rad]
if ~isfield(opts,'seed'), opts.seed = 319; end    % radar-noise seed

P  = turret_params();
dt = opts.dt;  t = (0:dt:opts.tend)';  N = numel(t);
clear turret_controller                            % fresh controller memory
global DESIGN_OVERRIDE
DESIGN_OVERRIDE = D;                               % controller uses exactly this design
cleanup = onCleanup(@() clear_override());         % ...and only during this run

x = [opts.th0; 0; 0];                              % [theta; theta_dot; i]
L.t = t;  L.th = zeros(N,1); L.thd = L.th; L.i = L.th; L.V = L.th;
L.th_true = L.th; L.r_true = L.th; L.fire = false(N,1);
L.shots = [];                                      % rows: [t, miss_true, hit]
t_last_pulse = -Inf;  fire_prev = 0;

for j = 1:N
    [rm, thm, ok, T] = target_motion_generator(t(j), scenario, opts.seed);
    [V, fc] = turret_controller(t(j), rm, thm, ok, x(1), x(2));

    % ---- laser pulse bookkeeping (same rules as the brief) ----
    if fc == 1 && fire_prev == 0 && (t(j) - t_last_pulse) >= P_rech()
        t_last_pulse = t(j);
        dth  = T.th - x(1);                                % true pointing error
        miss = T.r*abs(sin(dth));  if cos(dth) <= 0, miss = Inf; end
        L.shots(end+1,:) = [t(j), miss, miss <= D.Reng];
        L.fire(j) = true;
    end
    fire_prev = fc;

    % ---- log ----
    L.th(j) = x(1); L.thd(j) = x(2); L.i(j) = x(3); L.V(j) = V;
    L.th_true(j) = T.th; L.r_true(j) = T.r;

    % ---- integrate plant one step (RK4, V held) ----
    if j < N
        f  = @(xx) plant_vec(t(j), V, xx);
        k1 = f(x); k2 = f(x + dt/2*k1); k3 = f(x + dt/2*k2); k4 = f(x + dt*k3);
        x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        % ---- mechanical stops: clamp angle, kill outward velocity ----
        if x(1) > P.th_max, x(1) = P.th_max; x(2) = min(x(2),0); end
        if x(1) < P.th_min, x(1) = P.th_min; x(2) = max(x(2),0); end
    end
end
if isempty(L.shots), L.shots = zeros(0,3); end
end

function dx = plant_vec(t, V, x)
[a, b, c] = turret_plant(t, V, x(1), x(2), x(3));
dx = [a; b; c];
end

function tr = P_rech()
tr = 1.0;                                          % minimum inter-pulse interval [s]
end

function clear_override()
global DESIGN_OVERRIDE
DESIGN_OVERRIDE = [];
end