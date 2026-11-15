function S = random_case(seed)
% RANDOM_CASE  Random admissible engagement for the generalization test.
% Every draw is checked against the brief's envelope on a 10 ms grid over
% 0-10 s (800 <= r <= 2000 m, |v| <= 150 m/s, |a| <= 20 m/s^2, elevation
% inside the turret limits) and redrawn if it violates any limit.
% The caller's random state is saved and restored, so radar noise is unaffected.
old = rng;  rng(10000 + seed, 'twister');      % case-specific, reproducible draws
cleanup = onCleanup(@() rng(old));             % restore on every exit path
t  = 0:0.01:10;
for attempt = 1:500
    r0  = 1300 + 650*rand;                 % start range [m]
    el0 = deg2rad(8 + 37*rand);            % start elevation [deg -> rad]
    spd = 60 + 80*rand;                    % base speed [m/s]
    hdg = pi + el0 + deg2rad(-50 + 100*rand); % roughly inbound, +-50 deg
    A   = 19*sqrt(rand);                   % total maneuver accel <= 19 m/s^2
    ang = 2*pi*rand;                       % split between x and y
    S.p0 = r0*[cos(el0); sin(el0)];
    S.v0 = spd*[cos(hdg); sin(hdg)];
    S.Ax = A*cos(ang);  S.Ay = A*sin(ang);
    S.wx = 0.4 + 3.6*rand;  S.wy = 0.4 + 3.6*rand;   % 0.4-4 rad/s
    S.tm = 1 + 3*rand;                     % maneuver onset 1-4 s
    S.bo_on  = 1 + 4*rand;                 % blackout onset >= 1 s (brief)
    S.bo_len = 0.1 + 0.4*rand;             % blackout length <= 0.5 s (brief)
    if admissible(S, t), return; end
end
error('random_case: no admissible case found for seed %d', seed);
end

function ok = admissible(S, t)
tau = max(t - S.tm, 0);
px = S.p0(1) + S.v0(1)*t + S.Ax/S.wx*(tau - sin(S.wx*tau)/S.wx);
py = S.p0(2) + S.v0(2)*t + S.Ay/S.wy*(tau - sin(S.wy*tau)/S.wy);
vx = S.v0(1) + S.Ax/S.wx*(1 - cos(S.wx*tau));
vy = S.v0(2) + S.Ay/S.wy*(1 - cos(S.wy*tau));
ax = S.Ax*sin(S.wx*tau);  ay = S.Ay*sin(S.wy*tau);
r  = hypot(px, py);  el = atan2(py, px);
ok = all(r >= 800 & r <= 2000) && all(hypot(vx,vy) <= 150) && ...
    all(hypot(ax,ay) <= 20) && all(el >= deg2rad(-10) & el <= deg2rad(85));
end