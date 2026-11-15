function R = run_estimator_openloop(sc, D, tk, seed)
% RUN_ESTIMATOR_OPENLOOP  Feed one scenario's radar data through the estimator
% and record current-estimate and 0.5 s-prediction errors against the truth.
if nargin < 4, seed = 319; end   % radar-noise seed (Monte Carlo)
E = struct('X',zeros(3,2),'P',zeros(3),'mode','LS','init',false,'after_bo',false);
N = numel(tk); R.e_now = nan(N,1); R.e_pred = nan(N,1); R.e_el = nan(N,1);
for j = 1:N
    [rm, thm, ok] = target_motion_generator(tk(j), sc, seed);
    z = [rm*cos(thm), rm*sin(thm)];                % polar -> Cartesian
    E = target_estimator(E, z, ok, D);
    if ~E.init, continue; end
    [~,~,~,T0] = target_motion_generator(tk(j), sc, seed);         % truth now
    [~,~,~,Tp] = target_motion_generator(tk(j)+D.tp, sc, seed);    % truth at t+tp
    pn = propagate_target(E.X, 0);
    pp = propagate_target(E.X, D.tp);
    R.e_now(j)  = norm(pn - [T0.x T0.y]);
    R.e_pred(j) = norm(pp - [Tp.x Tp.y]);
    R.e_el(j)   = atan2(pp(2),pp(1)) - Tp.th;     % reference angle error [rad]
end
end