function [p, v] = propagate_target(X, dt)
% PROPAGATE_TARGET  Push the estimate X = [pos; vel; acc] (3x2) forward by dt.
% Used for (a) the prediction  dt = (t - t_k) + tp  -> turret reference
%          (b) the "now" state dt = (t - t_k)       -> firing decision
p = X(1,:) + X(2,:)*dt + 0.5*X(3,:)*dt^2;
v = X(2,:) + X(3,:)*dt;
end