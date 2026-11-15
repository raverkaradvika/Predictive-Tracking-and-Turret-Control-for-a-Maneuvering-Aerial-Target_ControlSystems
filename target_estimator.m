function E = target_estimator(E, z, valid, D)
% TARGET_ESTIMATOR  One radar tick of the coast-then-reacquire estimator (Idea 3).
%
% E     : estimator struct: E.X = 3x2 [pos; vel; acc] (columns = x, y axes),
%         E.P = 3x3 covariance used only in LS mode, E.mode ('LS' or 'FM'),
%         E.init, E.after_bo (just left blackout)
% z     : 1x2 measured [x y] (from radar r, theta)  -- ignored if ~valid
% valid : 1 if a radar measurement exists at this tick, 0 during blackout
% D     : design parameters
%
% Model per axis (constant acceleration, sample time T):
%   predict : X- = F X,   F = [1 T T^2/2; 0 1 T; 0 0 1]
%   update  : nu = z - p-;  X = X- + K nu
%
% Two gain regimes with one smooth hand-over:
%  'LS' (start-up / re-acquisition): recursive least squares = Kalman filter
%       with zero process noise. It is exactly a least-squares parabola fit
%       to every sample since (re)start; its gains shrink as data builds up.
%  'FM' (normal tracking): fading-memory alpha-beta-gamma, ONE knob theta:
%       K = [g; h/T; 2k/T^2], g = 1-theta^3, h = 1.5(1-theta^2)(1-theta), k = 0.5(1-theta)^3
%  Hand-over LS -> FM when the LS position gain falls below g of FM, i.e.
%  when the growing data window carries as much information as the fading memory.
%
% Coast-then-reacquire:
%  * blackout: predict only (coast); estimates clamped to physical limits
%  * first sample after blackout: if |nu| > nu_gate the memory is stale ->
%    go back to LS mode with covariance inflated by what the target could
%    have done unseen (|Delta a| <= 2 amax), so new data is trusted again.

T = D.Ts;
F = [1 T T^2/2; 0 1 T; 0 0 1];      % constant-acceleration transition matrix
H = [1 0 0];                        % we measure position only
R = D.sig_meas^2;                   % measurement noise variance per axis [m^2]

th = D.th_nom;                      % fading-memory gain vector (constant)
K_FM = [1 - th^3; 1.5*(1 - th^2)*(1 - th)/T; 2*0.5*(1 - th)^3/T^2];

% ---------- first ever measurement: initialise ----------
if ~E.init
    if valid
        E.X = [z; 0 0; 0 0];                    % position = measurement
        E.P = diag([R, D.vmax^2, D.amax^2]);    % vel/acc unknown: only physical bounds
        E.mode = 'LS';  E.init = true;  E.after_bo = false;
    end
    return
end

% ---------- predict (always; during blackout this is the "coast") ----------
E.X = F*E.X;
if strcmp(E.mode,'LS'), E.P = F*E.P*F'; end      % zero process noise

if ~valid
    E.after_bo = true;
else
    nu = z - E.X(1,:);                           % innovation (measurement surprise), 1x2

    % ---------- innovation-gated re-acquisition ----------
    if E.after_bo
        if norm(nu) > D.nu_gate                  % memory stale: target maneuvered unseen
            E.mode = 'LS';
            % covariance = what could have happened in a blackout of t_bo:
            % accel change up to 2*amax, giving velocity/position errors accordingly
            da = 2*D.amax;
            E.P = diag([norm(nu)^2, (da*D.t_bo)^2, da^2]);
        end
        E.after_bo = false;
    end

    % ---------- gains ----------
    if strcmp(E.mode,'LS')
        K = E.P*H' / (H*E.P*H' + R);             % Kalman / recursive-LS gain (3x1)
        if K(1) <= K_FM(1)                       % LS window now "as informed" as FM
            E.mode = 'FM';  K = K_FM;
        else
            E.P = (eye(3) - K*H)*E.P;            % shrink uncertainty
        end
    else
        K = K_FM;
    end

    % ---------- correct (same gain on x and y axes) ----------
    E.X = E.X + K*nu;
end

% ---------- physics clamp: estimates may never exceed the target envelope ----------
sp = norm(E.X(2,:));  if sp > D.vmax, E.X(2,:) = E.X(2,:)*D.vmax/sp; end
ac = norm(E.X(3,:));  if ac > D.amax, E.X(3,:) = E.X(3,:)*D.amax/ac; end
end