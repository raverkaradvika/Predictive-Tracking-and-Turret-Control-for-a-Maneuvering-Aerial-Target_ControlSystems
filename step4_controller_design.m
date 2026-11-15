%% STEP 4 - Controller design and stability analysis (Idea 1: lag-matched loop)
% Needs the Control System Toolbox (tf, margin, feedback, step, lsim, bode).
%
% Design logic (analytic, no numerical tuning):
%   want closed loop T(s) ~ e^{-s tp}  (loop lag cancels the 0.5 s prediction)
%   e^{-s tp} = 1 - tp s + (tp^2/2) s^2 - ...
%   2nd-order T = 1/(1 + a s + b s^2):   a = tp,  b = tp^2/2
%     -> wn = sqrt(2)/tp = 2.83 rad/s,  zeta = 1/sqrt(2) = 0.707
%   With Gp ~ K0/s and Gc = Kp/(tau s + 1):  L = Kp K0 / (s (tau s + 1))
%     -> Kv = Kp K0 = 1/tp = 2 1/s,   tau = tp/2 = 0.25 s,   Kp = 2/K0 ~ 2003 V/rad

clear; close all; clc;
P = turret_params();  D = design_params();
if ~exist('figs','dir'), mkdir figs; end
s  = tf('s');

Gp = P.N*P.Kt / ( s*((P.Lm*s + P.Rm)*(P.Jeq*s + P.b) + P.N^2*P.Kt*P.Ke) );
Gc = D.Kp / (D.tau*s + 1);                 % lag-matched compensator
L  = Gc*Gp;                                % loop transfer function
T  = feedback(L, 1);                       % closed loop  theta / theta_ref
S  = feedback(1, L);                       % sensitivity  e / theta_ref

fprintf('Kp = %.1f V/rad, tau = %.3f s\n', D.Kp, D.tau);
fprintf('Velocity error constant Kv = lim s*L(s) = %.3f 1/s  (target 1/tp = %.3f)\n', ...
    dcgain(minreal(s*L)), 1/D.tp);

%% 1. Stability: closed-loop poles, Routh-free check, margins
disp('Closed-loop poles:'); disp(pole(T));
fprintf('All poles in LHP: %d\n', isstable(T));
damp(T)
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Gain margin  = %.1f dB at %.1f rad/s\n', 20*log10(Gm), Wcg);
fprintf('Phase margin = %.1f deg at %.2f rad/s (crossover)\n', Pm, Wcp);

figure(1); margin(L); grid on;
title('Loop L(s) = G_c(s) G_p(s): gain and phase margins');
print(1,'-dpng','-r200','figs/fig_margins.png');

%% 2. Does the loop really behave like a 0.5 s delay?
% Ramp input (constant line-of-sight rate) -> output should lag by exactly tp
t  = (0:0.001:6)';
w  = 0.1;                                  % 0.1 rad/s ramp (typical LOS rate)
y  = lsim(T, w*t, t);
lag = (w*t(end) - y(end))/w;               % steady-state time lag [s]
fprintf('Steady-state ramp lag = %.3f s (design: tp = %.3f s)\n', lag, D.tp);

figure(2);
plot(t, w*t, 'k--', t, y, 'b', t, w*(t - D.tp).*(t>D.tp), 'r:', 'LineWidth',1.4); grid on;
xlabel('Time [s]'); ylabel('\theta [rad]');
legend('reference \theta_{ref} = \omega t','turret output','reference delayed by t_p','Location','northwest');
title('Ramp tracking: the closed loop lags by exactly t_p');
print(2,'-dpng','-r200','figs/fig_ramp_lag.png');

% Compare T(s) with an ideal delay e^{-s tp} in the frequency domain
[nd, dd] = pade(D.tp, 6);  Tdelay = tf(nd, dd);   % accurate delay approximation
figure(3); bode(T, Tdelay, '--', {0.1, 100}); grid on;
legend('closed loop T(s)', 'ideal delay e^{-s t_p}','Location','southwest');
title('Closed loop vs pure 0.5 s delay (match at low frequency)');
print(3,'-dpng','-r200','figs/fig_T_vs_delay.png');

%% 3. Transient response to an acquisition step (initial pointing error)
figure(4); step(T, 4); grid on; title('Closed-loop step response \theta/\theta_{ref}');
print(4,'-dpng','-r200','figs/fig_cl_step.png');
si = stepinfo(T); fprintf('Step: rise %.2f s, settling %.2f s, overshoot %.1f %%\n', ...
    si.RiseTime, si.SettlingTime, si.Overshoot);

%% 4. Noise: how much radar noise reaches the turret?
% Elevation noise 0.02 deg -> the reference noise is filtered by the estimator
% and then by T(s). Bandwidth of T is the key quantity.
bw = bandwidth(T);
fprintf('Closed-loop bandwidth = %.2f rad/s (resonance at 55.9 rad/s, radar Nyquist 62.8 rad/s)\n', bw);
fprintf('|T| at the 55.9 rad/s plant resonance = %.1f dB\n', 20*log10(abs(evalfr(T, 1j*55.9))));