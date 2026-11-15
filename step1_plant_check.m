%% STEP 1 - Turret elevation plant: derivation check and analysis
% Run this from the folder that contains turret_params.m and turret_plant.m.
% It (a) builds Gp(s) from the physical equations, (b) analyses its
% poles/zeros/Bode, and (c) checks that turret_plant.m (used later in the
% full simulation) behaves the same as the transfer function.

clear; close all; clc;
P = turret_params();
if ~exist('figs','dir'), mkdir figs; end
s = tf('s');

%% 1. Transfer function directly from the derived formula
%  theta(s)/V(s) = N*Kt / ( s * [ (Lm s + Rm)(Jeq s + b) + N^2 Kt Ke ] )
Gp = P.N*P.Kt / ( s*((P.Lm*s + P.Rm)*(P.Jeq*s + P.b) + P.N^2*P.Kt*P.Ke) );
Gp = minreal(Gp);
disp('Gp(s) = theta_turret / V_m :'); Gp

%% 2. Same plant from the state equations, as an independent cross-check
%  x = [theta; theta_dot; i]
A = [0, 1,                     0;
    0, -P.b/P.Jeq,            P.N*P.Kt/P.Jeq;
    0, -P.N*P.Ke/P.Lm,        -P.Rm/P.Lm];
B = [0; 0; 1/P.Lm];
C = [1 0 0];  D = 0;
Gss = ss(A,B,C,D);
fprintf('Max |Bode| mismatch TF vs SS: %.2e  (should be ~0)\n', ...
    max(abs(squeeze(freqresp(Gp - tf(Gss), logspace(-1,3,200))))));

%% 3. Poles, zeros, natural frequency and damping
disp('Poles of Gp:'); disp(pole(Gp));
disp('Zeros of Gp:'); disp(zero(Gp));
damp(Gp)

% Analytical value of the complex pair: closer look at the quadratic factor
den2 = conv([P.Lm P.Rm],[P.Jeq P.b]); den2(end) = den2(end) + P.N^2*P.Kt*P.Ke;
wn = sqrt(den2(3)/den2(1));            % natural frequency of the e-m mode
zeta = den2(2)/(2*sqrt(den2(1)*den2(3)));
fprintf('Quadratic factor: %.1f s^2 + %.1f s + %.1f\n', den2);
fprintf('Electromechanical mode: wn = %.2f rad/s (%.2f Hz), zeta = %.3f\n', wn, wn/2/pi, zeta);
fprintf('Low-frequency gain K0 = %.4e (rad/s)/V  ->  Gp ~ K0/s\n', P.K0);

% How much voltage the fastest admissible target needs (sanity check)
w_req = 150/800;                       % worst LOS rate: v_max / r_min [rad/s]
fprintf('Worst-case LOS rate %.3f rad/s needs ~%.0f V steady-state\n', w_req, w_req/P.K0);

%% 4. Plots for the paper
figure(1); pzmap(Gp); grid on; title('Pole-zero map of G_p(s)');
print(1,'-dpng','-r200','figs/fig_pzmap.png');

figure(2);
bode(Gp, P.K0/s, '--', {1e-1, 1e3}); grid on;
legend('G_p(s) full model', 'K_0/s low-frequency approximation','Location','southwest');
title('Bode plot of turret plant');
print(2,'-dpng','-r200','figs/fig_bode_plant.png');

% Velocity response to a 100 V step: shows the lightly damped ~56 rad/s ringing
figure(3);
step(100*Gp*s, 0.5); grid on;
ylabel('\theta-dot_{turret} [rad/s]'); title('Turret speed for a 100 V voltage step');
print(3,'-dpng','-r200','figs/fig_speed_step.png');

%% 5. Verify turret_plant.m against the transfer function
V0   = 100;                            % test voltage step [V]
tEnd = 1.0;
rhs  = @(t,x) plant_rhs(t, x, V0);     % wrapper (local function at file end)
opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
% give ode45 a uniform time vector -> it still integrates adaptively,
% but reports the solution at these evenly spaced points (lsim needs that)
[t_ode, x_ode] = ode45(rhs, 0:1e-3:tEnd, [0;0;0], opts);

y_tf = lsim(Gp, V0*ones(size(t_ode)), t_ode);  % same input through Gp(s)
fprintf('Max |theta_ode - theta_tf| = %.2e rad  (should be ~0)\n', max(abs(x_ode(:,1)-y_tf)));

figure(4);
plot(t_ode, rad2deg(x_ode(:,1)), 'LineWidth',1.5); hold on;
plot(t_ode, rad2deg(y_tf), '--', 'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('\theta_{turret} [deg]');
legend('turret\_plant.m + ode45','G_p(s) via lsim','Location','northwest');
title('Plant function verification (100 V step)');
print(4,'-dpng','-r200','figs/fig_plant_verify.png');

%% ---- local function: packs the 3 outputs of turret_plant into a vector ----
function dx = plant_rhs(t, x, V)
[th_d, th_dd, i_d] = turret_plant(t, V, x(1), x(2), x(3));
dx = [th_d; th_dd; i_d];
end