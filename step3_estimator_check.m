%% STEP 3 - Estimator + predictor check (open loop, no turret yet)
% Runs the estimator on the radar data of every scenario and compares
% (a) the CURRENT estimate and (b) the 0.5 s PREDICTION with the truth.
% Also sweeps the memory factor theta to justify the chosen value.

clear; close all; clc;
if ~exist('figs','dir'), mkdir figs; end
D    = design_params();
scen = {'A','B','C','D','E'};
tk   = (0:D.Ts:10)';

% ---------------- main run with the finalized design ----------------
fprintf('Scen | RMS pos err now [m] | RMS 0.5s-pred err [m] | RMS el-ref err [mrad] | max pred err [m]\n');
for n = 1:numel(scen)
    [R{n}] = run_estimator_openloop(scen{n}, D, tk); %#ok<SAGROW>
    m = tk >= 2;   % statistics after start-up transient
    fprintf('  %s  |      %6.2f         |       %6.2f          |        %6.2f          |   %6.2f\n', ...
        scen{n}, rmsv(R{n}.e_now(m)), rmsv(R{n}.e_pred(m)), 1e3*rmsv(R{n}.e_el(m)), max(R{n}.e_pred(m)));
end

% ---------------- figure: errors vs time ----------------
figure(1); clf;
subplot(2,1,1); hold on; grid on;
for n = 1:numel(scen), plot(tk, R{n}.e_pred, 'LineWidth',1.2); end
ylabel('|pred. pos. error| [m]'); legend(scen); 
title('0.5 s prediction error (maneuver starts / blackout at 1.0 s)');
subplot(2,1,2); hold on; grid on;
for n = 1:numel(scen), plot(tk, 1e3*R{n}.e_el, 'LineWidth',1.2); end
ylabel('elevation ref. error [mrad]'); xlabel('Time [s]'); ylim([-20 20]);
print(1,'-dpng','-r200','figs/fig_estimator_errors.png');

% ---------------- theta sweep (justification of th_nom), Monte Carlo ----------------
% Averaged over 10 independent radar-noise sequences so the choice does not
% depend on one particular noise realisation.
seeds = 1:10;
ths = [0.75 0.80 0.85 0.88 0.90];
fprintf('\nMemory-factor sweep (mean over %d noise seeds): RMS 0.5 s prediction error after t=2 s [m]\n', numel(seeds));
fprintf(' theta    A     B     C     D     E    | worst\n');
for th = ths
    Dt = D; Dt.th_nom = th; row = zeros(numel(seeds), numel(scen));
    for s = seeds
        for n = 1:numel(scen)
            Rt = run_estimator_openloop(scen{n}, Dt, tk, s); row(s,n) = rmsv(Rt.e_pred(tk>=2));
        end
    end
    mr = mean(row,1);
    fprintf(' %.2f  ', th); fprintf(' %5.2f', mr); fprintf('  | %5.2f\n', max(mr));
end

% ---------------- blackout analysis (gated re-acquire ON vs OFF), Monte Carlo ----------------
% Worst-case coasting drift after a 0.5 s blackout: |Delta a| <= 2*amax = 40 m/s^2
% -> 0.5*40*0.5^2 = 5 m. The gate (5 m) only re-opens the memory if the
% coasted track is really stale.
fprintf('\nBlackout window 1.0-2.5 s (mean over seeds): RMS now-error / RMS 0.5 s-pred error [m]\n');
win = tk >= 1.0 & tk <= 2.5;
for n = [3 4 5]
    a = zeros(numel(seeds),4);
    for s = seeds
        Doff = D; Doff.nu_gate = Inf;               % gate never opens
        Rg = run_estimator_openloop(scen{n}, D,    tk, s);
        Ro = run_estimator_openloop(scen{n}, Doff, tk, s);
        a(s,:) = [rmsv(Rg.e_now(win)) rmsv(Rg.e_pred(win)) rmsv(Ro.e_now(win)) rmsv(Ro.e_pred(win))];
    end
    fprintf(' %s: gate ON %5.2f / %5.2f   |   gate OFF %5.2f / %5.2f\n', scen{n}, mean(a,1));
end