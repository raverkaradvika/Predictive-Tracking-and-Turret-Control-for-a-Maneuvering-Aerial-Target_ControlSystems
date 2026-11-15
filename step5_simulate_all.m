%% STEP 5 - Full closed-loop simulation of all scenarios + performance metrics
% Uses simulate_engagement (RK4, 1 ms) with the finalized design (no retuning).
% Produces: per-scenario plots, a metrics table, a Monte-Carlo summary over
% radar-noise seeds, and the firing-threshold (kappa) comparison for Idea 5.

clear; close all; clc;
D = design_params();
if ~exist('figs','dir'), mkdir figs; end
scen = {'A','B','C','D','E'};

%% 1. Nominal run (seed 319) - plots + metrics table
fprintf('Scen | shots | hits | first hit [s] | hit ratio | RMS point err t>3s [mrad] | mean miss [m] | max|V| [V]\n');
for n = 1:numel(scen)
    L = simulate_engagement(scen{n}, D);
    sh = L.shots;  hit = sh(:,3) == 1;
    t_first = NaN;  if any(hit), t_first = sh(find(hit,1),1); end
    e = L.th - L.th_true;  m = L.t >= 3;
    fprintf('  %s  |  %2d   |  %2d  |     %5.2f     |   %4.2f    |          %5.2f            |     %4.2f      |   %4.0f\n', ...
        scen{n}, size(sh,1), sum(hit), t_first, mean(hit), 1e3*rmsv(e(m)), mean(sh(:,2)), max(abs(L.V)));

    figure(n); clf;
    subplot(3,1,1);
    plot(L.t, rad2deg(L.th_true), 'k', L.t, rad2deg(L.th), 'b', 'LineWidth',1.2); hold on; grid on;
    % hit / miss markers (NaN dummy point keeps the legend valid if a set is empty)
    ih = round(sh(hit,1)/1e-3)+1;   im = round(sh(~hit,1)/1e-3)+1;
    plot([NaN; sh(hit,1)],  [NaN; rad2deg(L.th(ih))], 'g^', 'MarkerFaceColor','g');
    plot([NaN; sh(~hit,1)], [NaN; rad2deg(L.th(im))], 'rv', 'MarkerFaceColor','r');
    ylabel('\theta [deg]'); legend('target (true)','turret','hit','miss','Location','southeast');
    title(['Scenario ' scen{n}]);
    subplot(3,1,2);
    plot(L.t, 1e3*e, 'b', 'LineWidth',1.0); grid on; ylim([-10 10]);
    hold on; plot(L.t, 1e3*D.Reng./L.r_true, 'k:', L.t, -1e3*D.Reng./L.r_true, 'k:');
    ylabel('pointing err [mrad]'); legend('\theta_{turret} - \theta_{target}','\pm R_{eng}/r (hit band)');
    subplot(3,1,3);
    plot(L.t, L.V, 'LineWidth',1.0); grid on; ylabel('V_m [V]'); xlabel('Time [s]');
    print(n,'-dpng','-r200',sprintf('figs/fig_sim_%s.png', scen{n}));
end

%% 2. Monte Carlo over radar-noise seeds (consistency)
seeds = 1:20;
fprintf('\nMonte Carlo over %d noise seeds:\n Scen | intercept rate | mean first hit [s] | worst first hit [s] | mean hit ratio\n', numel(seeds));
for n = 1:numel(scen)
    tf1 = nan(size(seeds)); hr = nan(size(seeds));
    for k = 1:numel(seeds)
        L = simulate_engagement(scen{n}, D, struct('seed', seeds(k)));
        hit = L.shots(:,3) == 1;
        if any(hit), tf1(k) = L.shots(find(hit,1),1); end
        hr(k) = mean(hit);
    end
    fprintf('  %s  |     %5.2f      |       %5.2f        |       %5.2f         |    %4.2f\n', ...
        scen{n}, mean(~isnan(tf1)), mean(tf1,'omitnan'), max(tf1), mean(hr,'omitnan'));
end

%% 3. Idea 5: firing threshold kappa (fire if predicted miss <= kappa*Reng)
kap = [0.5 0.75 1.0];  seeds = 1:10;
fprintf('\nFiring threshold study (scenarios B and E, %d seeds):\n kappa | mean first hit B | hit ratio B | mean first hit E | hit ratio E\n', numel(seeds));
for kk = kap
    Dk = D;  Dk.kappa = kk;  row = [];
    for sc = {'B','E'}
        tf1 = nan(size(seeds)); hr = nan(size(seeds));
        for k = 1:numel(seeds)
            L = simulate_engagement(sc{1}, Dk, struct('seed', seeds(k)));
            hit = L.shots(:,3) == 1;
            if any(hit), tf1(k) = L.shots(find(hit,1),1); end
            hr(k) = mean(hit);
        end
        row = [row, mean(tf1,'omitnan'), mean(hr,'omitnan')]; %#ok<AGROW>
    end
    fprintf('  %.2f |      %5.2f       |    %4.2f     |      %5.2f       |    %4.2f\n', kk, row);
end