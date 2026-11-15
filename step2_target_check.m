%% STEP 2 - Check the target generator: envelope limits + radar measurements
% Verifies 800 <= r <= 2000 m, |v| <= 150 m/s, |a| <= 20 m/s^2 for every
% scenario, and plots truth vs noisy radar samples (with the blackout).

clear; close all; clc;
if ~exist('figs','dir'), mkdir figs; end
scen  = {'A','B','C','D','E'};
tend  = 10;
t     = (0:0.001:tend)';        % fine grid for truth
Ts    = 0.05;
tk    = (0:Ts:tend)';           % radar sample instants

figure(1); clf; hold on; grid on; axis equal;
for n = 1:numel(scen)
    sc = scen{n};
    % ---- truth on fine grid ----
    X = zeros(numel(t),8);
    for j = 1:numel(t)
        [~,~,~,T] = target_motion_generator(t(j), sc);
        X(j,:) = [T.x T.y T.vx T.vy T.ax T.ay T.r T.th];
    end
    spd = hypot(X(:,3),X(:,4));  acc = hypot(X(:,5),X(:,6));
    fprintf('Scenario %s: r in [%.0f, %.0f] m | max|v| = %.1f m/s | max|a| = %.1f m/s^2 | el in [%.1f, %.1f] deg\n', ...
        sc, min(X(:,7)), max(X(:,7)), max(spd), max(acc), rad2deg(min(X(:,8))), rad2deg(max(X(:,8))));
    % Envelope enforcement: stop with an error if any limit is violated
    assert(min(X(:,7))>=800 && max(X(:,7))<=2000, 'Range limit violated');
    assert(max(spd)<=150, 'Speed limit violated');
    assert(max(acc)<=20,  'Acceleration limit violated');

    ls = {'-','--',':','-.','-'};   % different line styles: A/C and B/D share a path
    figure(1); plot(X(:,1), X(:,2), ls{n}, 'LineWidth',1.5+0.6*(n>2), 'DisplayName',['Scenario ' sc]);

    % ---- radar samples (only for C, to show the blackout) ----
    if sc == 'C'
        M = zeros(numel(tk),3);
        for j = 1:numel(tk)
            [M(j,1), M(j,2), M(j,3)] = target_motion_generator(tk(j), sc);
        end
        figure(2); clf;
        subplot(2,1,1);
        plot(t, X(:,7), 'k', tk, M(:,1), '.', 'MarkerSize',8); grid on;
        ylabel('r [m]'); legend('truth','radar','Location','best');
        title('Scenario C: radar measurements (gap = blackout 1.0-1.5 s)');
        subplot(2,1,2);
        plot(t, rad2deg(X(:,8)), 'k', tk, rad2deg(M(:,2)), '.', 'MarkerSize',8); grid on;
        xlabel('Time [s]'); ylabel('\theta [deg]');
        print(2,'-dpng','-r200','figs/fig_radar_C.png');
        fprintf('Scenario C: %d of %d radar samples missing (expected 10)\n', sum(M(:,3)==0), numel(tk));
    end
end
figure(1); plot(0,0,'k^','MarkerFaceColor','k','DisplayName','Turret');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best');
title('Target trajectories in the elevation plane (0-10 s)');
print(1,'-dpng','-r200','figs/fig_trajectories.png');

% Zoomed noise check: radar error statistics on scenario A
k = (0:2000)'; e = zeros(numel(k),2);
for j = 1:numel(k)
    [rm, thm, ~, T] = target_motion_generator(k(j)*Ts, 'A');
    e(j,:) = [rm - T.r, thm - T.th];
end
fprintf('Radar noise check: std(r err) = %.3f m (1.0), std(th err) = %.4f deg (0.02)\n', ...
    std(e(:,1)), rad2deg(std(e(:,2))));