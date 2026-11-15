%% STEP 6 - Generalization test and ablation study (validation of the design idea)
% (1) GENERALIZATION: the frozen design (no retuning) is run on N random
%     admissible cases: random start range/elevation, speed, heading, x-y
%     maneuver (|a| <= 19 m/s^2, 0.4-4 rad/s, onset 1-4 s) and blackout
%     (onset 1-5 s, length 0.1-0.5 s). Every case passes the envelope check.
% (2) ABLATION: remove one ingredient at a time to show WHY the design works:
%     - no prediction      (reference = current estimate, same slow loop)
%     - fast loop          (Kv = 10 1/s: lag 0.1 s, still predicting 0.5 s)
%     - fast loop, no pred (conventional "track the current target tightly")

clear; close all; clc;
if ~exist('figs','dir'), mkdir figs; end
D  = design_params();  P = turret_params();
N  = 50;                                   % number of random cases

%% 1. Generalization over random cases
res = nan(N, 5);   % [intercepted, t_first, miss_first, hit_ratio, n_shots]
for c = 1:N
    L = simulate_engagement('R', D, struct('seed', c));
    res(c,:) = score_engagement(L);
end
fprintf('GENERALIZATION over %d random admissible cases (frozen design):\n', N);
fprintf('  interception success : %d / %d (%.0f %%)\n', sum(res(:,1)), N, 100*mean(res(:,1)));
fprintf('  first hit time [s]   : mean %.2f | median %.2f | worst %.2f\n', ...
    mean(res(:,2),'omitnan'), median(res(:,2),'omitnan'), max(res(:,2)));
fprintf('  miss of first hit [m]: mean %.2f | worst %.2f   (limit 2.00)\n', ...
    mean(res(:,3),'omitnan'), max(res(:,3)));
fprintf('  hit ratio            : mean %.2f | worst %.2f\n', mean(res(:,4),'omitnan'), min(res(:,4)));
fail = find(res(:,1) == 0);
if ~isempty(fail), fprintf('  NOT intercepted: cases %s\n', mat2str(fail')); end

figure(1); clf;
subplot(1,2,1); histogram(res(:,2), 0:0.5:10); grid on;
xlabel('time of first hit [s]'); ylabel('number of cases'); title(sprintf('%d random cases', N));
subplot(1,2,2); plot(res(:,2), res(:,3), 'o'); grid on; yline(D.Reng,'r--');
xlabel('time of first hit [s]'); ylabel('miss of first hit [m]'); title('Accuracy of first hit');
print(1,'-dpng','-r200','figs/fig_generalization.png');

% Hardest case (latest first hit, or a failure) for the paper
[~, worst] = max(res(:,2) + 100*(res(:,1)==0));
L = simulate_engagement('R', D, struct('seed', worst));
figure(2); clf;
sh = L.shots; hit = sh(:,3)==1;
subplot(2,1,1); plot(L.t, rad2deg(L.th_true),'k', L.t, rad2deg(L.th),'b','LineWidth',1.2); hold on; grid on;
plot([NaN; sh(hit,1)], [NaN; rad2deg(L.th(round(sh(hit,1)/1e-3)+1))], 'g^','MarkerFaceColor','g');
plot([NaN; sh(~hit,1)],[NaN; rad2deg(L.th(round(sh(~hit,1)/1e-3)+1))], 'rv','MarkerFaceColor','r');
ylabel('\theta [deg]'); legend('target','turret','hit','miss','Location','best');
title(sprintf('Hardest random case (#%d)', worst));
subplot(2,1,2); plot(L.t, 1e3*(L.th - L.th_true),'b'); hold on; grid on; ylim([-10 10]);
plot(L.t, 1e3*D.Reng./L.r_true,'k:', L.t, -1e3*D.Reng./L.r_true,'k:');
ylabel('pointing err [mrad]'); xlabel('Time [s]');
print(2,'-dpng','-r200','figs/fig_hardest_case.png');

%% 2. Ablation study
Kv_fast = 10;  tau_fast = 1/(2*Kv_fast);            % zeta = 0.707 again, lag 0.1 s
V = struct('name',{},'D',{});
V(1).name = 'full design (lag-matched + predict)';  V(1).D = D;
Dv = D; Dv.tp = 0;                                    V(2).name = 'no prediction (same loop)';  V(2).D = Dv;
Dv = D; Dv.Kp = Kv_fast/P.K0; Dv.tau = tau_fast;      V(3).name = 'fast loop + 0.5 s predict';  V(3).D = Dv;
Dv.tp = 0;                                            V(4).name = 'fast loop, no prediction';   V(4).D = Dv;

cases = [repmat({'B'},1,1), {'C','E'}];               % named scenarios
Nr = 20;                                              % + first Nr random cases
fprintf('\nABLATION (scenarios B, C, E + %d random cases):\n', Nr);
fprintf(' %-38s | intercept | mean 1st hit [s] | hit ratio | RMS point err [mrad]\n','variant');
for v = 1:numel(V)
    R = [];  pe = [];
    for c = 1:numel(cases)
        L = simulate_engagement(cases{c}, V(v).D);  R(end+1,:) = score_engagement(L);  %#ok<AGROW>
        pe(end+1) = rmsv(L.th(L.t>=3) - L.th_true(L.t>=3));                 %#ok<AGROW>
    end
    for c = 1:Nr
        L = simulate_engagement('R', V(v).D, struct('seed', c));  R(end+1,:) = score_engagement(L); %#ok<AGROW>
        pe(end+1) = rmsv(L.th(L.t>=3) - L.th_true(L.t>=3));                             %#ok<AGROW>
    end
    fprintf(' %-38s |   %5.2f   |      %5.2f       |   %4.2f    |   %6.2f\n', V(v).name, ...
        mean(R(:,1)), mean(R(:,2),'omitnan'), mean(R(:,4),'omitnan'), 1e3*mean(pe));
end