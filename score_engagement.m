function s = score_engagement(L)
% SCORE_ENGAGEMENT  Summarise one simulated engagement:
% [intercepted, time of first hit, miss of first hit, hit ratio, shots]
sh = L.shots;  hit = sh(:,3) == 1;
if any(hit)
    j = find(hit, 1);
    s = [1, sh(j,1), sh(j,2), mean(hit), size(sh,1)];
else
    hr = NaN; if ~isempty(sh), hr = 0; end
    s = [0, NaN, NaN, hr, size(sh,1)];
end
end