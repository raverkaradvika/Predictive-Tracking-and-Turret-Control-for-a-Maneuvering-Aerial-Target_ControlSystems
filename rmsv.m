function y = rmsv(x)
% RMSV  Root-mean-square ignoring NaNs (toolbox-free, works in MATLAB and Octave).
x = x(~isnan(x));
y = sqrt(mean(x.^2));
end