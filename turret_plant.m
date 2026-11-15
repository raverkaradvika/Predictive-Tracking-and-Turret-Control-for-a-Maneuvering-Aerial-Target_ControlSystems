function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
% TURRET_PLANT  First-principles elevation-drive dynamics (required signature).
%
% States : theta_turret [rad], theta_dot_turret [rad/s], i [A]
% Input  : V_m  motor armature voltage [V]
% Returns: the time derivatives of the three states.
%
% Equations (all referred to the turret side of the 100:1 gearbox):
%   electrical : Lm*di/dt = V_m - Rm*i - Ke*(N*theta_dot)   (back-emf uses MOTOR speed)
%   mechanical : Jeq*theta_ddot = N*Kt*i - b*theta_dot      (gear multiplies torque by N)
%
% Assumptions: ideal rigid lossless gear (no backlash or compliance), balanced
% trunnion (no gravity torque), no Coulomb friction, no voltage saturation.
% With these the model is already LINEAR, so the "small-perturbation"
% model is exact everywhere except at the hard stops at -10 and 85 deg.
% The stops are applied in the simulation loop (clamp theta and set the
% outward velocity to zero), not in this derivative function.

persistent P
if isempty(P), P = turret_params(); end   % load parameters once (faster)

% theta_turret and t are unused because there is no gravity or
% time-varying term. They stay in the argument list to match the required signature.

% Electrical loop: applied voltage minus resistive drop minus back-emf
i_dot = (V_m - P.Rm*i - P.Ke*P.N*theta_dot_turret) / P.Lm;

% Mechanical: geared motor torque minus viscous damping, over total inertia
theta_ddot_turret = (P.N*P.Kt*i - P.b*theta_dot_turret) / P.Jeq;

% First state derivative is just the angular velocity (passed straight through)
theta_dot_turret = theta_dot_turret;
end