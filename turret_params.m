function P = turret_params()
% TURRET_PARAMS  Physical parameters of the Guardian elevation drive
% (ME319 Assignment 2, Section 2). Every other file reads parameters from
% here, so a value only ever has to be changed in one place.

% ---------------- given data ----------------
P.Jt  = 1200;    % turret + cradle inertia about the trunnion   [kg m^2]
P.Jm  = 0.20;    % motor rotor inertia (on the fast motor side)   [kg m^2]
P.N   = 100;     % gear ratio: motor speed = N * turret speed     [-]
P.b   = 1500;    % viscous damping on the turret side             [N m s/rad]
P.Kt  = 10;      % motor torque constant                          [N m/A]
P.Ke  = 10;      % motor back-emf constant                        [V s/rad]
P.Rm  = 1.0;     % armature resistance                            [ohm]
P.Lm  = 0.10;    % armature inductance                            [H]

% ---------------- derived quantities ----------------
% The rotor spins N times faster than the turret, so its kinetic energy is
% 1/2*Jm*(N*w)^2 = 1/2*(N^2*Jm)*w^2. Seen from the turret it therefore acts
% like an inertia N^2*Jm = 2000 kg m^2, which is larger than the turret itself.
P.Jeq = P.Jt + P.N^2*P.Jm;          % total inertia at turret = 3200 kg m^2

% Low-frequency plant gain: Gp(s) ~ K0/s, i.e. steady turret speed per volt.
P.K0  = P.N*P.Kt / (P.Rm*P.b + P.N^2*P.Kt*P.Ke);   % ~ 9.985e-4 (rad/s)/V

% Mechanical elevation stops
P.th_min = deg2rad(-10);            % [rad]
P.th_max = deg2rad(85);             % [rad]
end