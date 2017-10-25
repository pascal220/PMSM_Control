% Electric circuit
p = 6;            % Number of poles of the ac motor
R = 378e-3;       % Stator Resistance - Ohm
Ld = 3.427e-3;    % Inductance in the other H
Lq = 3.334e-3;    % Inductance in stator H
Ke = 0.0169;      % back emf

% Mechanical crank shaft
Jt = 57e-4;       % Total inertia
Kt = 191.3;       % Torque constant
ba = 3e-4;        % viscous damping
Tl = 1e-4;        % Torque load

% Kinematics 
l = 80;           % length of the rod
r = 22.5;         % radius of crank
q = 9;            % ofset

% Double mass
M = 10;           % Mass of the load
Jm = 6.375e-04;   % Inertia of the motor
ks = 48.75;       % spring stiffnes
kb = 4.875;       % joint stiffnes
bl = 0.001;       % damping in the spring