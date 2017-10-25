%% System parameters
syms Lq Ld R p Ke                                                           % Electrical parameters
syms Jm Jg Jc bm bg Tr Kt                                                   % Mechanical paremters 
syms r l q m k d                                                            % crank-shaft paremters 

%% System states
syms id iq theta d_theta % A rad rad/s
% System input
syms Vd Vq % V

%% Dynamics equations
Jt = Jm+Jg+(Jc/Tr^2);                                                       % Total Inertia

d_id = (1/Ld)*Vd - (R/Ld)*id - (Lq/Ld)*(p/2)*d_theta*iq;                    % dynamics of current in d-axis Vd = R*id + d_id*Ld + Lq*p/2*d_theta*iq
d_iq = (1/Lq)*Vq - (R/Lq)*iq + (Ld/Lq)*(p/2)*d_theta*id...                  % dynamics of current in q-axis Vq = R*iq + d_iq*Lq - Ld*p/2*d_theta*id + Ke*d_theta
        - (Ke/Lq)*d_theta;                                                  % back-EMF 
dd_theta = (1.5*p*(Ld-Lq)*iq*id)/Jt + (Kt/Jt)*iq...                         % Electromechanical torque
            - (bm/Jt)*d_theta...                                            % Dumping in motor
            - (bg/Jt)*d_theta;                                              % Dumping in gearbox

%% Kinematic equations
x   = (l*sqrt(1 - ((r*sin(theta)-q)/l)^2) - r*cos(theta));                  % Angle of shaft to linear postion                   
 
d_x = (sin(theta)/10 - (cos(theta)*(sin(theta)/3 - 1/6))/...
      (10*(1 - (sin(theta)/3 - 1/6)^2)^(1/2)))*d_theta;                     % Angluar velocity of shaft to linear velocity

dd_x = (d_theta*(cos(theta)/10 - cos(theta)^2/(30*(1 - (sin(theta)/3 - 1/6)^2)^(1/2))...
        + (sin(theta)*(sin(theta)/3 - 1/6))/(10*(1 - (sin(theta)/3 - 1/6)^2)^(1/2))...
        - (cos(theta)^2*(sin(theta)/3 - 1/6)^2)/(30*(1 - (sin(theta)/3 - 1/6)^2)^(3/2)))+sin(theta)/10 ...
        - (cos(theta)*(sin(theta)/3 - 1/6))/(10*(1 - (sin(theta)/3 - 1/6)^2)^(1/2)))*dd_theta; % Angluar acceleration of shaft to linear velocity
   
F = x*k + d_x*d + dd_x*m;

%% Nonlinear function
f = [d_id; d_iq; d_theta; dd_theta];                                        % d_x = f(f)
g = [x; d_x; dd_x; F];                                                      %   y = g(x)

%% Linearisation 
A = jacobian(f,[id iq theta d_theta]);                                      % A matrix, gives behavior of the system
B = jacobian(f, [Vd Vq]);                                                   % B input matrix
C = jacobian(g,[id iq theta d_theta]);                                      % C output matrix
D = jacobian(g, [Vd Vq]);                                                   % D feed-through matrix

%% Substituting operating points
A = subs(A,[id iq theta d_theta],[0 0 0 0]);                            % Orgin is the best equilibrium point (always)
C = subs(C,[id iq theta d_theta],[0 0 0 0]);                            % Orgin is the best equilibrium point (always)

%% Known motor parameters and gearbox
% Ufac kolegom ze dobrze zrobili robote. 
% Jm = 11.472;        % Inertia of AC motor - kgm^2
% p = 6;              % Number of poles of the ac motor
% R = 0.378;          % Stator Resistance - Ohm
% Tr = 27.5;          % Transmision ratio of the gearbox 
% lambda = 0.11;      % Flux of motor
% Ld = 3.334e-3;      % Inductance in q-axis - H
% Lq = 3.427e-3;      % Inductance in d-axis - H
% Ke = 1.131;         % EMF constant - V/(rad/s)
% Kt = 1.39;          % Torque constant - Nm/Arms
% bm = 1.2313;        % Viscous damping(friction) coefficient - Nm
% bg = 0.6290;        % Internal damping of gearbox 
% Jg = 0.00007;       % Inertia of gearbox - kgm^2
% Jc = 0.02356;       % Inertia of crank - kgm^2
% 
% id = 0;         %Current in the d-axis of the motor (formally a state)
% Vd = 0;         % Voltage in the d-axis of the motor (formally an input)
% 
% % Known crank-shaft parameters
% r = 0.225;   % Radius of shaft - m
% l = 0.9;     % length of conecting rod - m
% q = 0.009;    % offset - m
% m = 1816;     % mass on top - g
% 
% k = 205;
% d = 0.1;
% 
% c1_v = 0.9624;
% c2_v = 107.0132;
% c3_v = 1;
% c4_v = -874.6339;
% 
% c1_i = 27.9175; 
% c2_i = 8618;
% c3_i = 1;
% c4_i = 137.3771;

% Substituting parameters
A = subs(A,[Jm Jg Jc bm bg Ke Kt Tr R p Ld Lq],[1.1472 0.07 23.56 0.01 0.01 0.0778 1.39 27.5 0.378 6 3.334e-3 3.427e-3]);                            
B = subs(B,[Ld Lq],[3.334e-3 3.427e-3]);
C = subs(C,[Jm Jg Jc bm bg Kt Tr r l q m k d],[1.1472 0.07 23.56 0.01 0.01 1.39 27.5 0.225 0.9 0.009 1816 205 0.1]);                    

%% Converstion form symbolic to type double
A = double(A);
B = double(B);
C = double(C);
D = double(D);

% Segregation
C1 = C(1,:);
C2 = C(2,:);
C3 = C(3,:);
C4 = C(4,:);

%% Analysis
sys = ss(A,B,C,D)                                                           % Full system
sys1 = ss(A,B,C1,0);                                                        % Position
sys2 = ss(A,B,C2,0);                                                        % Velocity
sys3 = ss(A,B,C3,0);                                                        % Acceleration
sys4 = ss(A,B,C4,0);                                                        % Force


eig(sys)                                                                    % Eigen values/ poles/ eigen frequencies/ resonance frequancies
rank(ctrb(sys))                                                             % Controlability
rank(obsv(sys))                                                             % Observability

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';

% Bode plot of the system
figure (1)
subplot(4,1,1)
bode(sys1,opts)
title('Position')
subplot(4,1,2)
bode(sys2,opts)
title('Velocity')
subplot(4,1,3)
bode(sys3,opts)
title('Acceleration')
subplot(4,1,4)
bode(sys4,opts)
title('Force')
hold on 

% Step response of the system
figure(2)
subplot(4,1,1)
step(sys1)
title('Position')
subplot(4,1,2)
step(sys2)
title('Velocity')
subplot(4,1,3)
step(sys3)
title('Acceleration')
subplot(4,1,4)
step(sys4)
title('Force')

%% Pole placment control
% % Position control
% zeta1 = 1;
% wn1 = 0.1;
% other1 = -3.06;
% 
% P1 = [(-zeta1+sqrt(zeta1^2-1))*wn1; (-zeta1-sqrt(zeta1^2-1))*wn1-(wn1*0.1); other1; other1+1];
% K1 = place(A,B(:,2),P1);
% sys_cl1 = ss(A-B*K1,B(:,2),C,D);
% 
% % Force control
% zeta4 = 0.99;
% wn4 = 49.8;
% other4 = -49.9;
% 
% P4 = [(-zeta4+sqrt(zeta4^2-1))*wn4; (-zeta4-sqrt(zeta4^2-1))*wn4; other4; other4+1];
% K4 = place(A,B(:,2),P4);
% sys_cl4 = ss(A-B(:,2)*K4,B,C,D);
% 
% % LQR optimal control
% R = 1;
% Q = C'*C;
% K_lqr = lqr(A,B,Q,R);
% sys_lqr = ss(A-B*K_lqr,B,C,D);
% 
% 
% figure (3)
% step(sys_cl1)
% title('Position')
% 
% figure (4)
% step(sys_cl4)
% title('Force')
% 
% figure (5)
% step(sys_lqr)
% title('LQR')
% 
%% Transformation to Transfer Function 
% [num,den] = ss2tf(A,B,C,D);
% 
% H_sys = tf({num(1,:);num(2,:);num(3,:);num(4,:)},den);
