%% Parameter values
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

% PI velocity 
c1_v = 7.813;
c2_v = 4.867;
c3_v = 1;
c4_v = 100.4;

% PI current
c1_i = 80;
c2_i = 1.1017e+04;
c3_i = 1;
c4_i = 263;
%% State-Space of PI velocity
syms c1_v c2_v c3_v c4_v
Api_v = -c4_v/c3_v;
Bpi_v = 1;
Cpi_v = (c3_v*c2_v - c1_v*c4_v)/(c3_v^2);
Dpi_v = c1_v/c3_v;

%% State-Space of PI current
syms c1_i c2_i c3_i c4_i
Api_i = -c4_i/c3_i;
Bpi_i = 1;
Cpi_i = (c3_i*c2_i - c1_i*c4_i)/(c3_i^2);
Dpi_i = c1_i/c3_i; 

%% State-Space
syms Lq Ld R p lambda Ke Vd Vq                                                        
syms Jt b k Tr Tl Kt                                                                                                      
syms id iq theta d_theta 

%current-------------------------------------------------------------------
% nonlinear current equations
d_id = (1/Ld)*Vd - (R/Ld)*id + (Lq/Ld)*p*d_theta*iq;                    
d_iq = (1/Lq)*Vq - (R/Lq)*iq - (Ld/Lq)*p*d_theta*id...
     - ((lambda*p)/Lq)*d_theta;
    
f = [d_id d_iq];                                                                           

%linearisation
A_i = jacobian(f,[id iq]);                                     
B_i = jacobian(f,[Vd Vq]);          
C_i = eye(2);                                
D_i = 0;

%substituting values
A_i = subs(A_i,[id iq d_theta],[0 0 0]);
A_i = subs(A_i,[R p],[0.378 6]);
A_i = subs(A_i,[Ld Lq],[3.334e-3 3.334e-3]); 
B_i = subs(B_i,[Ld Lq],[3.334e-3 3.334e-3]);
A_i = double(A_i);
B_i = double(B_i);

%velocity------------------------------------------------------------------
% nonlinear velocity equations
dd_theta = (1.5*p*((Ld-Lq)*iq*id))/Jt + (Kt/Jt)*iq - (b/Jt)*d_theta;     

f = [d_theta dd_theta];                                                                           

%linearisation
A_vel = jacobian(f,[theta d_theta]);                                     
B_vel = jacobian(f, [id iq]);          
C_vel = eye(2);                                
D_vel = 0;

%substituting values
A_vel = subs(A_vel,[id iq theta d_theta],[0 0 0 0]);
B_vel = subs(B_vel,[id iq theta d_theta],[0 0 0 0]);
A_vel = subs(A_vel,[Ld Lq],[3.427e-3 3.334e-3]);
B_vel = subs(B_vel,[Ld Lq],[3.427e-3 3.334e-3]);
A_vel = subs(A_vel,[Jt b],[0.0057 3.0073e-04]);
B_vel = subs(B_vel,Kt,191.2949);

%% Interconnection between subsystems
% Series interconection PI with current system
Aser_i = [[Api_i, 0;0, Api_i], zeros(size(A_i));
                    B_i*Cpi_i,A_i];
Bser_i = [[Bpi_i 0;0 Bpi_i];
               B_i*Dpi_i];
Cser_i = [[D_i*Cpi_i 0;0 D_i*Cpi_i], C_i];
Dser_i = D_i*Dpi_i;

%current loop
Aloop_i = (Aser_i - Bser_i*eye(2)*Cser_i);
Bloop_i = Bser_i;
Cloop_i = Cser_i;
Dloop_i = Dser_i;

Aloop_i = subs(Aloop_i,[R c3_i],[0.378 1]);
Bloop_i = subs(Bloop_i,c3_i,1);
Aloop_i = subs(Aloop_i,[Lq Ld],[3.334e-3 3.427e-3]);
Bloop_i = subs(Bloop_i,[Lq Ld],[3.334e-3 3.427e-3]);
Aloop_i = subs(Aloop_i,[c1_i c2_i c4_i],[80.3333 1.1017e+04 263.0465]);
Bloop_i = subs(Bloop_i,[c1_i c2_i c4_i],[80.3333 1.1017e+04 263.0465]);

% Velocity loop
% Series interconection PI with velocity system and current loop
Atemp_v = [Aloop_i, zeros(4,2);
          B_vel*Cloop_i, A_vel];
Atemp_v(4,6) = -Ke/Lq;
Btemp_v = [Bloop_i;
          B_vel*Dloop_i];
Ctemp_v = [D_vel*Cloop_i, C_vel];
Dtemp_v = Dloop_i*D_vel;

Aser_v = [Api_v, zeros(1,6);
          Btemp_v(:,2)*Cpi_v, Atemp_v];
Bser_v = [Bpi_v;
          Btemp_v(:,2)*Dpi_v];
Cser_v = [[0; Dtemp_v*Cpi_v], Ctemp_v];
Dser_v = Dpi_v*Dtemp_v;

%velocity loop
Aloop_v = (Aser_v - Bser_v*[0 1]*Cser_v);
Bloop_v = Bser_v;
Cloop_v = Cser_v(2,:);
Dloop_v = Dser_v;

Aloop_v = subs(Aloop_v,[Lq p c3_v],[3.334e-3 6 1]);
Bloop_v = subs(Bloop_v,c3_v,1);
Aloop_v = subs(Aloop_v,[c1_v c2_v c4_v],[7.813 4.8671 100.4]);
Bloop_v = subs(Bloop_v,[c1_v c2_v c4_v],[7.813 4.8671 100.4]);
Aloop_v = subs(Aloop_v,Ke,0.0169);
 
Aloop_v = double(Aloop_v);
Bloop_v = double(Bloop_v);
Cloop_v = double(Cloop_v);
Dloop_v = double(Dloop_v);

% Disturbance = Torque load
B_d = [0; 0; 0; 0; 0; 0; -(1/Jt)];
B_d = subs(B_d,Jt,0.0057);
B_d = double(B_d);

%% Kinematics and Dynamics of a two mass system
syms xl d_xl
syms r l q kb ks bl M Jm   

% Angle of shaft to linear postion    
x  = l*(1- (r/l*sin(theta)-q/l)^2)^(1/2)-r*cos(theta); 

% Angluar velocity of shaft to linear velocity 
d_x = d_theta*(r*sin(theta) + (r*cos(theta)*(q/l - (r*sin(theta))/l))/...
    (1 - (q/l - (r*sin(theta))/l)^2)^(1/2));                                

% Angluar acceleration of shaft to linear acceleration
dd_theta = (1.5*p*((Ld-Lq)*iq*id))/Jm + (Kt/Jm)*iq...
         - (b/Jm)*d_theta...
         - ((b+bl)/Jm)*d_x + (bl/Jm)*d_xl...
         - (kb/Jm)*x + (kb/Jm)*xl; 
     
%equations of moments, of the two mass system
dd_xl = (bl/M)*d_x - (bl/M)*d_xl + (kb/M)*x - ((kb+ks)/M)*xl;
F2 = M*dd_xl;

% Output equations
g = [x; F2];

%The output matrix of the full system
C = jacobian(g,[id iq theta d_theta xl d_xl]);
C = subs(C,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);
C = subs(C, [Jm b p Kt],[6.375e-04 3.0073e-04 6 191.2949]);
C = subs(C, [r l q M kb ks bl],[22.5 80 9 10 48.75 4.875 0.001]);     
C = double(C);
C_GSSA = [zeros(2,3) C];

%The state matrix A of the full system
block1 = [zeros(6,2);0 0];
block2 = [zeros(2,5),[0 0;0 0]];
block3 = [0 1;0 0];
Awhole_pf = [Aloop_v block1;block2 block3];

f2 = [d_theta dd_theta d_xl dd_xl];   

block = jacobian(f2,[id iq theta d_theta xl d_xl]);
block = subs(block,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);
Awhole_pf(6:end,4:end) = block;
A_GSSA = subs(Awhole_pf,[Jm b p Kt],[6.375e-04 3.0073e-04 6 191.2949]);
A_GSSA = subs(A_GSSA,[r l q M kb ks bl],[22.5 80 9 10 48.75 4.875 0.001]);
A_GSSA = double(A_GSSA);

%The input matrix B of the full system
B_GSSA = [Bloop_v*((l*(1 - q^2/l^2)^(1/2))/(q*r));0;0];
B_GSSA = subs(B_GSSA,[r l q],[22.5 80 9]); 
B_GSSA = double(B_GSSA);

%Feedforward term
D_GSSA = zeros(2,1);

%creating the full SS system
sys_moj_open = ss(A_GSSA,B_GSSA,C_GSSA,D_GSSA);

%finding minimal realisation
sys_moj_open  = minreal(sys_moj_open);
sys_moj_open.StateName = {'e_v','e_iq','iq','theta','d_theta','x','d_x'};
sys_moj_open.InputName = {'v_{ref}'};
sys_moj_open.OutputName = {'Position','Force'};

%discritisation fo the system
Ts = 1e-3;
sys_moj_d = c2d(sys_moj_open,Ts,'zoh');
[A_dz,B_dz,C_dz,D_dz] = ssdata(sys_moj_d); 

%% Current loop equations
syms x1 x2 x3 x4 u1 u2

dot_x_i = Aloop_i*[x1;x2;x3;x4] + Bloop_i*[u1;u2];
y_i = Cloop_i*[x1;x2;x3;x4];

%% Velocity loop equations
syms x1 x2 x3 x4 x5 x6 x7 u

dot_x_v = Aloop_v*[x1; x2; x3; x4; x5; x6; x7] + Bloop_v*u  + B_d;
y_v = Cloop_v*[x1; x2; x3; x4; x5; x6; x7];

%% Whole system equations
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 u

dot_x_w = A_GSSA*[x1; x2; x3; x4; x5; x6; x7; x8; x9] + B_GSSA*u;
y_w = C_GSSA*[x1; x2; x3; x4; x5; x6; x7; x8; x9];

%% System alown
% syms id iq theta d_theta xl d_xl Vd Vq 
% syms Lq Ld R p lambda Ke                                                        
% syms Jm b Kt                                                                                                      
% syms r l q kb ks bl M   
% 
% x  = l*(1- (r/l*sin(theta)-q/l)^2)^(1/2)-r*cos(theta);                     
%  
% d_x = d_theta*(r*sin(theta) + (r*cos(theta)*(q/l - (r*sin(theta))/l))/...
%     (1 - (q/l - (r*sin(theta))/l)^2)^(1/2));                               
% 
% d_id = (1/Ld)*Vd - (R/Ld)*id + (Lq/Ld)*p*d_theta*iq;                    
% d_iq = (1/Lq)*Vq - (R/Lq)*iq - (Ld/Lq)*p*d_theta*id...
% - ((lambda*p)/Lq)*d_theta;
% 
% dd_theta = (1.5*p*((Ld-Lq)*iq*id))/Jm + (Kt/Jm)*iq...
%          - (b/Jm)*d_theta...
%          - ((b+bl)/Jm)*d_x + (bl/Jm)*d_xl...
%          - (kb/Jm)*x + (kb/Jm)*xl; 
% dd_xl = (bl/M)*d_x - (bl/M)*d_xl + (kb/M)*x - ((kb+ks)/M)*xl;     
% 
% T = Jm*dd_theta;
% F = M*dd_xl;
% 
% f = [d_id d_iq d_theta dd_theta d_xl dd_xl];
% g = [x F];
% 
% A = jacobian(f,[id iq theta d_theta xl d_xl]);
% B = jacobian(f,Vq);
% C = jacobian(g,[id iq theta d_theta xl d_xl]);
% D = jacobian(g,Vq);
% 
% A = subs(A,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);
% C = subs(C,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);

% A = subs(A,[Lq Ld R p lambda Ke Jm b Kt r l q kb ks bl M],[3.334e-3 3.334e-3 0.378 6 0.11 1.13 6.375e-04 3.0073e-04 1.39 22.5 80 9 48.75 4.875 0.001 10]);
% B = subs(B,[Lq Ld R p lambda Ke Jm b Kt r l q kb ks bl M],[3.334e-3 3.334e-3 0.378 6 0.11 1.13 6.375e-04 3.0073e-04 1.39 22.5 80 9 48.75 4.875 0.001 10]);
% C = subs(C,[Lq Ld R p lambda Ke Jm b Kt r l q kb ks bl M],[3.334e-3 3.334e-3 0.378 6 0.11 1.13 6.375e-04 3.0073e-04 1.39 22.5 80 9 48.75 4.875 0.001 10]);
% A = double(A);
% B = double(B);
% C = double(C);
% D = double(D);
% 
% sys = ss(A,B,C,D);
% 
% sys_min = minreal(sys);
% sys_min.StateName = {'iq','theta','d_theta','x','d_x'};
% sys_min.InputName = {'Vq_{ref}'};
% sys_min.OutputName = {'Position1','Force'};
% 
% figure(1)
% bode(sys_min)
% 
% figure(2)
% step(sys_min)