%% Electric parameters
% p = 6;            % Number of poles of the ac motor
% R = 378e-3;       % Stator Resistance - Ohm
% Ld = 3.427e-3;    % Inductance in the other H
% Lq = 3.334e-3;    % Inductance in stator H
% lambda = 0.11;    % Flux induced by PM
% Ke = 0.0169;

%% Mechanical parameters
% Jm = 6.375e-04;
% Kt = 191.2949;
% ba = 3.0073e-04;

%% Kinematics and last Dynamics
% l = 80;
% r = 22.5;
% q = 9;
% kb = 48.75;
% ks = 4.875;
% bl = 0.001;
% m = 10;

%% PI current
% c1_i = 80.3333;
% c2_i = 1.1017e+04;
% c3_i = 1;
% c4_i = 263.0465;

%% PI velocity
% c1_v = 7.813;
% c2_v = 4.867;
% c3_v = 1;
% c4_v = 100.4;

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

% Of current
d_id = (1/Ld)*Vd - (R/Ld)*id + (Lq/Ld)*p*d_theta*iq;                    
d_iq = (1/Lq)*Vq - (R/Lq)*iq - (Ld/Lq)*p*d_theta*id - ((lambda*p)/Lq)*d_theta;
    
f = [d_id d_iq];                                                                           

A_i = jacobian(f,[id iq]);                                     
B_i = jacobian(f,[Vd Vq]);          
C_i = eye(2);                                
D_i = 0;

A_i = subs(A_i,[id iq d_theta],[0 0 0]);
A_i = subs(A_i,[R p],[0.378 6]);
A_i = subs(A_i,[Ld Lq],[3.334e-3 3.334e-3]); 
B_i = subs(B_i,[Ld Lq],[3.334e-3 3.334e-3]);

% Of velocity
% Jt = Jm + Jg + (Jc/Tr^2);
dd_theta = (1.5*p*((Ld-Lq)*iq*id))/Jt + (Kt/Jt)*iq - (b/Jt)*d_theta;     

f = [d_theta dd_theta];                                                                           

A_vel = jacobian(f,[theta d_theta]);                                     
B_vel = jacobian(f, [id iq]);          
C_vel = eye(2);                                
D_vel = 0;

A_vel = subs(A_vel,[id iq theta d_theta],[0 0 0 0]);
B_vel = subs(B_vel,[id iq theta d_theta],[0 0 0 0]);
A_vel = subs(A_vel,[Ld Lq],[3.427e-3 3.334e-3]);
B_vel = subs(B_vel,[Ld Lq],[3.427e-3 3.334e-3]);
A_vel = subs(A_vel,[Jt b],[0.0057 3.0073e-04]);
B_vel = subs(B_vel,Kt,191.2949);

%% Nie chce mnie sie 
% Current loop
Aser_i = [[Api_i, 0;0, Api_i], zeros(size(A_i));
                    B_i*Cpi_i,A_i];
Bser_i = [[Bpi_i 0;0 Bpi_i];
               B_i*Dpi_i];
Cser_i = [[D_i*Cpi_i 0;0 D_i*Cpi_i], C_i];
Dser_i = D_i*Dpi_i;

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

Aloop_v = (Aser_v - Bser_v*[0 1]*Cser_v);
Bloop_v = Bser_v;
Cloop_v = Cser_v;
Dloop_v = Dser_v;

Aloop_v = subs(Aloop_v,[Lq p c3_v],[3.334e-3 6 1]);
Bloop_v = subs(Bloop_v,c3_v,1);
Aloop_v = subs(Aloop_v,[c1_v c2_v c4_v],[7.813 4.8671 100.4]);
Bloop_v = subs(Bloop_v,[c1_v c2_v c4_v],[7.813 4.8671 100.4]);
Aloop_v = subs(Aloop_v,Ke,0.0169);

%% Kinematics and last Dynamics
syms xl d_xl
syms r l q kb ks bl M Jm   

x  = l*(1- (r/l*sin(theta)-q/l)^2)^(1/2)-r*cos(theta);                      % Angle of shaft to linear postion                   
 
d_x = d_theta*(r*sin(theta) + (r*cos(theta)*(q/l - (r*sin(theta))/l))/...
    (1 - (q/l - (r*sin(theta))/l)^2)^(1/2));                                % Angluar velocity of shaft to linear velocity

dd_theta = (1.5*p*((Ld-Lq)*iq*id))/Jm + (Kt/Jm)*iq...
         - (b/Jm)*d_theta...
         - ((b+bl)/Jm)*d_x + (bl/Jm)*d_xl...
         - (kb/Jm)*x + (kb/Jm)*xl; 
     
dd_xl = (bl/M)*d_x - (bl/M)*d_xl + (kb/M)*x - ((kb+ks)/M)*xl;
F2 = M*dd_xl;

g = [x; F2];

C = jacobian(g,[id iq theta d_theta xl d_xl]);

C = subs(C,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);
C = subs(C,[Jm b p Kt r l q M kb ks bl],[6.375e-04 3.0073e-04 6 191.2949 22.5e-3 80e-3 9e-3 10 48.75 4.875 0.001]);     
C = double(C);

C_GSSA = [zeros(2,3) C];

block1 = [zeros(6,2);0 0];
block2 = [zeros(2,5),[0 0;0 0]];
block3 = [0 1;0 0];
Awhole_pf = [Aloop_v block1;block2 block3];

f2 = [d_theta dd_theta d_xl dd_xl];   

block = jacobian(f2,[id iq theta d_theta xl d_xl]);
block = subs(block,[id iq theta d_theta xl d_xl],[0 0 0 0 0 0]);
Awhole_pf(6:end,4:end) = block;
A_GSSA = subs(Awhole_pf,[Jm b p Kt r l q M kb ks bl],[6.375e-04 3.0073e-04 6 191.2949 22.5e-3 80e-3 9e-3 10 48.75 4.875 0.001]);
A_GSSA = double(A_GSSA);

B_GSSA = [Bloop_v*((l*(1 - q^2/l^2)^(1/2))/(q*r));0;0];
B_GSSA = subs(B_GSSA,[r l q],[22.5e-3 80e-3 9e-3]); 
B_GSSA = double(B_GSSA);

D_GSSA = zeros(2,1);

sys_moj_open = ss(A_GSSA,B_GSSA,C_GSSA,D_GSSA);

sys_min  = minreal(sys_moj_open);
sys_min.StateName = {'e_v','e_iq','iq','theta','d_theta','x','d_x'};
sys_min.InputName = {'Angular Velocity (ref)'};
sys_min.OutputName = {'Position','Force'};
sys_min.InputUnit = 'mm/s/rad';
sys_min.OutputUnit = {'mm', 'N'};

[A_GSSA,B_GSSA,C_GSSA,D_GSSA] = ssdata(sys_min); 

%% The discrete model
Ts = 0.001;

sys_dz = c2d(sys_min,Ts,'zoh');
% sys_df = c2d(sys_min,Ts,'foh');
% sys_dt = c2d(sys_min,Ts,'tustin');

[A_dz,B_dz,C_dz,D_dz] = ssdata(sys_dz); 

% figure(1001)
% step(sys_min,sys_dz)
% legend('Continuous','Discrete')

% figure(1000)
% bode(sys_min,sys_dz)

%% Observer + MPC
% %Kalman Observer
%Noise
noise = read_noise();

vel_ref = noise(:,2);
vel_feed = noise(:,3);
iq_ref = noise(:,4);
iq_feed = noise(:,5);
error_vel = vel_ref - vel_feed;
error_iq = iq_ref - iq_feed;

noise_con = [error_vel, error_iq, iq_feed, vel_feed];

% Q >> R, system quick but nosie. R >> Q system slow.
Qn = max(var(noise_con)); % process noise
Rn = [0.001 0;
      0   0.1];% measurment noise

A_ex = [eye(2), C_GSSA;
        zeros(7,2), A_GSSA];
B_ex = [0;0;B_GSSA];
C_ex = [eye(2),C_GSSA];
D_ex = D_GSSA;

sys_ex = ss(A_ex,B_ex,C_ex,D_ex);
sys_ex_d = c2d(sys_ex,0.001);
 
% [A_dz,B_dz,C_dz,D_dz] = ssdata(sys_d); 

[kest,L,P,M] = kalman(sys_ex_d,Qn,Rn); %Finding Kalman

% %MPC
mpcverbosity('off');
%Sampling time
Ts = 0.001;
%Prediction horizon
p = 50;
%Control horizon
m = 10;

MPCobj = mpc(sys_dz,Ts,p,m);

%Desgined Kalman observer/filter (used instead of the default)
setEstimator(MPCobj,L,M);

%Controller tuning weights
MPCobj.W.MVRate = 0.001;
MPCobj.W.OV = [100 0];
%Bounds and other properties of manipulated variables
MPCobj.MV.Min = -1000;
MPCobj.MV.Max = 1000;
%Bounds and other properties of the output variables
MPCobj.OV(1).Min = -8.5;
MPCobj.OV(1).Max = 8.5;
MPCobj.OV(2).Min = -300;
MPCobj.OV(2).Max = 300;

% review(MPCobj)
%% System with Cascade Control
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
% d_iq = (1/Lq)*Vq - (R/Lq)*iq - (Ld/Lq)*p*d_theta*id - Ke/Lq*d_theta;
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
% 
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
% sys_min.InputName = {'Voltage q- axis (ref)'};
% sys_min.OutputName = {'Position1','Force'};
% 
% sys_wc_dz = c2d(sys_min,Ts,'zoh');
% 
% [A_wc_dz,B_wc_dz,C_wc_dz,D_wc_dz] = ssdata(sys_wc_dz); 

%% MPC WC
% Q >> R, system quick but nosie. R >> Q system slow.
% Qn = max(var(noise_con)); %process noise
% Rn = [0.01 0;
%       0   0.1];      %measurment noise
% 
% [kest_wc,L_wc,P,M] = kalman(sys_wc_dz,Qn,Rn); %Finding Kalman
% 
% mpcverbosity('off');
% Sampling time
% Ts = 0.001;
% Prediction horizon
% p = 100;
% Control horizon
% m = 20;
% 
% MPCobj_wc = mpc(sys_min,Ts,p,m);
% 
% Controller tuning weights
% MPCobj_wc.W.MVRate = 0.0005;
% MPCobj_wc.W.OV = [200 0];
% 
% Bounds and other properties of manipulated variables
% MPCobj_wc.MV.Min = -2500;
% MPCobj_wc.MV.Max = 2500;
% 
% Bounds and other properties of the output variables
% MPCobj_wc.OV(1).Min = -8.5;
% MPCobj_wc.OV(1).Max = 8.5;
% MPCobj_wc.OV(2).Min = -1000;
% MPCobj_wc.OV(2).Max = 1000;
