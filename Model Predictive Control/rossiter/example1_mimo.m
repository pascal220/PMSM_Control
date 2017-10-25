%%%%%  Typical data entry required for a 
%%%%%  MIMO MFD model
%%%%%    [Ao + A1z^{-1}+...] y = [B1 z^{-1} + B2 z^{-2}+...] u
%%%%%    A = [A0,A1,A2,...]      B = [B1,B2,...]
%%%%%
%%%%%  Illustrates closed-loop GPC simulations with constraint handling
%%%%%  and the impact of the T-filter
%%%%%
%%%%%   THIS IS A SCRIPT FILE. CREATES ITS OWN DATA AS REQUIRED
%%%%%   EDIT THIS FILE TO ENTER YOUR OWN MODELS, ETC.
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

%%% Model
A=[];B=[];
A(1,1:3:12) = poly([.5,.8,-.2]);
A(2,2:3:12) = poly([-.2,.9,.5]);
A(3,3:3:12) = poly([-.1,.4,.6]);
A(1,5:3:12) = [ -.2 .1 .02];
A(2,4:3:12) = [ .4 0 -.1];
A(3,4:3:12) = [0 .2 .2];
A(2,6:3:12) = [-1 .1 .3];
A(:,4:12) = A(:,4:12)*.8;

B = [.5 0.2 -.5 1 2 1;2 0 .3 -.8 .6 .5;0 .9 -.4 1 .3 .5];
sizey=3;

%%% Tuning parameters
nu=5;
ny =30;
Wu = diag([1 1 1]);
Wy=diag([1,1, 1]);
Tfilt = [eye(3),-eye(3)*.8];

%%% Constraints
Dumax = [.2;.25;.4];
umax=[1;1;1];
umin = [-1;-1;-2];


%%%%% Set point, disturbance and noise
ref = [zeros(1,10),ones(1,130);zeros(1,10),-ones(1,130)/2;zeros(1,10),ones(1,130)*.7];
dist=[zeros(3,60),ones(3,80)*.2]; 
noise = [zeros(3,100),randn(3,40)*.03];


%%%%% Closed-loop simulation without and with a T-filter
[y,u,Du,r] = mpc_simulate(B,A,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise);
[yt,ut,Dut,r] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise);

%%%%% Collect control law without and with a T-filter
%%%%%         Du(k) = Pr*r(k+1) - Dk*Du(k-1) - Nk*y(k) 
[Nk,Dk,Pr] = mpc_simulate(B,A,nu,ny,Wu,Wy);
%%%%%         Du(k) = Pr*r(k+1) - Dkt*Dut(k-1) - Nkt*yt(k)  (filtered past data)
[Nkt,Dkt,Pr] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy);
