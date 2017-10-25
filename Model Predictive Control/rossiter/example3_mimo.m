%%%%%  Typical data entry required for a 
%%%%%  MIMO state space  example
%%%%%    x(k+1) = A x(k) + B u(k)
%%%%%    y(k) = C x(k) + D u(k) + dist     Note: Assumes D=0
%%%%%
%%%%%   Assumes J = sum (r-y)^2 +  (u(k+i-1)-uss) R (u(k+i-1)-uss)
%%%%%                uss the steady state input
%%%%%
%%%%%  Illustrates closed-loop simulations with constraint handling
%%%%%  
%%%%%
%%%%%   THIS IS A SCRIPT FILE. CREATES ITS OWN DATA AS REQUIRED
%%%%%   EDIT THIS FILE TO ENTER YOUR OWN MODELS, ETC.
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

%%% Model
A = [0.6000   -0.4000    0.2000;
    1.0000    0.4000    0.4000;
    0.8000    1.0000    0.4000];
B =[ 0.4 0.3;
     0 -1;
     0 0.1]/2;
C =[1.0000   -2.2000    1.1200;
     0        1          1];
D =[0 0;0 0];

%%%% Constraints
umax=[0.8;2];
umin=-[1.5;2];
Dumax=[0.4;0.5];

%%%% Tuning parameters
R=eye(2)*2;
ny=15;          %%%% prediction horizon
nu=4;           %%%% control horizon
x0=[1;1;0]*0;   %%%% Initial state

ref = [zeros(2,20),[1*ones(2,60)]];    %%%% Set point
dist = [zeros(2,35),-ones(2,45)*.5];   %%%% disturbance
noise = [zeros(2,50),randn(2,30)*.04]; %%%% measurement noise

%%%%% Closed-loop simulation 
[x,y,u,r] = imgpc_simulate(A,B,C,D,R,ny,nu,umax,umin,Dumax,x0,ref,dist,noise);