%%%%%  Typical data entry required for a 
%%%%%  SISO state space  example
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
A=[1.4000   -0.1050   -0.1080; 2 0 0; 0 1 0];
B =[2; 0; 0]/10;
C =[ 0.5000    0.7500    0.5000]*10;
D=0;

%%% Constraints
umax=.04;
umin=-.04;
Dumax=.03;

%%% Tuning parameters
R=10;
ny=15;       %% Prediction horizon
nu=3;        %% Control horizon

x0=[1;1;0]*0;  %% Initial condition

ref = [zeros(1,20),[1*ones(1,60)]];    %%% Set point
dist = [zeros(1,35),-ones(1,45)*.5];   %%% disturbance
noise = [zeros(1,50),randn(1,30)*.04]; %%% noise

%%%%% Closed-loop simulation 
[x,y,u,r] = imgpc_simulate(A,B,C,D,R,ny,nu,umax,umin,Dumax,x0,ref,dist,noise);