%%%%%  Typical data entry required for a 
%%%%%  SISO state space  example
%%%%%    x(k+1) = A x(k) + B u(k)
%%%%%    y(k) = C x(k) + D u(k)     Note: Typically D=0
%%%%%
%%%%%   Assumes J = sum x(k+i) Q x(k+1) + u(k+i-1) R u(k+i-1)
%%%%%
%%%%%         and uses    (u-uss) = -k(x-xss)       *ss for steady-state
%%%%%
%%%%%  Illustrates closed-loop simulations with constraint handling
%%%%%  
%%%%%
%%%%%   THIS IS A SCRIPT FILE. CREATES ITS OWN DATA AS REQUIRED
%%%%%   EDIT THIS FILE TO ENTER YOUR OWN MODELS, ETC.
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

%% Model
A=0.9;
B=1;
C=1;
D=0;

%% Constraints
umax=0.2;
umin=-0.2;
Kxmax=1;
xmax=1.2;

%% Tuning parameters
Q=1;
R=1;
nc=5;   %%% no. of d.o.f.
x0=0;   %%% initial condition


ref = [zeros(1,20),ones(1,80)];   %% set point
dist = [zeros(1,40),ones(1,60)];   %% disturbance
noise = [zeros(1,70),randn(1,30)*.04];  %% noise

%%%%% Closed-loop simulation 
[x,y,u,c] = ssmpc_simulate(A,B,C,D,Q,R,umax,umin,Kxmax,xmax,nc,x0,ref,dist,noise);