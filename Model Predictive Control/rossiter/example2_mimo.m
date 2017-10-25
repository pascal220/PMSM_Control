%%%%%  Typical data entry required for a 
%%%%%  SISO state space  example
%%%%%    x(k+1) = A x(k) + B u(k)
%%%%%    y(k) = C x(k) + D u(k) + dist     Note: Assumes D=0
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
A =[0.9146         0    0.0405;
    0.1665    0.1353    0.0058;
         0         0    0.1353];
B =[0.0544   -0.0757;
    0.0053    0.1477;
    0.8647         0];
C =[1.7993   13.2160         0;
    0.8233         0         0];
D=zeros(2,2);

%% Tuning parameters
Q=C'*C;
R=eye(2);
nc=5;   %%% no. of d.o.f.

%% Constraints
umax=[1;2];
umin=-[1;2];
Kxmax=[0 0 0];
xmax=1;
x0=[0;0;0];    %%% Initial condition


ref = [zeros(2,20),[2*ones(1,120);ones(1,120)]];  %% Set point
dist = [zeros(2,60),-ones(2,80)*.2];              %% Disturbance
noise = [zeros(2,100),randn(2,40)*.04];           %% noise

%%%%% Closed-loop simulation 
[x,y,u,c] = ssmpc_simulate(A,B,C,D,Q,R,umax,umin,Kxmax,xmax,nc,x0,ref,dist,noise);