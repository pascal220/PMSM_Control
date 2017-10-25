%%%%%  Typical data entry required for a 
%%%%%  SISO transfer function example
%%%%%    [1-1.8z^{-1}+0.81z^{-2}] y  =  [z^{-1}+0.3z^{-2}] u
%%%%%
%%%%%  Illustrates closed-loop GPC simulations with constraint handling
%%%%%  and the impact of the T-filter
%%%%%
%%%%%   THIS IS A SCRIPT FILE. CREATES ITS OWN DATA AS REQUIRED
%%%%%   EDIT THIS FILE TO ENTER YOUR OWN MODELS, ETC.
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

%% Constraints
umax=1;
umin=-1;
Dumax=.5;

%% Model
A=[1 -1.8 .81]; 
B=[1,.3]/100;
sizey=1;

%% Tuning parameters
Wu =1;
Wy=1;
ny=15;
nu=3;
Tfilt=[1 -0.8];

%%% Set point, disturbance and noise
ref = [zeros(1,10),ones(1,130)];
dist=[zeros(1,60),ones(1,80)*.01];
noise = [zeros(1,100),randn(1,40)*.01];

%%%%% Closed-loop simulation without and with a T-filter
[y,u,Du,r] = mpc_simulate(B,A,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise);
[yt,ut,Dut,r] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise);

%%%%% Collect control law without and with a T-filter
%%%%%         Du(k) = Pr*r(k+1) - Dk*Du(k-1) - Nk*y(k) 
[Nk,Dk,Pr] = mpc_simulate(B,A,nu,ny,Wu,Wy);
%%%%%         Du(k) = Pr*r(k+1) - Dkt*Dut(k-1) - Nkt*yt(k)  (filtered past data)
[Nkt,Dkt,Pr] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy);



%%% Closed-loop poles without T-filter are given from
%%% Pc = [I+z^{-1}Dk(z)]*A(z)*Delta(z) + Nk(z)*z^{-1}B(z)
    Pc1 = conv([1,Dk],conv(A,[1,-1])); n1 = length(Pc1);
    Pc2 = conv([0,B],Nk); n2 = length(Pc2);
    if n1>n2; Pc2(n1)=0;elseif n2>n1;Pc1(n2)=0;end
    Pc=Pc1+Pc2; clear Pc1 Pc2 n1 n2;

%%% Closed-loop poles with the T-filter are given from
%%% Pc = [Tfilt+z^{-1}Dkt(z)]*A(z)*Delta(z) + Nkt(z)*z^{-1}B(z)
    TD=Tfilt;TD(2:length(Dkt)+1) = TD(2:length(Dkt)+1)+Dkt;
    Pc1 = conv(TD,conv(A,[1,-1])); n1 = length(Pc1);
    Pc2 = conv([0,B],Nkt); n2 = length(Pc2);
    if n1>n2; Pc2(n1)=0;elseif n2>n1;Pc1(n2)=0;end
    Pct=Pc1+Pc2; clear Pc1 Pc2 n1 n2 TD;
    Pct = deconv(Pct,Tfilt);
    
