%%%%%%%%%%%%%% Either:                        (WITH T-filter!!)
%%%%%%%%%%%%%%  (1) Gives control law parameters (nargin = 6 only)
%%%%%%%%%%%%%%  (2) Simulates MIMO GPC with constraint handling
%%%
%%%%%  [Nk,Dk,Pr] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy)
%%%%%         Du(k) = Pr*r(k+1) - Dk*Du(k-1) - Nk*y(k) 
%%
%%%%%  [y,u,Du,r] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)
%              y, u, Du, r are dimensionally compatible 
% closed-loop outputs/inputs/input increments and supplied set-point and disturbance
%
% MFD model     Ay(k) = Bu(k-1) + Tfilt*dist
%
% ny is output horizon
% nu is the input horizon
% Wu is the diagonal control weighting 
% Wy is the diagonal output weighting
% sizey no. outputs and inputs (assumed square)
% dist, noise are the disturbance and noise signals
% ref is the reference signal
% Dumax is a vector of limits on input increments (assumed symetric)
% umax, umin are vectors of limits on the inputs
%
%%%%%
%%%%%  [y,u,Du,r] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [y,u,Du,r] = mpc_simulate_tfilt(B,A,Tfilt,nu,ny,Wu,Wy,Dumax,umax,umin,ref,dist,noise)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Error checks
sizey = size(A,1);
if size(B,2)==sizey;B=[B,zeros(sizey,sizey)];end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find prediction matrices 
%%%%    yfut = H *Dufut + P*Dupast + Q*ypast
%%%%    yfut = H *Dufut + Pt*Dutpast + Qt*ytpast   %% filtered data
[H,P,Q] = mpc_predmat(A,B,ny);
[Pt,Qt] = mpc_predtfilt(H,P,Q,Tfilt,sizey,ny);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find control law and parameters of the cost function
%%%%   Dufut = Pr*rfut - Dk*Dutpast - Nk*ytpast 
%%%%    J = Dufut'*S*Dufut + Dufut'*2X*[Dutpast;ytpast;rfut]
[Nk,Dk,Pr,S,X] = mpc_law(H,Pt,Qt,nu,Wu,Wy,sizey);

if nargin==7; %%%% collect control law and stop
    y=Nk(1:sizey,:); 
    u=Dk(1:sizey,:); 
    Du=Pr(1:sizey,:);  
  else    %%%% continue to simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin~=13;disp('Incomplete input information - stopping');break;end    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%   Define constraint matrices
%%%%%%   CC*Dufut - dd - dd1*ut <= 0
[CC,dd,dd1]  = mpc_constraints(Dumax,umax,umin,sizey,nu);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Set up simulation parameters
nNk = size(Nk,2)/sizey;
nDk = size(Dk,2)/sizey;
nT = size(Tfilt,2)/sizey;
T2 = Tfilt(:,sizey+1:nT*sizey);
nA = size(A,2)/sizey;
nB = size(B,2)/sizey;
init = max([nNk,nDk,nA,nB])+2;
y = zeros(sizey,init);    yt=y;
u = y;
Du = u;     Dut = u;
r = u;
d=u;
opt = optimset('quadprog');
opt.Diagnostics='off';    %%%%% Switches of unwanted MATLAB displays
opt.LargeScale='off';     %%%%% However no warning of infeasibility
opt.Display='off';
runtime = size(ref,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



for i=init:runtime-1;
   
    %%% Update unconstrained control law

    
    %%%%% Update filtered data
    ytpast = yt(:,i-1:-1:i-nT+1);
    yt(:,i) = y(:,i) + noise(:,i) - T2*ytpast(:);
    Dutpast = Dut(:,i-2:-1:i-nT);
    Dut(:,i-1) = Du(:,i-1) - T2*Dutpast(:);

   
%%% Define vectors of past filtered data for use by control law
d(1:sizey,i+1)=dist(:,i+1);
ypast = yt(:, i:-1:i+1-nNk);
Dupast = Dut(:, i-1:-1:i-nDk) ;
upast = u(:, i-1);
rfut = ref(:,i+1); 

%%%%%%% Unconstrained law - if needed
Dufast = Pr*rfut - Nk*ypast(:) - Dk*Dupast(:);

% Form constraint matrices and solve constrained optimisation
%  CC*Dufast-dd-dd1*upast<=0
dt = dd+dd1*upast;
Dufast2 = quadprog(S,X*[Dupast(:);ypast(:);rfut(:)],CC,dt,[],[],[],[],[],opt);
Du(:,i) = Dufast2(1:sizey);
u(:,i) = u(:,i-1)+Du(:,i);

%  Ensure the constraints satisfied by proposed control law   
for j=1:sizey;
   if u(j,i)>u(j,i-1)+Dumax(j),u(j,i)=u(j,i-1)+Dumax(j);end
   if u(j,i)<u(j,i-1)-Dumax(j),u(j,i)=u(j,i-1)-Dumax(j);end
   if u(j,i)>umax(j); u(i)=umax(j);end
   if u(j,i)<umin(j); u(i)=umin(j);end
end
Du(:,i) = u(:,i)-u(:,i-1);
%  End to update control law


%%% Simulate the process
upast2 = u(:,i:-1:i-nB+1);
ypast2 = y(:, i:-1:i+2-nA);
y(:,i+1) = -A(:,sizey+1:nA*sizey)*ypast2(:) + B*[upast2(:)]+ d(:,i+1);
r(:,i+1) = ref(:,i+1);


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Ensure all outputs are dimensionally compatible
u(:,i+1) = u(:,i);
Du(:,i+1) = Du(:,i)*0;
noise = noise(:,1:i+1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Produce a neat plot
time=0:size(u,2)-1;
for i=1:sizey;
    figure(i+3);clf reset
    plotall(y(i,:),r(i,:),u(i,:),Du(i,:),d(i,:),noise(i,:),umax(i),umin(i),Dumax(i),time,i);
end


disp('*******************************************************************************');
disp(['***    For GPCT there are ',num2str(sizey),' figures beginning at figure 4   ***']);
disp('*******************************************************************************');


end   %%%% Check for nargin = 6



%%%%% Function to do plotting in the MIMO case and 
%%%%% allow a small boundary around each plot

function plotall(y,r,u,Du,d,noise,umax,umin,Dumax,time,loop)

uupper = [umax,umax]';
ulower = [umin,umin]';
Dulim = [Dumax,Dumax]';
time2 = [0,time(end)];
rangeu = (max(umax)-min(umin))/20;
rangey = (max(max(y))-min(min(y)))/20;
ranged = (max(max([d,noise]))-min(min([d,noise])))/20;

subplot(221);plot(time,y','-',time,r','--');
axis([time2,min(min(y))-rangey,max(max(y))+rangey]);
xlabel(['GPCT - Outputs and set-point in loop ',num2str(loop)]);
subplot(222);plot(time,Du','-',time2,Dulim,'--',time2,-Dulim,'--');
axis([time2,min(-Dumax)-rangeu,max(Dumax)+rangeu]);
xlabel(['GPCT - Input increments in loop ',num2str(loop)]);
subplot(223);plot(time,u','-',time2,uupper,'--',time2,ulower,'--');
axis([time2,min(umin)-rangeu,max(umax)+rangeu]);
xlabel(['GPCT - Inputs in loop ',num2str(loop)]);
subplot(224);plot(time,d','b',time,noise,'g');
axis([time2,min(min([d,noise]))-ranged,max(max([d,noise]))+ranged]);
xlabel(['GPCT - Disturbance/noise in loop ',num2str(loop)]);
