%%% Simulation of dual mode optimal predictive control
%%%
%%  [x,y,u,c,r] = ssmpc_simulate(A,B,C,D,Q,R,umax,umin,Kxmax,xmax,nc,x0,ref,dist,noise)
%%
%%%     x(k+1) = A x(k) + B u(k)            x0 is the initial condition
%%%     y(k) = C x(k) + D u(k) + dist       Note: Assumes D=0, dist unknown
%%%
%%  input constraint    umax, umin
%%  state constraints  | Kxmax x | < xmax
%%  reference trajectory     ref, r
%%  perturbation to control  c
%%  Number of d.o.f.         nc
%%  Weighting matrices in J  Q, R
%%  measurement noise        noise
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [x,y,u,c,r] = ssmpc_simulate(A,B,C,D,Q,R,umax,umin,Kxmax,xmax,nc,x0,ref,dist,noise)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Find L2 optimal control law  and observor with integrator    
%%%%%  Control law is written as  
%%%%%   
%%%%       Control law is   u = -Knew z + Pr r + c
%%%%
%%%%%      Observor is   z = Ao*z +Bo*u + L*(y + noise - Co*z );        z=[xhat;dhat]
%%%%%
%%%%%      K the underlying control law:  u-uss = -K(x-xss) 
[K,L,Ao,Bo,Co,Do,Knew,Pr] = ssmpc_observor(A,B,C,D,Q,R);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% To set up prediction matrices for dual mode state space MPC
%%%%% Assume closed-loop predictions
%%%%%  x =  Pc1*c + Pz1*z + Pr1*r      z=[x;d]
%%%%%  u =  Pc2*c + Pz2*z + Pr2*r 
%%%%%  y =  Pc3*c + Pz3*z + Pr3*r 
[Pc1,Pc2,Pc3,Pz1,Pz2,Pz3,Pr1,Pr2,Pr3] = ssmpc_predclp(A,B,C,D,Knew,Pr,nc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% The optimal cost is 
%%%%%     J = c'Sc + c'X + unconstrained optimal
[S] = ssmpc_costfunction(A,B,K,nc,Q,R);   S=(S+S')/2;
X = zeros(nc*size(B,2),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Constraints are summarised as 
%%%%%   CC c - dfixed - dx0*[z;r;d] <= 0     d is a known disturbance
[CC,dfixed,dx0] = ssmpc_constraints(Pc1,Pc2,Pz1,Pz2,Pr1,Pr2,umin,umax,Kxmax,xmax);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%   SIMULATION

%%%%%%%%%% Initial Conditions 
nu=size(B,2);
c = zeros(nu*nc,2); u =zeros(nu,2); y=zeros(nu,2); x=[x0,x0];
z = [x;y];
r=ref;
nK = size(Kxmax,1);
runtime = size(ref,2)-1;
opt = optimset;
opt.Display='off'; %'notify';
opt.Diagnostics='off';
opt.LargeScale='off';

for i=2:runtime;

%%%%%%%%%%%%%%% CONSTRAINT HANDLING PART 
c(:,i+1) = c(:,i);  
c(:,i) =  quadprog(S,X,CC,dfixed+dx0*[z(:,i);r(:,i)],[],[],[],[],[],opt);

%%%%% Control law
u(:,i) = -Knew*z(:,i) + c(1:nu,i) + Pr*r(:,i);

%%%%% Physical constraint check  
   for j=1:nu;
        if u(j,i) > umax(j);  u(j,i) = umax(j);  end
        if u(j,i) < umin(j);  u(j,i) = umin(j);  end
   end

%%%% Simulate model      
     x(:,i+1) = A*x(:,i) + B*u(:,i) ;
     y(:,i+1) = C*x(:,i+1) + dist(:,i+1);
%%%% Observer part
     z(:,i+1) = Ao*z(:,i) +Bo*u(:,i) + L*(y(:,i) + noise(:,i) - Co*z(:,i));


end

%%%% Ensure all variables have conformal lengths
u(:,i+1) = u(:,i);  
c(:,i+1)=c(:,i);
r = r(:,1:i+1);
noise = noise(:,1:i+1);
d = dist(:,1:i+1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Produce a neat plot
time=0:size(u,2)-1;
for i=1:nu;
    figure(i);clf reset
    plotall(y(i,:),r(i,:),u(i,:),c(i,:),d(i,:),noise(i,:),umax(i),umin(i),time,i);
end

disp('**************************************************');
disp(['***    There are ',num2str(nu),' figures    ***']);
disp('**************************************************');


%%%%% Function to do plotting in the MIMO case and 
%%%%% allow a small boundary around each plot
function plotall(y,r,u,c,d,noise,umax,umin,time,loop)

uupper = [umax,umax]';
ulower = [umin,umin]';
time2 = [0,time(end)];
rangeu = (max(umax)-min(umin))/20;
rangey = (max(max(y))-min(min(y)))/20;
ranged = (max(max([d,noise]))-min(min([d,noise])))/20;if ranged==0;ranged=1;end
rangec = (max(c)-min(c))/20; if rangec==0;rangec=1;end

subplot(221);plot(time,y','-',time,r','--');
axis([time2,min(min(y))-rangey,max(max(y))+rangey]);
xlabel(['LQMPC - Outputs and set-point in loop ',num2str(loop)]);
subplot(222);plot(time,c','-');
axis([time2,min(c)-rangec,max(c)+rangec]);
xlabel(['LQMPC - Input perturbations in loop ',num2str(loop)]);
subplot(223);plot(time,u','-',time2,uupper,'--',time2,ulower,'--');
axis([time2,min(umin)-rangeu,max(umax)+rangeu]);
xlabel(['LQMPC - Inputs in loop ',num2str(loop)]);
subplot(224);plot(time,d','b',time,noise,'g');
axis([time2,min(min([d,noise]))-ranged,max(max([d,noise]))+ranged]);
xlabel(['LQMPC - Disturbance/noise in loop ',num2str(loop)]);


