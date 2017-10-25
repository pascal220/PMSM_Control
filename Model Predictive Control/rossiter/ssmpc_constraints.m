%%%%%   Constraints are summarised as 
%%%%%   CC c - dfixed - dx0*[z;r;d] <= 0
%%%%%                       d is a known disturbance
%%%%%
%%%%%   Predictions are 
%%%%%   x =  Pc1*c + Pz1*z + Pr1*r + Pd1*d
%%%%%   u =  Pc2*c + Pz2*z + Pr2*r + Pd2*d
%%%%%   Constraints are
%%%%%   umin < u < umax    Kxmax * x <xmax
%%%%%
%%%%% [CC,dfixed,dx0] = ssmpc_constraints(Pc1,Pc2,Pz1,Pz2,Pr1,Pr2,umin,umax,Kxmax,xmax);
%%%%%
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [CC,dfixed,dx0] = ssmpc_constraints(Pc1,Pc2,Pz1,Pz2,Pr1,Pr2,umin,umax,Kxmax,xmax);

nx=size(Kxmax,2);


[CC,dfixed,dx0] = inputcons(Pc2,Pz2,Pr2,umax,umin);         %%%% Input constraints
[CCx,dfixedx,dx0x] = statecons(Pc1,Pz1,Pr1,Kxmax,xmax,nx); %%%% State constraints

CC=[CC;CCx];
dfixed = [dfixed;dfixedx];
dx0 = [dx0;dx0x];


%%%%%   Set up input constraints  CC * c - dfixed - dx0 * [z;r] <=0
%%%%%      given
%%%%%   u =  Pc2*c + Pz2*z + Pr2*r + Pd2*d
%%%%%   umin < u < umax
%%%%%
function [CC,dfixed,dx0] = inputcons(Pc2,Pz2,Pr2,umax,umin);


nrows = size(Pc2,1);
steps = nrows/length(umax);
Up = umax; Ul = umin;

for i=2:steps
    Up = [Up;umax];
    Ul = [Ul;umin];
end

%%%%%      Pc2 c + Pz2 z + Pr2 r < Up
%%%%%      Pc2 c + Pz2 z + Pr2 r > Ul   or -Pc2 c - Pz2 z - Pr2 r < - Ul
CC = [Pc2;-Pc2];
dfixed = [Up;-Ul];
dx0 = [-Pz2,-Pr2;Pz2,Pr2];


%%%%   Set up state constraints for a state-space system
%%%%   CC * c - dfixed - dx0 * [z;r] <=0     z is state estimate
%%%%
%%%%   x =  Pc1*c + Pz1*z + Pr1*r  and  | Kxmax * x| < xmax

function [CC,dfixed,dx0] = statecons(Pc1,Pz1,Pr1,Kxmax,xmax,nx);

nrows = size(Pc1,1);
steps = nrows/nx;
nK = size(Kxmax,1);
nc = size(Pc1,2);
nz = size(Pz1,2);
nr = size(Pr1,2);
Xp = xmax; Xl = -xmax;

for i=1:steps-1
    Xp = [Xp;xmax];
end

for i=1:steps;
    Pc((i-1)*nK+1:i*nK,1:nc) = Kxmax*Pc1((i-1)*nx+1:i*nx,:);
    Pz((i-1)*nK+1:i*nK,1:nz) = Kxmax*Pz1((i-1)*nx+1:i*nx,:);
    Pr((i-1)*nK+1:i*nK,1:nr) = Kxmax*Pr1((i-1)*nx+1:i*nx,:);
end

%%%%% Constraints are 
%%%%%      Pc2 c + Pz2 z + Pr2 r < Up
%%%%%      Pc2 c + Pz2 z + Pr2 r > Ul   or -Pc2 c - Pz2 z - Pr2 r < - Ul
CC = [Pc;-Pc];
dfixed = [Xp;Xp];
dx0 = [-Pz,-Pr;Pz,Pr];


