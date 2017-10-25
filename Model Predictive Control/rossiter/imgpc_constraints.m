%%%%%   Constraints are summarised as 
%%%%%   umin < u < umax    and | Du | < Dumax
%%%%%   
%%%%%   or     CC*ufut -dfixed - dxu*u(k-1)<=0  
%%%%%                        [Note: absolute inputs not increments]
%%%%%
%%%%%     nu is the control horizon
%%%%%
%%%%%    [CC,dfixed,dxu] = imgpc_constraints(nu,umin,umax,Dumax)
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [CC,dfixed,dxu] = imgpc_constraints(nu,umin,umax,Dumax)

sizeu = length(umax);

%%%%% Absolute limits
CC=[eye(nu*sizeu);-eye(nu*sizeu)];
dfixed=[umax;-umin];
for k=2:nu;
    dfixed = [umax;dfixed;-umin];
end

%%%% rate limits
[Cd,Hd] = caha([eye(sizeu),-eye(sizeu)],sizeu,nu);
CCR=[Cd;-Cd];
dfixedr=[Dumax];
for k=2:2*nu;
    dfixedr = [dfixedr;Dumax];
end
dxu=[zeros(2*nu*sizeu,sizeu);-Hd;Hd];

%%% All limits
CC=[CC;CCR];
dfixed = [dfixed;dfixedr];
