%%%  Given a model x = Ax + Bu
%%%        control u = -Kx + c
%%%  cost function J = sum xQx + uRu    (sum to infinity)
%%%
%%%   Then the cost function reduces to 
%%%      J =  cSc + unconstrained optimal
%%%     SS is for just one block, S is for nc blocks
%%%
%%%    [S,SS] = ssmpc_costfunction(A,B,K,nc,Q,R);
%%%
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [S,SS] = ssmpc_costfunction(A,B,K,nc,Q,R);

Phi = A-B*K;
Sx = dlyap(Phi',Q);      %%% S = [I +Phi'Phi + Phi^2'Phi^2+...]
Su = dlyap(Phi',K'*K);   %%% S = [K'K +Phi'K'KPhi + Phi^2'K'K Phi^2+...]
SS = B'*[Sx+Su]*B + eye(size(B,2));
nu=size(B,2);
for k=1:nc;
    vec=(k-1)*nu+1:k*nu;
    S(vec,vec)=SS;
end