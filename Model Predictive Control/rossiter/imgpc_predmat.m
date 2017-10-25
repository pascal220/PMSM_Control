%%%%  To form prediction matrices over horizon ny given
%%%%
%%%%   x(k+1) = Ax(k)+Bu(k);    y(k) = Cx(k) + D u(k);  Assumes D=0
%%%%  
%%%%   Use absolute inputs (not increments)
%%%%
%%%%   yfut = P*x + H*ufut + L*offset   [offset = y(process) - y(model)]
%%%%
%%%   Also estimate steady-state input as     uss = M(r-offset)
%%%
%%%%  [H,P,L,M] = imgpc_predmat(A,B,C,D,ny);
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [H,P,L,M] = imgpc_predmat(A,B,C,D,ny)


%%% Estimate uss as uss = M(r-offset)
Gss = C*inv(eye(size(A,2))-A)*B;
M=inv(Gss);

%%%% Initialise
Px=C*A;  Pu=C*B;  P=C;
nx=size(A,1);
nB=size(B,2);
nC = size(C,1);
L=[];
%%%% Use recursion to find predictions
for i=1:ny;
   
   Puterm = P*B;
   for j=i:ny;
         vrow=(j-1)*nC+1:j*nC;
         vcol=(j-i)*nB+1:(j-i+1)*nB;
         Pu(vrow,vcol)=Puterm;
   end
   P=P*A;
   vrow=(i-1)*nC+1:i*nC;
   Px(vrow,1:nx) = P;
   L=[L;eye(nC)];
end
    H=Pu;
    P=Px;
    