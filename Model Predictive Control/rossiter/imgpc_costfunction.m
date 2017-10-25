%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find a predictive control law and optimal cost given predictions
%%%%   yfut = P*xfut+H*ufut+L*offset
%%%%   uss  = M(r-offset)
%%%%
%%%%   Uses the cost function   J = sum (r-y)^2 + R(u-uss)^2
%%%%     J = ufut'*S*ufut + 2 ufut'*X*[x;r-offset] + k
%%%%   
%%%%   Cost weights absolute inputs (not increments) so need assumption
%%%%             u(k+nu+i) = u(k+nu-1) in predictions
%%%%
%%%%   Control weighting          R
%%%%   Control/ouput horizon      nu, ny
%%%%   Output dimension           sizey
%%%%
%%%%   Control law is
%%%%         ufut = -K*x  + Pr*(r-offset)     (No advance knowledge is used)
%%%%
%%%%   [S,X,K,Pr] = imgpc_costfunction(H,P,L,M,R,nu,sizey,ny);
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [S,X,K,Pr,H] = imgpc_costfunction(H,P,L,M,R,nu,sizey,ny);


%% J = | Lr - P x- H ufut - L*offset |  + R |ufut-uss|
%% J = | Lr - P x- H ufut - L*offset |  + R |ufut - M*(r-offset)|

%%% Sum last columns of H
v=(nu-1)*sizey;
for k=1:sizey;
    HH(:,k) = sum(H(:,v+k:sizey:end),2);
end
H = [H(:,1:(nu-1)*sizey),HH];

%%% Form weighting matrix and uss matrix
MM=[];
for k=1:nu;
    v = (k-1)*sizey+1:k*sizey;
    RR(v,v)=R;
    MM=[MM;M];
end


%%%% Cost function
%% J = | L*r - P x- H ufut - L*offset |  + R |ufut - MM*(r-offset)|
%% J = ufut'*S*ufut + 2 ufut'*X*[x;r-offset] + k
%%   = ufut'(H'H+R)ufut   +  2ufut'* H'*P*x + 2ufut'*H'*L*(offset-r)
%%               - 2*ufut'R*MM*(r-offset) + k

S = H'*H+RR;
X1 = H'*P;
X2 = -H'*L-RR*MM;
X=[X1,X2];

%%%% Unconstrained control law
%%%%         ufut = -K*x  + Pr*(r-offset)     (No advance knowledge is used)
Mi=inv(S);
K = Mi*X1;
Pr = -Mi*X2;

