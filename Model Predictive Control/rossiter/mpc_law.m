%%%%   Compute MPC control law given the prediction matrices  
%%%%   Assumes: (i)   u constant after nu steps
%%%%            (ii)  predictions given as
%%%%                     y = H*Du(future) + P*Du(past) + Q*y(past)   
%%%%            (iii) weights on cost are Wy (outputs), Wu (inputs)
%%%%            (iv)  sizey is the number of outputs
%%%%
%%%%   Performance index is given as 
%%%%     J = Du(future)' S Du(future) + Du(future)'*2X*[Du(past);y(past);r]
%%%%   
%%%%   Control law is given as 
%%%%          Du(future) = Pr*r - Dk*Du(past) - Nk*y(past) 
%%%%
%%%% Pr is given as a simple gain (edit this code to reinstitute advance knowledge)
%%%%
%%%%            [Nk,Dk,Pr,S,X] =mpc_law(H,P,Q,nu,Wu,Wy,sizey)
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function  [Nk,Dk,Pr,S,X] =mpc_law(H,P,Q,nu,Wu,Wy,sizey)


%%%% Control horizon
P1 = H(:,1:sizey*nu);

%% Set up weighting matrices

WY=Wy;
WU=Wu;
L = eye(sizey);
npred = size(P1,1)/sizey;
for i = 2:npred;
   v=(i-1)*sizey+1:i*sizey; 
   WY(v,v) = Wy;
   WU(v,v) = Wu;
   L = [L;eye(sizey)];
 end
 WU = WU(1:nu*sizey,1:nu*sizey);
 
%%% Define performance index parameters
S = P1'*WY*P1 + WU;
X = [P1'*WY*P,P1'*WY*Q,-P1'*WY];
   

%%%% Define the control law parameters
M = inv(S);
Nk = M*P1'*WY*Q;
Dk = M*P1'*WY*P;
Pr = M*P1'*WY;

%%%%% Remove advance knowledge on the set point
Pr = Pr*L;
X = [P1'*WY*P,P1'*WY*Q,-P1'*WY*L];

