%%% To find the prediction matrices with a T-filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    yfut = H*Dufut + Pt*Dut + Qt*yt
%%%
%%%      Dut = Du/Tfilt   yt = y/Tfilt
%%% 
%%%  GIVEN    yfut = H *Dufut + P*Dupast + Q*ypast
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%       [Pt,Qt] = mpc_predtfilt(H,P,Q,Tfilt,sizey,ny);
%%%
%%%  sizey is the dimension of y
%%%  ny is the output horizon
%%%  Tfilt - parameters of the T-filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)



function [Pt,Qt] = mpc_predtfilt(H,P,Q,Tfilt,sizey,ny);

[Ct,Ht] = caha(Tfilt,sizey,ny);
Pt = Ct*P;
Qt =Ct*Q;
nT = size(Tfilt,2)/sizey;   %% Therefore Ht has nT-1 block columns
np2 = size(Pt,2)/sizey;
np3 = size(Qt,2)/sizey;


if np2<nT-1; Pt(1,sizey*(nT-1))=0;end
if np3<nT-1; Qt(1,sizey*(nT-1))=0;end
Pt(:,1:sizey*(nT-1)) = Pt(:,1:sizey*(nT-1)) - H*Ht;
Qt(:,1:sizey*(nT-1)) = Qt(:,1:sizey*(nT-1)) + Ht;

