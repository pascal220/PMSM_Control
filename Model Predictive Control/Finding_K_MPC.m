% R = 0.001*eye(10);
% 
% Q = zeros(100,100);
% for i = 1:2:100
%     Q(i,i) = 100;
% end
% 
% syms A B C
% Phi = [C*B, zeros(2,9); zeros(98,10)];
% 
% H = Phi'*Q*Phi+R;
% 
% K_mpc = [1 0 0 0 0 0 0 0 0 0]*inv(H)*Phi'*Q;
K_mpc = 1\(B_dz'*C_dz');