%  function [CA,HA] = caha(A,sizey,n)
%%
%%   To find Toeplitz matrix Ca and hankel matrix Ha 
%%   with Ca having n block rows
%%   Assume n > order(A)
%%   sizey is the dimension of implied A(z)
%%  
%% Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)

function [Ca,Ha] = caha(A,sizey,n)

na = size(A,2)/sizey;
nA = size(A,2);
for i=1:na;
   Av((i-1)*sizey+1:i*sizey,1:sizey) = A(:,(i-1)*sizey+1:i*sizey);
end

for i=1:n;
   v=(i-1)*sizey;
   Ca(v+1:v+nA,v+1:v+sizey)=Av;
end
Ca = Ca(1:n*sizey,:);

Ha = zeros(n*sizey,sizey);

for i=1:na-1;
   v = (i-1)*sizey+1:i*sizey;
   Ha(v,1:(na-i)*sizey) = A(:,i*sizey+1:nA);
end



