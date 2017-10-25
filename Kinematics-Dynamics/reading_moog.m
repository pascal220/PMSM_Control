function [ output_args ] = Untitled3( input_args )
% We are reading the Moog .fstrm file
name = 'Sin 6Hz and 13Hz';
data = readfstrm(name);
N = size(data.OutputData,1);
% t_all = 0:data.Ts:N*data.Ts-data.Ts;

u_pos(1,1) = data.OutputData(1,4);
c = 2;
for i = 10:10:7360    
    u_pos(c,1) = data.OutputData(i,4);
    c = c+1;
end


end

