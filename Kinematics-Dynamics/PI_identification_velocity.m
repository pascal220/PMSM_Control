K = 0.12;
Tn = 10e-3;

        exp1 = xlsread('Sin1');
        exp2 = xlsread('Sin2');
        exp3 = xlsread('Sin3');
        exp4 = xlsread('Sin4');
        exp5 = xlsread('Sin5');
        exp6 = xlsread('Sin6');
        exp7 = xlsread('White for vel 1');
        exp8 = xlsread('White for vel 2');
        exp9 = xlsread('White for vel 3');
        exp10 = xlsread('White for vel 4');
        exp11 = xlsread('White for vel 5');
        exp12 = xlsread('White for vel 6');
        exp13 = xlsread('White for vel 7');
        exp14 = xlsread('White for vel 8');
        exp15 = xlsread('White for vel 9');
        exp16 = xlsread('White for vel 10');
        
        u_ref = [exp1(1:end,2); exp2(1:end,2); exp3(1:end,2); exp4(1:end,2);...
            exp5(1:end,2); exp6(1:end,2); exp7(1:end,2); exp8(1:end,2);...
            exp9(1:end,2); exp10(1:end,2); exp11(1:end,2); exp12(1:end,2);...
            exp13(1:end,2); exp14(1:end,2); exp15(1:end,2); exp16(1:end,2)];
     
        u_feed = [exp1(1:end,3); exp2(1:end,3); exp3(1:end,3); exp4(1:end,3);...
            exp5(1:end,3); exp6(1:end,3); exp7(1:end,3); exp8(1:end,3);...
            exp9(1:end,3); exp10(1:end,3); exp11(1:end,3); exp12(1:end,3);...
            exp13(1:end,3); exp14(1:end,3); exp15(1:end,3); exp16(1:end,3)];
        
        y = [exp1(1:end,4); exp2(1:end,4); exp3(1:end,4); exp4(1:end,4);...
            exp5(1:end,4); exp6(1:end,4); exp7(1:end,4); exp8(1:end,4);...
            exp9(1:end,4); exp10(1:end,4); exp11(1:end,4); exp12(1:end,4);...
            exp13(1:end,4); exp14(1:end,4); exp15(1:end,4); exp16(1:end,4)];

e = u_ref - u_feed;
opt = tfestOptions('Display','on');
Ts = 1e-03;

t = 0:Ts:(length(y)*Ts)-Ts;
%% Estimation
% data = iddata(e,y,Ts);

% H_pi_vel =  tfest(data,1,1,opt);
% ss_vel = ssest(data,5,opt);
 
% c1_v = 7.813;
% c2_v = 4.867;
% c3_v = 1;
% c4_v = 100.4;
 
% H_pi_vel2 =  tf([c1_v c2_v],[c3_v c4_v]);

H_pi =  tf([0.01 1],[0.01 0]);

%% Validation
figure
compare(data,H_pi);

% figure(2)
% bode(H_pi_vel,H_pi_vel1,H_pi_vel2);