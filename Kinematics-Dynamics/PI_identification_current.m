K = 19;
Tn = 1.8e-3;

exp1 = xlsread('White noise 8c1');
exp2 = xlsread('White noise 8c2');
exp3 = xlsread('White noise 8c3');
exp4 = xlsread('White noise 8c4');
exp5 = xlsread('White noise 8c5');
exp6 = xlsread('White noise 8c6');
exp7 = xlsread('White noise 8c7');
exp8 = xlsread('White noise 8c8');
exp9 = xlsread('White noise 8c9');
exp10 = xlsread('White noise 8c10');
exp11 = xlsread('Sin 1Hz 50Hz 8');
exp12 = xlsread('Sin 2Hz 49Hz 8');
exp13 = xlsread('Sin 3Hz 48Hz 8');
exp14 = xlsread('Sin 4Hz 47Hz 8');
exp15 = xlsread('Sin 5Hz 46Hz 8');
exp16 = xlsread('Sin 6Hz 45Hz 8');
exp17 = xlsread('Sin 7Hz 44Hz 8');
exp18 = xlsread('Sin 8Hz 43Hz 8');
exp19 = xlsread('Sin 9Hz 42Hz 8');
exp20 = xlsread('Sin 10Hz 41Hz 8');
exp21 = xlsread('Sin 11Hz 40Hz 8');
exp22 = xlsread('Sin 12Hz 39Hz 8');
exp23 = xlsread('Sin 13Hz 38Hz 8');
exp24 = xlsread('Sin 14Hz 37Hz 8');
exp25 = xlsread('Sin 15Hz 36Hz 8');
exp26 = xlsread('Sin 16Hz 35Hz 8');
exp27 = xlsread('Sin 17Hz 34Hz 8');
exp28 = xlsread('Sin 18Hz 33Hz 8');
exp29 = xlsread('Sin 19Hz 32Hz 8');
exp30 = xlsread('Sin 20Hz 31Hz 8');
exp31 = xlsread('Sin 21Hz 30Hz 8');
exp32 = xlsread('Sin 22Hz 29Hz 8');
exp33 = xlsread('Sin 23Hz 28Hz 8');
exp34 = xlsread('Sin 24Hz 27Hz 8');
exp35 = xlsread('Sin 25Hz 26Hz 8');
exp36 = xlsread('White noise1 8c1 v2');
exp37 = xlsread('White noise1 8c2 v2');
exp38 = xlsread('White noise1 8c3 v2');
exp39 = xlsread('White noise1 8c4 v2');
exp40 = xlsread('White noise1 8c5 v2');
exp41 = xlsread('White noise1 8c6 v2');
exp42 = xlsread('White noise1 8c7 v2');
exp43 = xlsread('White noise1 8c8 v2');
exp44 = xlsread('White noise1 8c9 v2');
exp45 = xlsread('White noise1 8c10 v2');
exp46 = xlsread('Sin 1Hz 50Hz 8 v2');
exp47 = xlsread('Sin 2Hz 49Hz 8 v2');
exp48 = xlsread('Sin 3Hz 48Hz 8 v2');
exp49 = xlsread('Sin 4Hz 47Hz 8 v2');
exp50 = xlsread('Sin 5Hz 46Hz 8 v2');
exp51 = xlsread('Sin 6Hz 45Hz 8 v2');
exp52 = xlsread('Sin 7Hz 44Hz 8 v2');
exp53 = xlsread('Sin 8Hz 43Hz 8 v2');
exp54 = xlsread('Sin 9Hz 42Hz 8 v2');
exp55 = xlsread('Sin 10Hz 41Hz 8 v2');
exp56 = xlsread('Sin 11Hz 40Hz 8 v2');
exp57 = xlsread('Sin 12Hz 39Hz 8 v2');
exp58 = xlsread('Sin 13Hz 38Hz 8 v2');
exp59 = xlsread('Sin 14Hz 37Hz 8 v2');
exp60 = xlsread('Sin 15Hz 36Hz 8 v2');
exp61 = xlsread('Sin 16Hz 35Hz 8 v2');
exp62 = xlsread('Sin 17Hz 34Hz 8 v2');
exp63 = xlsread('Sin 18Hz 33Hz 8 v2');
exp64 = xlsread('Sin 19Hz 32Hz 8 v2');
exp65 = xlsread('Sin 20Hz 31Hz 8 v2');
exp66 = xlsread('Sin 21Hz 30Hz 8 v2');
exp67 = xlsread('Sin 22Hz 29Hz 8 v2');
exp68 = xlsread('Sin 23Hz 28Hz 8 v2');
exp69 = xlsread('Sin 24Hz 27Hz 8 v2');
exp70 = xlsread('Sin 25Hz 26Hz 8 v2');


u_ref = [exp1(1:end,4); exp2(1:end,4); exp3(1:end,4); exp4(1:end,4);...
    exp5(1:end,4); exp6(1:end,4); exp7(1:end,4); exp8(1:end,4);...
    exp9(1:end,4); exp10(1:end,4); exp11(1:end,4); exp12(1:end,4);...
    exp13(1:end,4); exp14(1:end,4); exp15(1:end,4); exp16(1:end,4);...
    exp17(1:end,4); exp18(1:end,4); exp19(1:end,4); exp20(1:end,4);...
    exp21(1:end,4); exp22(1:end,4); exp23(1:end,4); exp24(1:end,4);...
    exp25(1:end,4);exp26(1:end,4); exp27(1:end,4); exp28(1:end,4);...
    exp29(1:end,4);exp30(1:end,4);exp31(1:end,4); exp32(1:end,4);...
    exp33(1:end,4); exp34(1:end,4); exp35(1:end,4)];

u_feed = [exp1(1:end,5); exp2(1:end,5); exp3(1:end,5); exp4(1:end,5);...
    exp5(1:end,5); exp6(1:end,5); exp7(1:end,5); exp8(1:end,5);...
    exp9(1:end,5); exp10(1:end,5); exp11(1:end,5); exp12(1:end,5);...
    exp13(1:end,5); exp14(1:end,5); exp15(1:end,5); exp16(1:end,5);...
    exp17(1:end,5); exp18(1:end,5); exp19(1:end,5); exp20(1:end,5);...
    exp21(1:end,5); exp22(1:end,5); exp23(1:end,5); exp24(1:end,5);...
    exp25(1:end,5);exp26(1:end,5); exp27(1:end,5); exp28(1:end,5);...
    exp29(1:end,5);exp30(1:end,5);exp31(1:end,5); exp32(1:end,5);...
    exp33(1:end,5); exp34(1:end,5); exp35(1:end,5)];

y = [exp1(1:end,6); exp2(1:end,6); exp3(1:end,6); exp4(1:end,6);...
    exp5(1:end,6); exp6(1:end,6); exp7(1:end,6); exp8(1:end,6);...
    exp9(1:end,6); exp10(1:end,6); exp11(1:end,6); exp12(1:end,6);...
    exp13(1:end,6); exp14(1:end,6); exp15(1:end,6); exp16(1:end,6);...
    exp17(1:end,6); exp18(1:end,6); exp19(1:end,6); exp20(1:end,6);...
    exp21(1:end,6); exp22(1:end,6); exp23(1:end,6); exp24(1:end,6);...
    exp25(1:end,6);exp26(1:end,6); exp27(1:end,6); exp28(1:end,6);...
    exp29(1:end,6);exp30(1:end,6);exp31(1:end,6); exp32(1:end,6);...
    exp33(1:end,6); exp34(1:end,6); exp35(1:end,6)];

e = u_ref - u_feed;
Ts = 1.25e-04;
data = iddata(y,e,Ts);
opt = tfestOptions('Display','on');
 
t = 0:Ts:(length(y)*Ts)-Ts;
%% System
% syms c1_i c2_i c3_i c4_i
% Api_i = -c4_i/c3_i;
% Bpi_i = 1;
% Cpi_i = (c3_i*c2_i - c1_i*c4_i)/(c3_i^2);
% Dpi_i = c1_i/c3_i;

%% Controller estimation
% H_pi_i1 =  tfest(data,1,1,opt);

% c1_i = 29.026225801364696;
% c2_i = 5.148208865195679e+04;
% c3_i = 1;
% c4_i = 2.936123024877891e+03;
 
% H_pi_i2 =  tf([c1_i c2_i],[c3_i c4_i]);

H_pi =  tf([50 1],[0.001 0]);
%% Validation
figure
compare(data,H_pi);

% figure(2)
% bode(H_pi_i,H_pi_i1,H_pi_i2);