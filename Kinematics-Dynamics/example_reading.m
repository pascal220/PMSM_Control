%Reading data of specified name
name = 'validation Sin 1Hz 50Hz';
sample1 = readfstrm(name);
N = size(sample1.OutputData,1);
t = 0:sample1.Ts:N*sample1.Ts-sample1.Ts;

name = 'validation Sin 2Hz 49Hz';
sample2 = readfstrm(name);
name = 'validation Sin 3Hz 48Hz';
sample3 = readfstrm(name);
name = 'validation Sin 4Hz 47Hz';
sample4 = readfstrm(name);
name = 'validation Sin 5Hz 46Hz';
sample5 = readfstrm(name);
name = 'validation Sin 6Hz 45Hz';
sample6 = readfstrm(name);
name = 'validation Sin 7Hz 44Hz';
sample7 = readfstrm(name);
name = 'validation Sin 8Hz 43Hz';
sample8 = readfstrm(name);
name = 'validation Sin 9Hz 42Hz';
sample9 = readfstrm(name);
name = 'validation Sin 10Hz 41Hz';
sample10 = readfstrm(name);
name = 'validation Sin 11Hz 40Hz';
sample11 = readfstrm(name);
name = 'validation Sin 12Hz 39Hz';
sample12 = readfstrm(name);
name = 'validation Sin 13Hz 38Hz';
sample13 = readfstrm(name);
name = 'validation Sin 14Hz 37Hz';
sample14 = readfstrm(name);
name = 'validation Sin 15Hz 36Hz';
sample15 = readfstrm(name);
name = 'validation Sin 16Hz 35Hz';
sample16 = readfstrm(name);
name = 'validation Sin 17Hz 34Hz';
sample17 = readfstrm(name);
name = 'validation Sin 18Hz 33Hz';
sample18 = readfstrm(name);
name = 'validation Sin 19Hz 32Hz';
sample19 = readfstrm(name);
name = 'validation Sin 20Hz 31Hz';
sample20 = readfstrm(name);
name = 'validation Sin 21Hz 30Hz';
sample21 = readfstrm(name);
name = 'validation Sin 22Hz 29Hz';
sample22 = readfstrm(name);
name = 'validation Sin 23Hz 28Hz';
sample23 = readfstrm(name);
name = 'validation Sin 24Hz 27Hz';
sample24 = readfstrm(name);
name = 'validation Sin 25Hz 26Hz';
sample25 = readfstrm(name);

pos_ref = [sample1.OutputData(:,3);sample2.OutputData(:,3);sample3.OutputData(:,3);sample4.OutputData(:,3);sample5.OutputData(:,3);...
            sample6.OutputData(:,3);sample7.OutputData(:,3);sample8.OutputData(:,3);sample9.OutputData(:,3);sample10.OutputData(:,3);...
            sample11.OutputData(:,3);sample12.OutputData(:,3);sample13.OutputData(:,3);sample14.OutputData(:,3);sample15.OutputData(:,3);...
            sample16.OutputData(:,3);sample17.OutputData(:,3);sample18.OutputData(:,3);sample19.OutputData(:,3);sample20.OutputData(:,3);...
            sample21.OutputData(:,3);sample22.OutputData(:,3);sample23.OutputData(:,3);sample24.OutputData(:,3);sample25.OutputData(:,3)];

pos_feed = [sample1.OutputData(:,5);sample2.OutputData(:,5);sample3.OutputData(:,5);sample4.OutputData(:,5);sample5.OutputData(:,5);...
            sample6.OutputData(:,5);sample7.OutputData(:,5);sample8.OutputData(:,5);sample9.OutputData(:,5);sample10.OutputData(:,5);...
            sample11.OutputData(:,5);sample12.OutputData(:,5);sample13.OutputData(:,5);sample14.OutputData(:,5);sample15.OutputData(:,5);...
            sample16.OutputData(:,5);sample17.OutputData(:,5);sample18.OutputData(:,5);sample19.OutputData(:,5);sample20.OutputData(:,5);...
            sample21.OutputData(:,5);sample22.OutputData(:,5);sample23.OutputData(:,5);sample24.OutputData(:,5);sample25.OutputData(:,5)];
                  
% N = size(pos_ref);
% t_p = 0:sample1.Ts:N*sample1.Ts-sample1.Ts;  
%% Excel
name = 'validation Sin 1Hz 50Hz';
sample1_exe = xlsread(name);
name = 'validation Sin 2Hz 49Hz';
sample2_exe = xlsread(name);
name = 'validation Sin 3Hz 48Hz';
sample3_exe = xlsread(name);
name = 'validation Sin 4Hz 47Hz';
sample4_exe = xlsread(name);
name = 'validation Sin 5Hz 46Hz';
sample5_exe = xlsread(name);
name = 'validation Sin 6Hz 45Hz';
sample6_exe = xlsread(name);
name = 'validation Sin 7Hz 44Hz';
sample7_exe = xlsread(name);
name = 'validation Sin 8Hz 43Hz';
sample8_exe = xlsread(name);
name = 'validation Sin 9Hz 42Hz';
sample9_exe = xlsread(name);
name = 'validation Sin 10Hz 41Hz';
sample10_exe = xlsread(name);
name = 'validation Sin 11Hz 40Hz';
sample11_exe = xlsread(name);
name = 'validation Sin 12Hz 39Hz';
sample12_exe = xlsread(name);
name = 'validation Sin 13Hz 38Hz';
sample13_exe = xlsread(name);
name = 'validation Sin 14Hz 37Hz';
sample14_exe = xlsread(name);
name = 'validation Sin 15Hz 36Hz';
sample15_exe = xlsread(name);
name = 'validation Sin 16Hz 35Hz';
sample16_exe = xlsread(name);
name = 'validation Sin 17Hz 34Hz';
sample17_exe = xlsread(name);
name = 'validation Sin 18Hz 33Hz';
sample18_exe = xlsread(name);
name = 'validation Sin 19Hz 32Hz';
sample19_exe = xlsread(name);
name = 'validation Sin 20Hz 31Hz';
sample20_exe = xlsread(name);
name = 'validation Sin 21Hz 30Hz';
sample21_exe = xlsread(name);
name = 'validation Sin 22Hz 29Hz';
sample22_exe = xlsread(name);
name = 'validation Sin 23Hz 28Hz';
sample23_exe = xlsread(name);
name = 'validation Sin 24Hz 27Hz';
sample24_exe = xlsread(name);
name = 'validation Sin 25Hz 26Hz';
sample25_exe = xlsread(name);

%% Reading force fstream
exp1 = readfstrm('Sin 1Hz 50Hz 1');
exp2 = readfstrm('Sin 2Hz 49Hz 1');
exp3 = readfstrm('Sin 3Hz 48Hz 1');
exp4 = readfstrm('Sin 4Hz 47Hz 1');
exp5 = readfstrm('Sin 5Hz 46Hz 1');
exp6 = readfstrm('Sin 6Hz 45Hz 1');
exp7 = readfstrm('Sin 7Hz 44Hz 1');
exp8 = readfstrm('Sin 8Hz 43Hz 1');
exp9 = readfstrm('Sin 9Hz 42Hz 1');
exp10 = readfstrm('Sin 10Hz 41Hz 1');
exp11 = readfstrm('Sin 11Hz 40Hz 1');
exp12 = readfstrm('Sin 12Hz 39Hz 1');
exp13 = readfstrm('Sin 13Hz 38Hz 1');
exp14 = readfstrm('Sin 14Hz 37Hz 1');
exp15 = readfstrm('Sin 15Hz 36Hz 1');
exp16 = readfstrm('Sin 16Hz 35Hz 1');
exp17 = readfstrm('Sin 17Hz 34Hz 1');
exp18 = readfstrm('Sin 18Hz 33Hz 1');
exp19 = readfstrm('Sin 19Hz 32Hz 1');
exp20 = readfstrm('Sin 20Hz 31Hz 1');
exp21 = readfstrm('Sin 21Hz 30Hz 1');
exp22 = readfstrm('Sin 22Hz 29Hz 1');
exp23 = readfstrm('Sin 23Hz 28Hz 1');
exp24 = readfstrm('Sin 24Hz 27Hz 1');
exp25 = readfstrm('Sin 25Hz 26Hz 1');

N = size(exp1.OutputData,1);
t = 0:exp1.Ts:N*exp1.Ts-exp1.Ts;

exp1_exe = xlsread('Sin 1Hz 50Hz 1');
exp2_exe = xlsread('Sin 2Hz 49Hz 1');
exp3_exe = xlsread('Sin 3Hz 48Hz 1');
exp4_exe = xlsread('Sin 4Hz 47Hz 1');
exp5_exe = xlsread('Sin 5Hz 46Hz 1');
exp6_exe = xlsread('Sin 6Hz 45Hz 1');
exp7_exe = xlsread('Sin 7Hz 44Hz 1');
exp8_exe = xlsread('Sin 8Hz 43Hz 1');
exp9_exe = xlsread('Sin 9Hz 42Hz 1');
exp10_exe = xlsread('Sin 10Hz 41Hz 1');
exp11_exe = xlsread('Sin 11Hz 40Hz 1');
exp12_exe = xlsread('Sin 12Hz 39Hz 1');
exp13_exe = xlsread('Sin 13Hz 38Hz 1');
exp14_exe = xlsread('Sin 14Hz 37Hz 1');
exp15_exe = xlsread('Sin 15Hz 36Hz 1');
exp16_exe = xlsread('Sin 16Hz 35Hz 1');
exp17_exe = xlsread('Sin 17Hz 34Hz 1');
exp18_exe = xlsread('Sin 18Hz 33Hz 1');
exp19_exe = xlsread('Sin 19Hz 32Hz 1');
exp20_exe = xlsread('Sin 20Hz 31Hz 1');
exp21_exe = xlsread('Sin 21Hz 30Hz 1');
exp22_exe = xlsread('Sin 22Hz 29Hz 1');
exp23_exe = xlsread('Sin 23Hz 28Hz 1');
exp24_exe = xlsread('Sin 24Hz 27Hz 1');
exp25_exe = xlsread('Sin 25Hz 26Hz 1');