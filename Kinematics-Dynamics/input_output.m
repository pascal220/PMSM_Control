function [y , u, t_sample] = input_output(k)

    if k == 1 % Current 
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

        u_q = [exp1(1:end,4); exp2(1:end,4); exp3(1:end,4); exp4(1:end,4);...
            exp5(1:end,4); exp6(1:end,4); exp7(1:end,4); exp8(1:end,4);...
            exp9(1:end,4); exp10(1:end,4); exp11(1:end,4); exp12(1:end,4);...
            exp13(1:end,4); exp14(1:end,4); exp15(1:end,4); exp16(1:end,4);...
            exp17(1:end,4); exp18(1:end,4); exp19(1:end,4); exp20(1:end,4);...
            exp21(1:end,4); exp22(1:end,4); exp23(1:end,4); exp24(1:end,4);...
            exp25(1:end,4);exp26(1:end,4); exp27(1:end,4); exp28(1:end,4);...
            exp29(1:end,4);exp30(1:end,4);exp31(1:end,4); exp32(1:end,4);...
            exp33(1:end,4); exp34(1:end,4); exp35(1:end,4);...
            exp36(1:end,4); exp37(1:end,4); exp38(1:end,4); exp39(1:end,4);...
            exp40(1:end,4); exp41(1:end,4); exp42(1:end,4); exp43(1:end,4);...
            exp44(1:end,4); exp45(1:end,4); exp46(1:end,4); exp47(1:end,4);...
            exp48(1:end,4); exp49(1:end,4); exp50(1:end,4); exp51(1:end,4);...
            exp52(1:end,4); exp53(1:end,4); exp54(1:end,4); exp55(1:end,4);...
            exp56(1:end,4); exp57(1:end,4); exp58(1:end,4); exp59(1:end,4);...
            exp60(1:end,4);exp61(1:end,4); exp62(1:end,4); exp63(1:end,4);...
            exp64(1:end,4);exp65(1:end,4);exp66(1:end,4); exp67(1:end,4);...
            exp68(1:end,4); exp69(1:end,4); exp70(1:end,4)];

        y_q = [exp1(1:end,5); exp2(1:end,5); exp3(1:end,5); exp4(1:end,5);...
            exp5(1:end,5); exp6(1:end,5); exp7(1:end,5); exp8(1:end,5);...
            exp9(1:end,5); exp10(1:end,5); exp11(1:end,5); exp12(1:end,5);...
            exp13(1:end,5); exp14(1:end,5); exp15(1:end,5); exp16(1:end,5);...
            exp17(1:end,5); exp18(1:end,5); exp19(1:end,5); exp20(1:end,5);...
            exp21(1:end,5); exp22(1:end,5); exp23(1:end,5); exp24(1:end,5);...
            exp25(1:end,5);exp26(1:end,5); exp27(1:end,5); exp28(1:end,5);...
            exp29(1:end,5);exp30(1:end,5);exp31(1:end,5); exp32(1:end,5);...
            exp33(1:end,5); exp34(1:end,5); exp35(1:end,5);...
            exp36(1:end,5); exp37(1:end,5); exp38(1:end,5); exp39(1:end,5);...
            exp40(1:end,5); exp41(1:end,5); exp42(1:end,5); exp43(1:end,5);...
            exp44(1:end,5); exp45(1:end,5); exp46(1:end,5); exp47(1:end,5);...
            exp48(1:end,5); exp49(1:end,5); exp50(1:end,5); exp51(1:end,5);...
            exp52(1:end,5); exp53(1:end,5); exp54(1:end,5); exp55(1:end,5);...
            exp56(1:end,5); exp57(1:end,5); exp58(1:end,5); exp59(1:end,5);...
            exp60(1:end,5);exp61(1:end,5); exp62(1:end,5); exp63(1:end,5);...
            exp64(1:end,5);exp65(1:end,5);exp66(1:end,5); exp67(1:end,5);...
            exp68(1:end,5); exp69(1:end,5); exp70(1:end,5)];

        t_sample = 1.25e-04;
        u_d = zeros(size(u_q));
        y_d = zeros(size(y_q));
        
        
        u = [u_d u_q];
        y = [y_d y_q];

    elseif k == 2 % Velocity
        exp1 = xlsread('White noise 1v1');
        exp2 = xlsread('White noise 1v2');
        exp3 = xlsread('White noise 1v3');
        exp4 = xlsread('White noise 1v5');
        exp5 = xlsread('White noise 1v5');
        exp6 = xlsread('White noise 1v6');
        exp7 = xlsread('White noise 1v7');
        exp8 = xlsread('White noise 1v8');
        exp9 = xlsread('White noise 1v9');
        exp10 = xlsread('White noise 1v10');
        exp11 = xlsread('Sin 1Hz 50Hz 1');
        exp12 = xlsread('Sin 2Hz 49Hz 1');
        exp13 = xlsread('Sin 3Hz 48Hz 1');
        exp14 = xlsread('Sin 4Hz 47Hz 1');
        exp15 = xlsread('Sin 5Hz 46Hz 1');
        exp16 = xlsread('Sin 6Hz 45Hz 1');
        exp17 = xlsread('Sin 7Hz 44Hz 1');
        exp18 = xlsread('Sin 8Hz 43Hz 1');
        exp19 = xlsread('Sin 9Hz 42Hz 1');
        exp20 = xlsread('Sin 10Hz 41Hz 1');
        exp21 = xlsread('Sin 11Hz 40Hz 1');
        exp22 = xlsread('Sin 12Hz 39Hz 1');
        exp23 = xlsread('Sin 13Hz 38Hz 1');
        exp24 = xlsread('Sin 14Hz 37Hz 1');
        exp25 = xlsread('Sin 15Hz 36Hz 1');
        exp26 = xlsread('Sin 16Hz 35Hz 1');
        exp27 = xlsread('Sin 17Hz 34Hz 1');
        exp28 = xlsread('Sin 18Hz 33Hz 1');
        exp29 = xlsread('Sin 19Hz 32Hz 1');
        exp30 = xlsread('Sin 20Hz 31Hz 1');
        exp31 = xlsread('Sin 21Hz 30Hz 1');
        exp32 = xlsread('Sin 22Hz 29Hz 1');
        exp33 = xlsread('Sin 23Hz 28Hz 1');
        exp34 = xlsread('Sin 24Hz 27Hz 1');
        exp35 = xlsread('Sin 25Hz 26Hz 1');
        exp36 = xlsread('White noise v1 1');
        exp37 = xlsread('White noise v2 1');
        exp38 = xlsread('White noise v3 1');
        exp39 = xlsread('White noise v4 1');
        exp40 = xlsread('White noise v5 1');
        exp41 = xlsread('Sin 1Hz 1');
        exp42 = xlsread('Sin 10Hz 1');
        exp43 = xlsread('Sin 20Hz 1');
        exp44 = xlsread('Sin 30Hz 1');
        exp45 = xlsread('Sin 40Hz 1');
        exp46 = xlsread('Sin 50Hz 1');
        
        u = [exp1(1:end,2); exp2(1:end,2); exp3(1:end,2); exp4(1:end,2);...
            exp5(1:end,2); exp6(1:end,2); exp7(1:end,2); exp8(1:end,2);...
            exp9(1:end,2); exp10(1:end,2); exp11(1:end,2); exp12(1:end,2);...
            exp13(1:end,2); exp14(1:end,2); exp15(1:end,2); exp16(1:end,2);...
            exp17(1:end,2); exp18(1:end,2); exp19(1:end,2); exp20(1:end,2);...
            exp21(1:end,2); exp22(1:end,2); exp23(1:end,2); exp24(1:end,2);...
            exp25(1:end,2);exp26(1:end,2); exp27(1:end,2); exp28(1:end,2);...
            exp29(1:end,2);exp30(1:end,2); exp31(1:end,2); exp32(1:end,2);...
            exp33(1:end,2); exp34(1:end,2); exp35(1:end,2);...
            exp36(1:end,2);exp37(1:end,2); exp38(1:end,2); exp39(1:end,2);...
            exp40(1:end,2); exp41(1:end,2); exp42(1:end,2); exp43(1:end,2);...
            exp44(1:end,2); exp45(1:end,2); exp46(1:end,2)];

        y = [exp1(1:end,3); exp2(1:end,3); exp3(1:end,3); exp4(1:end,3);...
            exp5(1:end,3); exp6(1:end,3); exp7(1:end,3); exp8(1:end,3);...
            exp9(1:end,3); exp10(1:end,3); exp11(1:end,3); exp12(1:end,3);...
            exp13(1:end,3); exp14(1:end,3); exp15(1:end,3); exp16(1:end,3);...
            exp17(1:end,3); exp18(1:end,3); exp19(1:end,3); exp20(1:end,3);...
            exp21(1:end,3); exp22(1:end,3); exp23(1:end,3); exp24(1:end,3);...
            exp25(1:end,3);exp26(1:end,3); exp27(1:end,3); exp28(1:end,3);...
            exp29(1:end,3);exp30(1:end,3);exp31(1:end,3); exp32(1:end,3);...
            exp33(1:end,3); exp34(1:end,3); exp35(1:end,3);...
            exp36(1:end,3);exp37(1:end,3); exp38(1:end,3); exp39(1:end,3);...
            exp40(1:end,3); exp41(1:end,3); exp42(1:end,3); exp43(1:end,3);...
            exp44(1:end,3); exp45(1:end,3); exp46(1:end,3)];
                             
        t_sample = 1e-03;
            
    elseif k == 3; % Position
            name = 'validation Sin 1Hz 50Hz';
            sample1 = readfstrm(name);
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

            u = [sample1.OutputData(:,3);sample2.OutputData(:,3);sample3.OutputData(:,3);sample4.OutputData(:,3);sample5.OutputData(:,3);...
                sample6.OutputData(:,3);sample7.OutputData(:,3);sample8.OutputData(:,3);sample9.OutputData(:,3);sample10.OutputData(:,3);...
                sample11.OutputData(:,3);sample12.OutputData(:,3);sample13.OutputData(:,3);sample14.OutputData(:,3);sample15.OutputData(:,3);...
                sample16.OutputData(:,3);sample17.OutputData(:,3);sample18.OutputData(:,3);sample19.OutputData(:,3);sample20.OutputData(:,3);...
                sample21.OutputData(:,3);sample22.OutputData(:,3);sample23.OutputData(:,3);sample24.OutputData(:,3);sample25.OutputData(:,3)];

            y = [sample1.OutputData(:,5);sample2.OutputData(:,5);sample3.OutputData(:,5);sample4.OutputData(:,5);sample5.OutputData(:,5);...
                sample6.OutputData(:,5);sample7.OutputData(:,5);sample8.OutputData(:,5);sample9.OutputData(:,5);sample10.OutputData(:,5);...
                sample11.OutputData(:,5);sample12.OutputData(:,5);sample13.OutputData(:,5);sample14.OutputData(:,5);sample15.OutputData(:,5);...
                sample16.OutputData(:,5);sample17.OutputData(:,5);sample18.OutputData(:,5);sample19.OutputData(:,5);sample20.OutputData(:,5);...
                sample21.OutputData(:,5);sample22.OutputData(:,5);sample23.OutputData(:,5);sample24.OutputData(:,5);sample25.OutputData(:,5)];
            
            t_sample = sample1.Ts;
    else 
        sample1 = readfstrm('Sin 1Hz 50Hz 1');
        sample2 = readfstrm('Sin 2Hz 49Hz 1');
        sample3 = readfstrm('Sin 3Hz 48Hz 1');
        sample4 = readfstrm('Sin 4Hz 47Hz 1');
        sample5 = readfstrm('Sin 5Hz 46Hz 1');
        sample6 = readfstrm('Sin 6Hz 45Hz 1');
        sample7 = readfstrm('Sin 7Hz 44Hz 1');
        sample8 = readfstrm('Sin 8Hz 43Hz 1');
        sample9 = readfstrm('Sin 9Hz 42Hz 1');
        sample10 = readfstrm('Sin 10Hz 41Hz 1');
        sample11 = readfstrm('Sin 11Hz 40Hz 1');
        sample12 = readfstrm('Sin 12Hz 39Hz 1');
        sample13 = readfstrm('Sin 13Hz 38Hz 1');
        sample14 = readfstrm('Sin 14Hz 37Hz 1');
        sample15 = readfstrm('Sin 15Hz 36Hz 1');
        sample16 = readfstrm('Sin 16Hz 35Hz 1');
        sample17 = readfstrm('Sin 17Hz 34Hz 1');
        sample18 = readfstrm('Sin 18Hz 33Hz 1');
        sample19 = readfstrm('Sin 19Hz 32Hz 1');
        sample20 = readfstrm('Sin 20Hz 31Hz 1');
        sample21 = readfstrm('Sin 21Hz 30Hz 1');
        sample22 = readfstrm('Sin 22Hz 29Hz 1');
        sample23 = readfstrm('Sin 23Hz 28Hz 1');
        sample24 = readfstrm('Sin 24Hz 27Hz 1');
        sample25 = readfstrm('Sin 25Hz 26Hz 1');
        
        u = [sample1.OutputData(:,3);sample2.OutputData(:,3);sample3.OutputData(:,3);sample4.OutputData(:,3);sample5.OutputData(:,3);...
            sample6.OutputData(:,3);sample7.OutputData(:,3);sample8.OutputData(:,3);sample9.OutputData(:,3);sample10.OutputData(:,3);...
            sample11.OutputData(:,3);sample12.OutputData(:,3);sample13.OutputData(:,3);sample14.OutputData(:,3);sample15.OutputData(:,3);...
            sample16.OutputData(:,3);sample17.OutputData(:,3);sample18.OutputData(:,3);sample19.OutputData(:,3);sample20.OutputData(:,3);...
            sample21.OutputData(:,3);sample22.OutputData(:,3);sample23.OutputData(:,3);sample24.OutputData(:,3);sample25.OutputData(:,3)];

        y = [sample1.OutputData(:,5);sample2.OutputData(:,5);sample3.OutputData(:,5);sample4.OutputData(:,5);sample5.OutputData(:,5);...
            sample6.OutputData(:,5);sample7.OutputData(:,5);sample8.OutputData(:,5);sample9.OutputData(:,5);sample10.OutputData(:,5);...
            sample11.OutputData(:,5);sample12.OutputData(:,5);sample13.OutputData(:,5);sample14.OutputData(:,5);sample15.OutputData(:,5);...
            sample16.OutputData(:,5);sample17.OutputData(:,5);sample18.OutputData(:,5);sample19.OutputData(:,5);sample20.OutputData(:,5);...
            sample21.OutputData(:,5);sample22.OutputData(:,5);sample23.OutputData(:,5);sample24.OutputData(:,5);sample25.OutputData(:,5)];
        
        t_sample = sample1.Ts;
    end
end