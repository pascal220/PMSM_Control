function [ noise ] = read_noise()

noise1 = xlsread('Noise1 active');
noise2 = xlsread('Noise2 active');
noise3 = xlsread('Noise3 active');
noise4 = xlsread('Noise4 active');
noise5 = xlsread('Noise5 active');
noise6 = xlsread('Noise6 active');
noise7 = xlsread('Noise7 active');
noise8 = xlsread('Noise8 active');
noise9 = xlsread('Noise9 active');
noise10 = xlsread('Noise10 active');

noise =[noise1;noise2;noise3;noise4;noise5;...
        noise6;noise7;noise8;noise9;noise10];

end