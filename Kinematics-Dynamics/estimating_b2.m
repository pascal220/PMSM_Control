%% Finding b2
clear all;clc;
load('b2Estimation.mat') 
x = Angle.signals.values;
t = Angle.time;

figure(1001)
plot(t,x)
xlabel('time')
ylabel('Angle of Oscilation')

p1 = mean(x(425:435))
p2 = mean(x(633:643))
sigma = log(p1/p2)/2
zeta = 1/sqrt((1 + (2*pi/sigma)^2))