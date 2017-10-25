exp1 = xlsread('Noise1 active');
exp2 = xlsread('Noise2 active');
exp3 = xlsread('Noise3 active');
exp4 = xlsread('Noise4 active');
exp5 = xlsread('Noise5 active');
exp6 = xlsread('Noise6 active');
exp7 = xlsread('Noise7 active');
exp8 = xlsread('Noise8 active');
exp9 = xlsread('Noise9 active');
exp10 = xlsread('Noise10 active');

 x = [exp1(84:end,5); exp2(84:end,5); exp3(84:end,5); exp4(84:end,5);...
      exp5(84:end,5); exp6(84:end,5); exp7(84:end,5); exp8(84:end,5);...
      exp9(84:end,5); exp10(84:end,5)];

Fs = length(x);

xdft = fft(x);
xdft = xdft(1:N/2+1);
psdx = (1/(Fs^2)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(x):Fs/2;

figure (1)
plot(freq,10*log10(psdx))
grid on
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dB/Hz)')

figure(2)
periodogram(x,rectwin(length(x)),length(x),Fs)

cos_kurna_pojebanego = periodogram(x,rectwin(length(x)),length(x),Fs);
mxerr = max(psdx-cos_kurna_pojebanego);