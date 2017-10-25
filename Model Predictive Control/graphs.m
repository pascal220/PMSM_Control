figure
subplot(2,1,1);
plot(Sin_50_con.time,Sin_50_con.signals(1).values);
xlabel('Time')
ylabel('position/mm')
title('Input to Position Output')
subplot(2,1,2);
plot(Sin_50_con.time,Sin_50_con.signals(2).values,'g');
xlabel('Time')
ylabel('froce/N')
title('Input to Force Output')