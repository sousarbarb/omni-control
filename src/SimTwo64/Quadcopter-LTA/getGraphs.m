close all;


log = load('xy.log')

log(:,3) = log(:,2);
log(:,2) = log(:,1);
log(:,1) = 0.04:0.04:200;

figure;
plot(log(:,1), log(:,2))
hold on;
grid on;
plot(log(:,1), log(:,3),'r')
title('2D Robot Position')
xlabel('Time (ms)') % x-axis label
ylabel('Distance (m)') % y-axis label