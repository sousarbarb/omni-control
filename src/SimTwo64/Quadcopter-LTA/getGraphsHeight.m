close all;


log = load('altura.log')

log(:,2) = log(:,1);
log(:,1) = 0.04:0.04:200;

figure;
plot(log(:,1), log(:,2))
hold on;
grid on;

title('Height Robot Position')
xlabel('Time (ms)') % x-axis label
ylabel('Height (m)') % y-axis label