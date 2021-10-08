close all
clear all
clc

%% INITIALIZE
main_loaddata

%% PROCESS DATA
WVSim  = [ WIniFinV(2,1) + (WIniFinV(1,1)-WIniFinV(2,1)) * exp(-TV/KTauV(2,1)) ; WIniFinV(2,2) + (WIniFinV(1,2)-WIniFinV(2,2)) * exp(-TV/KTauV(2,2)) ]';
WVnSim = [ WIniFinVn(2,1) + (WIniFinVn(1,1)-WIniFinVn(2,1)) * exp(-TVn/KTauVn(2,1)) ; WIniFinVn(2,2) + (WIniFinVn(1,2)-WIniFinVn(2,2)) * exp(-TVn/KTauVn(2,2)) ; WIniFinVn(2,3) + (WIniFinVn(1,3)-WIniFinVn(2,3)) * exp(-TVn/KTauVn(2,3)) ]';
WWSim  = [ WIniFinW(2,1) + (WIniFinW(1,1)-WIniFinW(2,1)) * exp(-TW/KTauW(2,1)) ; WIniFinW(2,2) + (WIniFinW(1,2)-WIniFinW(2,2)) * exp(-TW/KTauW(2,2)) ; WIniFinW(2,3) + (WIniFinW(1,3)-WIniFinW(2,3)) * exp(-TW/KTauW(2,3)) ]';

%% ERROR MEASURES
Error.VAbs  = abs(WV  - WVSim );
Error.VnAbs = abs(WVn - WVnSim);
Error.WAbs  = abs(WW  - WWSim );
Error.VAvg  = mean(Error.VAbs );
Error.VnAvg = mean(Error.VnAbs);
Error.WAvg  = mean(Error.WAbs );
Error.VAvgNorm  = mean(Error.VAvg  ./ abs(WIniFinV( 2,:)));
Error.VnAvgNorm = mean(Error.VnAvg ./ abs(WIniFinVn(2,:)));
Error.WAvgNorm  = mean(Error.WAvg  ./ abs(WIniFinW( 2,:)));
Error

%% VISUALIZE
figure
colors = get(gca,'colororder');
hold on
plot(TV,WV(:,1)   , '--'  , 'Color' , colors(1,:))
plot(TV,WV(:,2)   , '--'  , 'Color' , colors(2,:))
plot(TV,WVSim(:,1), '-'   , 'Color' , colors(1,:))
plot(TV,WVSim(:,2), '-'   , 'Color' , colors(2,:))
grid on
xlim([0,TV(end)])
xlabel('t (s) \rightarrow')
ylabel('dφ_i/dt (rad.s^{-1}) \rightarrow')
legend('{dφ_1/dt}_{filtered}','{dφ_2/dt}_{filtered}','{dφ_1/dt}_{1st-order}','{dφ_2/dt}_{1st-order}','Location', 'east')
title('Example of the angular speed response in the v motion')
pos = get(gcf, 'Position');
pos(3:4) = [500 500];
set(gcf, 'Position', pos)
% saveas(gcf,'motors-model_v','fig')
% saveas(gcf,'motors-model_v','epsc')

figure
colors = get(gca,'colororder');
hold on
plot(TVn,WVn(:,1)   , '--'  , 'Color' , colors(1,:))
plot(TVn,WVn(:,2)   , '--'  , 'Color' , colors(2,:))
plot(TVn,WVn(:,3)   , '--'  , 'Color' , colors(3,:))
plot(TVn,WVnSim(:,1), '-'   , 'Color' , colors(1,:))
plot(TVn,WVnSim(:,2), '-'   , 'Color' , colors(2,:))
plot(TVn,WVnSim(:,3), '-'   , 'Color' , colors(3,:))
grid on
xlim([0,TVn(end)])
xlabel('t (s) \rightarrow')
ylabel('dφ_i/dt (rad.s^{-1}) \rightarrow')
legend('{dφ_1/dt}_{filtered}','{dφ_2/dt}_{filtered}','{dφ_3/dt}_{filtered}','{dφ_1/dt}_{1st-order}','{dφ_2/dt}_{1st-order}','{dφ_3/dt}_{1st-order}','Location', 'east')
title('Example of the angular speed response in the v_n motion')
pos = get(gcf, 'Position');
pos(3:4) = [500 500];
set(gcf, 'Position', pos)
% saveas(gcf,'motors-model_vn','fig')
% saveas(gcf,'motors-model_vn','epsc')

figure
colors = get(gca,'colororder');
hold on
plot(TW,WW(:,1)   , '--'  , 'Color' , colors(1,:))
plot(TW,WW(:,2)   , '--'  , 'Color' , colors(2,:))
plot(TW,WW(:,3)   , '--'  , 'Color' , colors(3,:))
plot(TW,WWSim(:,1), '-'   , 'Color' , colors(1,:))
plot(TW,WWSim(:,2), '-'   , 'Color' , colors(2,:))
plot(TW,WWSim(:,3), '-'   , 'Color' , colors(3,:))
grid on
xlim([0,TW(end)])
xlabel('t (s) \rightarrow')
ylabel('dφ_i/dt (rad.s^{-1}) \rightarrow')
legend('{dφ_1/dt}_{filtered}','{dφ_2/dt}_{filtered}','{dφ_3/dt}_{filtered}','{dφ_1/dt}_{1st-order}','{dφ_2/dt}_{1st-order}','{dφ_3/dt}_{1st-order}')
title('Example of the angular speed response in the \omega motion')
pos = get(gcf, 'Position');
pos(3:4) = [500 500];
set(gcf, 'Position', pos)
% saveas(gcf,'motors-model_w','fig')
% saveas(gcf,'motors-model_w','epsc')