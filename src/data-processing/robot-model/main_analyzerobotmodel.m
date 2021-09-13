close all
clear all
clc

%% INITIALIZATION

% V
V = [
  1.019649723	0.127148556
  0.991749781	0.126386737
  0.99742388	0.034088429
  1.023573645	0.102892786
  1.036446848	0.134906133
  1.006368728	0.105686235
  1.018903015	0.137857672
  1.035146103	0.121650518
  1.057659144	0.096520772
  1.008149127	0.102614731
  1.009556983	0.116285016
  1.062505037	0.121202798
  1.032899369	0.164626513 ];

% Vn
Vn = [
  1.036479434	0.103168304
  1.018940856	0.055893969
  1.040217285	0.114039192
  1.015438813	0.129470328
  1.026356471	0.100878869
  1.043532643	0.144758944
  1.013730563	0.108135701
  1.036597461	0.084243616
  0.98211214	0.140557519
  1.007401932	0.092888782
  1.038576161	0.134486052
  1.042297976	0.044684361 ];

% W
W = [
  0.964083157	0.074280552
  0.984440415	0.102179064
  0.983568619	0.081049461
  0.963232957	0.090112515
  0.983482677	0.098982899
  0.971540239	0.090970423
  0.969569631	0.051717735
  0.978403478	0.100355525
  0.983632448	0.109618467
  0.979066316	0.080558834
  1.002263092	0.090682555
  0.976214796	0.06283469
  0.970307322	0.041057221
  1.00548873	0.099883973
  0.970048384	0.0915308
  0.976853202	0.101671679
  0.999355119	0.087035161
  0.974884423	0.087982458
  0.982470823	0.102728276
  0.989829881	0.100913941 ];


%% ANALYZE DATA (numerically)

% V
VMean   = mean(V)
VStdev  = std(V)
VMedian = median(V)

% Vn
VnMean   = mean(Vn)
VnStdev  = std(Vn)
VnMedian = median(Vn)

% W
WMean   = mean(W)
WStdev  = std(W)
WMedian = median(W)


%% ANALYZE DATA (visually)

% Box plot
BoxPlotLegend = [
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"V "
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"Vn"
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
"W "
];

figure
hold on
boxplot([ V(:,1) ; Vn(:,1) ; W(:,1) ],BoxPlotLegend)
grid on
ylim([0.9 1.1])
xlabel('Conditions of the experiments')
ylabel('K_p \rightarrow')
title('Boxplot to analyze the distribution of values and outliers - K_p')

figure
hold on
boxplot([ V(:,2) ; Vn(:,2) ; W(:,2) ],BoxPlotLegend)
grid on
ylim([0 0.2])
xlabel('Conditions of the experiments')
ylabel('\tau \rightarrow')
title('Boxplot to analyze the distribution of values and outliers - \tau')

% Histogram
figure

width = 0.01;
subplot(1,2,1)
hold on
histogram( V(:,1),'BinWidth',width)
histogram(Vn(:,1),'BinWidth',width)
histogram( W(:,1),'BinWidth',width)
grid on
xlabel('K_p \rightarrow')
legend('v','v_n','\omega')
title('K_p')

width = 0.01;
subplot(1,2,2)
hold on
histogram( V(:,2),'BinWidth',width)
histogram(Vn(:,2),'BinWidth',width)
histogram( W(:,2),'BinWidth',width)
grid on
xlabel('\tau \rightarrow')
legend('v','v_n','\omega')
title('\tau')