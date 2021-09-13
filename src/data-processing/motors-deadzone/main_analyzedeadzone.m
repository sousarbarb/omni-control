close all
clear all
clc

%% INITIALIZATION

% Rubber floor
VdRubber = [
  3.623   3.0269	1.6938
  3.1982	3.0247	1.8312
  3.3302	3.2915	1.82801
  3.0495	3.288   1.40323
  3.4616	3.0086	1.5419
  3.1802	3.2786	1.5403
  3.4543	3.2727	1.5393
  3.3126	3.2751	1.5376
  3.1712	3.5416	1.6751
  3.3091	3.2657	1.6739	]

% Carpet
VdCarpet = [
  3.6581	3.5061	2.0418
  3.52    3.5061	1.6317
  3.3847	3.2364	1.767
  3.6528	3.234   1.6305
  3.6502	3.234   1.6299
  3.6515	3.5035	1.4936
  3.381   3.501   1.4925
  3.5124	3.501   1.6276
  2.7019	3.501   1.6276
  3.3773	3.2317	1.3563  ]


%% ANALYZE DATA (numerically)

% Rubber floor
VdRubberMean   = mean(VdRubber)
VdRubberStdev  = std(VdRubber)
VdRubberMedian = median(VdRubber)

% Carpet
VdCarpetMean   = mean(VdCarpet)
VdCarpetStdev  = std(VdCarpet)
VdCarpetMedian = median(VdCarpet)


%% ANALYZE DATA (visually)

% Box plot
BoxPlotLegend = [
  "Rubber - V"
  "Rubber - Vn"
  "Rubber - W"
  "Carpet - V"
  "Carpet - Vn"
  "Carpet - W"
];

figure
hold on
boxplot([VdRubber VdCarpet],BoxPlotLegend)
grid on
xlabel('Conditions of the experiments')
ylabel('Deadzone of the robot motors (V) \rightarrow')
title('Boxplot to analyze the distribution of values and outliers')

% Histogram
width = 0.1;

figure

subplot(1,2,1)
hold on
histogram(VdRubber(:,1),'BinWidth',width)
histogram(VdRubber(:,2),'BinWidth',width)
histogram(VdRubber(:,3),'BinWidth',width)
grid on
xlabel('Deadzone of the robot motors (V) \rightarrow')
legend('v','v_n','\omega')
title('Rubber')

subplot(1,2,2)
hold on
histogram(VdCarpet(:,1),'BinWidth',width)
histogram(VdCarpet(:,2),'BinWidth',width)
histogram(VdCarpet(:,3),'BinWidth',width)
grid on
xlabel('Deadzone of the robot motors (V) \rightarrow')
legend('v','v_n','\omega')
title('Carpet')