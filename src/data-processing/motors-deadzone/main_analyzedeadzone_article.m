close all
clear all
clc

%% INITIALIZATION

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

% Carpet
VdCarpetMean   = mean(VdCarpet)
VdCarpetStdev  = std(VdCarpet)
VdCarpetMedian = median(VdCarpet)


%% ANALYZE DATA (visually)

% Box plot
BoxPlotLegend = [
  "v"
  "vn"
  "w"
];

figure
hold on
boxplot(VdCarpet,BoxPlotLegend)
grid on
xlabel('Motion of the robot')
ylabel('V_{0,i} (V) \rightarrow')
title("Analysis of the dead zone of the motors")