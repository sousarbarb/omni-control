clear
close all
clc

%% INITIALISATION
load datain/data_odo_raw.mat
% Filter
T = 0.01;
maxVar = [7.5 7.5 50];
boolLastValidSample = false;
windowSize = 13;
% Save old data
OldVRob  = VRob;

%% DATA PROCESSMENT
for i=1:numFiles
  % Filter data
  [VRob{i}] = nonLinearFilterMaxVar(OldVRob{i},maxVar,T,boolLastValidSample);
  [VRob{i}] = nonCausalAverageFilter(VRob{i},windowSize);
end

%% SAVE DATA
%save dataout/data_filtered.mat