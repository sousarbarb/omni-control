main

close all

%% GLOBAL FILTERING
i=4
filename{i}

%% LOCAL FILTERING
j1 = 5584;
j2 = 9740;
% maxXvar = [2 0.05 0.5];
maxDiffXrefRel = [0.25 0.25 0.25];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 21;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


% Plot
plotData(TimeOdo{i},VRob{i},OldVRob{i},VrRob{i});


% Save data
w_1.TimeOdo = TimeOdo{i};
w_1.VRob    = VRob{i};
w_1.OldVRob = OldVRob{i};
w_1.VrRob   = VrRob{i};
writematrix(                           ...
  [ w_1.TimeOdo w_1.VRob w_1.VrRob ] , ...
  "dataout/w_1_filtered.csv"        ...
);
save dataout/w_1_filtered.mat w_1