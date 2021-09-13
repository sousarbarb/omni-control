main

close all

%% GLOBAL FILTERING
i=5
filename{i}

%% LOCAL FILTERING
j1 = 462;
j2 = 23732;
maxDiffXrefRel = [0.2 0.2 0.2];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 21;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


% Plot
plotData(TimeOdo{i},VRob{i},OldVRob{i},VrRob{i});


% Save data
w_2.TimeOdo = TimeOdo{i};
w_2.VRob    = VRob{i};
w_2.OldVRob = OldVRob{i};
w_2.VrRob   = VrRob{i};
writematrix(                           ...
  [ w_2.TimeOdo w_2.VRob w_2.VrRob ] , ...
  "dataout/w_2_filtered.csv"        ...
);
save dataout/w_2_filtered.mat w_2