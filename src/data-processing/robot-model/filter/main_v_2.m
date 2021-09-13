main

close all

%% GLOBAL FILTERING
i=2
filename{i}

%% LOCAL FILTERING
j1 = 524;
j2 = 1190;
% maxXvar = [2 0.05 0.5];
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 1780;
j2 = 2432;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 2846;
j2 = 3570;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 4064;
j2 = 4759;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 5180;
j2 = 5904;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 6425;
j2 = 7166;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 7550;
j2 = 8544;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 9069;
j2 = 9983;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 10482;
j2 = 11540;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 11934;
j2 = 12948;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 13555;
j2 = 14367;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);

% Plot
plotData(TimeOdo{i},VRob{i},OldVRob{i},VrRob{i});


% Save data
v_2.TimeOdo = TimeOdo{i};
v_2.VRob    = VRob{i};
v_2.OldVRob = OldVRob{i};
v_2.VrRob   = VrRob{i};
writematrix(                           ...
  [ v_2.TimeOdo v_2.VRob v_2.VrRob ] , ...
  "dataout/v_2_filtered.csv"        ...
);
save dataout/v_2_filtered.mat v_2