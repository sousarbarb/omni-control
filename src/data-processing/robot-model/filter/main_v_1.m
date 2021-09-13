main

close all

%% INITIALIZATION
i=1
filename{i}

%% LOCAL FILTERING
j1 = 6127;
j2 = 7058;
% maxXvar = [2 0.05 0.5];
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 10050;
j2 = 10771;
maxDiffXrefRel = [0.25 0.25 0.25];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 21;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);

j1 = 10530;
j2 = 10620;
maxDiffXrefRel = [0.25 0.25 0.25];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(VRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 35;
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);

% Plot
plotData(TimeOdo{i},VRob{i},OldVRob{i},VrRob{i});


% Save data
v_1.TimeOdo = TimeOdo{i};
v_1.VRob    = VRob{i};
v_1.OldVRob = OldVRob{i};
v_1.VrRob   = VrRob{i};
writematrix(                           ...
  [ v_1.TimeOdo v_1.VRob v_1.VrRob ] , ...
  "dataout/v_1_filtered.csv"        ...
);
save dataout/v_1_filtered.mat v_1