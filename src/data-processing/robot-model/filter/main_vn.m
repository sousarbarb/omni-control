main
close all

%% GLOBAL FILTERING
i=3
filename{i}

%% LOCAL FILTERING
j1 = 581;
j2 = 1185;
% maxXvar = [2 0.05 0.5];
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 1619;
j2 = 2279;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 2692;
j2 = 3610;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 4314;
j2 = 5054;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 5564;
j2 = 6072;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 6519;
j2 = 7155;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 7577;
j2 = 8411;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 8778;
j2 = 9574;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 9905;
j2 = 10739;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 11158;
j2 = 11923;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 12282;
j2 = 13007;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);

j1 = 12760;
j2 = 12800;
maxDiffXrefRel = [0.25 0.25 0.25];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(VRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 21;
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);


j1 = 13438;
j2 = 14107;
maxDiffXrefRel = [0.5 0.5 0.5];
[VRob{i}(j1:j2,:)] = nonLinearFilterMaxDiffRef(OldVRob{i}(j1:j2,:),VrRob{i}(j1:j2,:),maxDiffXrefRel);
windowSize = 9;
[VRob{i}(j1:j2,:)] = nonCausalMedianFilter(VRob{i}(j1:j2,:),windowSize);
[VRob{i}(j1:j2,:)] = nonCausalAverageFilter(VRob{i}(j1:j2,:),windowSize);

% Plot
plotData(TimeOdo{i},VRob{i},OldVRob{i},VrRob{i});


% Save data
vn.TimeOdo = TimeOdo{i};
vn.VRob    = VRob{i};
vn.OldVRob = OldVRob{i};
vn.VrRob   = VrRob{i};
writematrix(                           ...
  [ vn.TimeOdo vn.VRob vn.VrRob ] , ...
  "dataout/vn_filtered.csv"        ...
);
save dataout/vn_filtered.mat vn