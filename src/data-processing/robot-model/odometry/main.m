clear
close all
clc

%% INITIALISATION
options.iTime  = 0;       % Indexes for timestamps
options.iTicks = [1 3];   % Indexes for ticks count (wheels encoders)
options.iWr    = [7 9];   % Indexes for reference angular speed of the wheels
options.iVr    = [10 12]; % Indexes for reference robot velocity
folderOut = "dataout";    % Folder destination for the processed data
% Files to be processed
filename = {         ...
  "datain/v_1.csv" , ...
  "datain/v_2.csv" , ...
  "datain/vn.csv"  , ...
  "datain/w_1.csv" , ...
  "datain/w_2.csv" , ...
};

%% EXECUTION

% Initialisation
numFiles = length(filename);
OdoProc  = cell(1,numFiles);
TimeOdo  = cell(1,numFiles);
WRob     = cell(1,numFiles);
VRob     = cell(1,numFiles);
WrRob    = cell(1,numFiles);
VrRob    = cell(1,numFiles);
% Processed the selected files containing the odometric data
for i=1:numFiles
  [OdoProc{i},TimeOdo{i},error,WRob{i},VRob{i},WrRob{i},VrRob{i}] = processOdoDataRaw(filename{i},folderOut,options);
end

save dataout/data_odo_raw.mat