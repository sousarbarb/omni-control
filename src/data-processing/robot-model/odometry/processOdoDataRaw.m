function [OdoProc,TimeOdo,error,WRob,VRob,WrRob,VrRob] = processOdoDataRaw(filename,folderOut,options)
%PROCESSODODATARAW

  %% INITIALISATION
  error = 0;
  % Robot kinematic parameters
  R = 0.049283;
  L = 0.191367;
  GearEncRes = 1024*12;
  Fs = 100;
  
  %% DATA PROCESSMENT
  
  % Adjust indexes
  options.iTime  = options.iTime  + 1;
  options.iTicks = options.iTicks + 1;
  options.iWr = options.iWr + 1;
  options.iVr = options.iVr + 1;
  % Read csv file (expected to only contain the numeric data, unlike the
  % data from optitrack)
  M = csvread(filename);  % Data matrix retrieved from the csv file
  
  %% OUTPUT
  
  % Time array
  TimeOdo = M(:,options.iTime) - M(1,options.iTime);
  DataToWrite = TimeOdo;
  % Odometry data
  OdoProc = M(:,options.iTicks(1):options.iTicks(end));
  DataToWrite = [DataToWrite OdoProc];
  % Wheels angular speed
  WRob = 2 * pi * Fs * OdoProc / GearEncRes;
  % Robot velocity
  VRob = zeros(size(M(:,options.iVr(1):options.iVr(end))));
  VRob(:,1) = -sqrt(3)/3 * WRob(:,1) + sqrt(3)/3 * WRob(:,2);
  VRob(:,2) =       -1/3 * WRob(:,1) -       1/3 * WRob(:,2) + 2/3 * WRob(:,3);
  VRob(:,3) =       -1/3 * WRob(:,1) -       1/3 * WRob(:,2) - 1/3 * WRob(:,3);
  VRob(:,3) = VRob(:,3) / L;
  VRob = -VRob * R;
  % Robot reference velocity
  VrRob = M(:,options.iVr(1):options.iVr(end));
  % Wheels reference angular speed
  WrRob = zeros(size(M(:,options.iWr(1):options.iWr(end))));
  WrRob(:,1) = -sqrt(3)/2 * VrRob(:,1) - 0.5 * VrRob(:,2) - L * VrRob(:,3);
  WrRob(:,2) =  sqrt(3)/2 * VrRob(:,1) - 0.5 * VrRob(:,2) - L * VrRob(:,3);
  WrRob(:,3) =                                 VrRob(:,2) - L * VrRob(:,3);
  WrRob = -WrRob / R;
  % Add data to save csv
  DataToWrite = [DataToWrite WRob VRob WrRob VrRob];
  % File with the processed data
  [~,filenameWithoutExt,~] = fileparts(filename);
  writematrix(                                                              ...
    DataToWrite                                                         ,   ...
    strcat(strcat(folderOut,"/"),strcat(filenameWithoutExt,"_odo_raw.csv")) ...
  );
end








