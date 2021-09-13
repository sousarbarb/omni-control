function [t, Encoders, PWM, MotWr, RobVr, Vbatt] = LoadLogData(filename)
  
  % Initialization
  Data = readmatrix(filename);

  % Indexes
  it = 1;
  ienc = 2;
  ipwm = 5;
  imotwr = 8;
  irobvr = 11;
  ibatt = 14;

  % Outputs
  t = Data(:,it);
  Encoders = Data(:,ienc:ienc+2);
  PWM   = Data(:,ipwm:ipwm+2);
  MotWr = Data(:,imotwr:imotwr+2);
  RobVr = Data(:,irobvr:irobvr+2);
  Vbatt = Data(:,ibatt);

end