clear
close all
clc

%% INITIALIZATION

% Filename to save the trajectory file (format: CSV - "x m,y m,th rad")
% filename = '../SimTwo64/MSL/trajectories/square_vn-1.0.txt';
% filename = '../SimTwo64/MSL/trajectories/square-th_vn-1.0.txt';
% filename = '../SimTwo64/MSL/trajectories/s_vn-1.0_r-1.0.txt';
% filename = '../SimTwo64/MSL/trajectories/8_vn-1.0_r-1.0.txt';

% Parameters
parameters.Tctrl = 0.04;        % period of the position controllers (s)
parameters.vn    = 1.00;        % nominal velocity along the trajectory (m/s)
parameters.v_min = 0.10;        % minimum velocity along the trajectory (m/s)
parameters.acc_up   = 1.0;      % maximum acceleration when starting (m/s^2)
parameters.acc_down = 1.0;      % maximum deceleration when approaching (m/s^2)
parameters.tol_xy_ctrl = 0.10;  % distance tolerance of the controller (m)

% Trajectory
% trajectory = [
%   0 0 0;
%   2 0 0;
%   2 2 0;
%   0 2 0;
%   0 0 0;
% ];

trajectory = [
  0 0 0;
  2 0 -30;
  2 2 0;
  0 2 30;
  0 0 0;
];

% R = 1.0;
% delta_theta = 0.04 / R; %deg2rad(1);
% theta = pi:-delta_theta:delta_theta;
% trajectory = [
%   R*cos(theta')+1*R ,  R*sin(theta') , 0*theta' ;
%   R*cos(theta')+3*R , -R*sin(theta') , 0*theta' ;
%   R*4               ,  0             , 0        ;
% %   R*cos(theta')+5*R ,  R*sin(theta') , 0*theta' ;
% %   R*6               ,  0             , 0        ;
% ];

% R = 1.0;
% delta_theta = 0.04 / R; %deg2rad(1);
% theta = pi:-delta_theta:delta_theta;
% trajectory = [
%   R*cos(theta')+1*R ,  R*sin(theta') , 0*theta' ;
%   R*cos(theta')+3*R , -R*sin(theta') , 0*theta' ;
% ];
% theta = 0:delta_theta:pi-delta_theta;
% trajectory = [
%   trajectory                                    ;
%   R*cos(theta')+3*R ,  R*sin(theta') , 0*theta' ;
%   R*cos(theta')+1*R , -R*sin(theta') , 0*theta' ;
%   0                 ,  0             , 0        ;
% ];

trajectory(:,3) = deg2rad(trajectory(:,3));



%% GENERATE TRAJECTORY
[trajectory_generated] = GenerateTrajectoryOmni3(trajectory, parameters);



%% SAVE FILE
if exist('filename') ~= 0
  if ~isempty(filename)
    writematrix(trajectory_generated,filename)
  end
end

%% VISUALIZATION

[numpoints,~] = size(trajectory_generated);
t = linspace(0, (numpoints-1) * parameters.Tctrl, numpoints);
% Position XY
figure
hold on
plot(trajectory_generated(:,1),trajectory_generated(:,2), '.')
plot(trajectory(:,1),trajectory(:,2), 'ro')
grid on
axis equal
xlabel('x (m) \rightarrow')
ylabel('y (m) \rightarrow')
legend('\{x,y\}','WP_i')
title('Trajectory Generation: XY')

% Position 1 axis
figure
subplot(3,1,1)
hold on
plot(t,trajectory_generated(:,1), '.')
grid on
xlabel('t (s) \rightarrow')
ylabel('X^W (m) \rightarrow')
title('Squared Trajectory: X')

subplot(3,1,2)
hold on
plot(t,trajectory_generated(:,2), '.')
grid on
xlabel('t (s) \rightarrow')
ylabel('Y^W (m) \rightarrow')
title('Squared Trajectory: Y')

subplot(3,1,3)
hold on
plot(t,rad2deg(wrapToPi(trajectory_generated(:,3))), '.')
grid on
xlabel('t (s) \rightarrow')
ylabel('\theta^W (??) \rightarrow')
title('Squared Trajectory: \theta')

% Velocity 1 axis
trajectory_generated_v = zeros(numpoints,2);
trajectory_generated_v(1,1) = ...
    Dist(trajectory_generated(1,:),trajectory_generated(2,:)) / parameters.Tctrl;
trajectory_generated_v(1,2) = ...
    wrapToPi(trajectory_generated(2,3) - trajectory_generated(1,3)) / parameters.Tctrl;
for i=2:numpoints-1
  trajectory_generated_v(i,1) = ...
      0.5 * ...
      ( Dist(trajectory_generated(i-1,:),trajectory_generated(i,:)) +   ...
        Dist(trajectory_generated(i+1,:),trajectory_generated(i,:)) ) / ...
      parameters.Tctrl;
  trajectory_generated_v(i,2) = ...
      0.5 * ...
      ( wrapToPi(trajectory_generated(i  ,3) - trajectory_generated(i-1,3)) +   ...
        wrapToPi(trajectory_generated(i+1,3) - trajectory_generated(i  ,3)) ) / ...
      parameters.Tctrl;
end
trajectory_generated_v(end,1) = ...
    Dist(trajectory_generated(end,:),trajectory_generated(end-1,:)) / parameters.Tctrl;
trajectory_generated_v(end,2) = ...
    wrapToPi(trajectory_generated(end,3) - trajectory_generated(end-1,3)) / parameters.Tctrl;

figure
subplot(3,1,2)
hold on
plot(t,trajectory_generated_v(:,1), '.')
grid on
xlabel('t (s) \rightarrow')
ylabel('v (m.s^{-1}) \rightarrow')
title('Squared Trajectory: v')

subplot(3,1,3)
hold on
plot(t,trajectory_generated_v(:,2), '.')
grid on
xlabel('t (s) \rightarrow')
ylabel('\omega (rad.s^{-1}) \rightarrow')
title('Squared Trajectory: \omega')