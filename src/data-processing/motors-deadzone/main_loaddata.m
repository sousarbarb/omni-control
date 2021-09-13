close all
clear all
clc

%% INITIALIZATION

% Log file
filename = '../../../data/motors-deadzone/deadzone_floor-carpet_v_1.txt';

% Experiment parameters
Tlog = 0.010;
pwm_max = 1023;
enc_res = 1024;
mot_ngear = 12;


%% LOAD LOG DATA
[t, Encoders, PWM, MotWr, RobVr, Vbatt] = LoadLogData(filename);


%% PROCESS LOG DATA

% Time
t = t - t(1);

% Voltage applied to the motors
MotVolt = (PWM .* Vbatt) ./ pwm_max;

% Angular velocity of the wheels
WhW = 2 * pi * Encoders / (0.010 * mot_ngear * enc_res);


%% VISUALIZE DEADZONE OF THE MOTORS
main_visualizedeadzone
