close all
clear all
clc


%% INITIALIZATION
% folder = "C:/Users/sousa/OneDrive - INESC TEC/inesctec/projects/trajectory-control/git/omni-control/";
folder = "../../../data/omni-control/5dpo/";
filenames = [
  "sq_0.5_3.csv" ,        ...1
  "sq_0.5_4.csv" ,        ...2
  "sq_0.5_5.csv" ,        ...3
  "sq_0.5_6.csv" ,        ...4
  "sq_0.5_7.csv" ,        ...5
  "sq_0.5_8.csv" ,        ...6
  "sq_0.5_9.csv" ,        ...7
  "sq_0.5_10.csv" ,       ...8
  "sq_0.5_11.csv" ,       ...9
  "sq_0.5_12.csv" ,       ...10
  "sq_0.5_13.csv" ,       ...11
  "sq_0.5_14.csv" ,       ...12
  "sq_0.5_15.csv" ,       ...13
  "sq_0.5_16.csv" ,       ...14
  "sq_0.5_17.csv" ,       ...15
  "sq_0.5_18.csv" ,       ...16
  "sq_0.75_3.csv" ,       ...17
  "sq_0.75_4.csv" ,       ...18
  "sq_0.75_5.csv" ,       ...19
  "sq_0.75_6.csv" ,       ...20
  "sq_0.75_7.csv" ,       ...21
  "sq_0.75_8.csv" ,       ...22
  "sq_0.75_9.csv" ,       ...23
  "sq_0.75_10.csv" ,      ...24
  "sq_0.75_11.csv" ,      ...25
  "sq_0.75_12.csv" ,      ...26
  "sq_0.75_13.csv" ,      ...27
  "sq_0.75_14.csv" ,      ...28
  "sq_0.75_15.csv" ,      ...29
  "sq_0.75_16.csv" ,      ...30
  "sq_0.75_17.csv" ,      ...31
  "sq_0.75_18.csv" ,      ...32
  "sq_0.75_19.csv" ,      ...33
  "sq_0.75_20.csv" ,      ...34
  "sq_0.75_21.csv" ,      ...35
  "sq_0.75_22.csv" ,      ...36
  "sq_1.0_3.csv" ,        ...37
  "sq_1.0_4.csv" ,        ...38
  "sq_1.0_5.csv" ,        ...39
  "sq_1.0_6.csv" ,        ...40
  "sq_1.0_7.csv" ,        ...41
  "sq_1.0_8.csv" ,        ...42
  "sq_1.0_9.csv" ,        ...43
  "sq_1.0_10.csv" ,       ...44
  "sq_1.0_11.csv" ,       ...45
  "sq_1.0_12.csv" ,       ...46
  "sq_1.0_13.csv" ,       ...47
  "sq_1.0_14.csv" ,       ...48
  "sq_1.0_15.csv" ,       ...49
  "sq_1.0_16.csv" ,       ...50
  "sq_1.0_17.csv" ,       ...51
  "sq_1.0_18.csv" ,       ...52
  "sq_1.0_19.csv" ,       ...53
  "sq_1.0_20.csv" ,       ...54
  "sq_1.0_21.csv" ,       ...55
  "sq_1.0_22.csv" ,       ...56
  "sq_1.0_23.csv" ,       ...57
  "sq_1.0_24.csv" ,       ...58
  "sq-th_0.75_5.csv" ,    ...59
  "sq-th_0.75_6.csv" ,    ...60
  "sq-th_0.75_7.csv" ,    ...61
  "sq-th_0.75_8.csv" ,    ...62
  "sq-th_0.75_9.csv" ,    ...63
  "sq-th_0.75_10.csv" ,   ...64
  "sq-th_0.75_11.csv" ,   ...65
  "sq-th_0.75_12.csv" ,   ...66
  "sq-th_0.75_13.csv" ,   ...67
  "sq-th_0.75_14.csv" ,   ...68
  "sq-th_0.75_15.csv" ,   ...69
  "sq-th_0.75_16.csv" ,   ...70
  "sq-th_0.75_17.csv" ,   ...71
  "sq-th_0.75_18.csv" ,   ...72
  "sq-th_0.75_19.csv" ,   ...73
  "sq-th_0.75_20.csv" ,   ...74
  "sq-th_0.75_21.csv" ,   ...75
  "sq-th_0.75_22.csv" ,   ...76
  "8_r-1.0_0.75_5.csv" ,  ...77
  "8_r-1.0_0.75_6.csv" ,  ...78
  "8_r-1.0_0.75_7.csv" ,  ...79
  "8_r-1.0_0.75_8.csv" ,  ...80
  "8_r-1.0_0.75_9.csv" ,  ...81
  "8_r-1.0_0.75_10.csv" , ...82
  "8_r-1.0_0.75_11.csv" , ...83
  "8_r-1.0_0.75_12.csv" , ...84
  "8_r-1.0_0.75_13.csv" , ...85
  "8_r-1.0_0.75_14.csv" , ...86
  "8_r-1.0_0.75_15.csv" , ...87
  "8_r-1.0_0.75_16.csv" , ...88
  "8_r-1.0_0.75_17.csv" , ...89
  "8_r-1.0_0.75_18.csv" , ...90
  "8_r-1.0_0.75_19.csv" , ...91
  "8_r-1.0_0.75_20.csv" , ...92
  "8_r-1.0_0.75_21.csv" , ...93
  "8_r-1.0_0.75_22.csv" , ...94
  "8_r-1.0_1.0_8.csv" ,   ...95
  "8_r-1.0_1.0_9.csv" ,   ...96
  "8_r-1.0_1.0_10.csv" ,  ...97
  "8_r-1.0_1.0_11.csv" ,  ...98
  "8_r-1.0_1.0_12.csv" ,  ...99
  "8_r-1.0_1.0_13.csv" ,  ...100
  "8_r-1.0_1.0_14.csv" ,  ...101
  "8_r-1.0_1.0_15.csv" ,  ...102
  "8_r-1.0_1.0_16.csv" ,  ...103
  "8_r-1.0_1.0_17.csv" ,  ...104
  "8_r-1.0_1.0_18.csv" ,  ...105
  "8_r-1.0_1.0_19.csv" ,  ...106
  "8_r-1.0_1.0_20.csv" ,  ...107
  "8_r-1.0_1.0_21.csv" ,  ...108
  "8_r-1.0_1.0_22.csv" ,  ...109
];
num_files = length(filenames);


%% READ DATA

% Options for importing the CSV files
readdata_opts = detectImportOptions( folder + filenames(1) );
% (change the following line if needed)
readdata_opts.DataLines = 13;

% Data cell
Data = cell(1,num_files);

for i=1:num_files
  Data{i} = readmatrix( folder + filenames(i) , readdata_opts);
end


%% PROCESS DATA

% Indexes (1..n) (change these indexes if needed)
it = 1;
iv   = 3;
iv_r = 6;
ip   = 33;
ip_r = 36;

% Data variables cells
Time = cell(1,num_files);
Xpos = cell(1,num_files);
Xpos_r = cell(1,num_files);
Xvel = cell(1,num_files);
Xvel_r = cell(1,num_files);

for i=1:num_files
  Time{i} = Data{i}(:, it );
  Time{i} = Time{i} - Time{i}(1);
  Xpos{i}   = Data{i}(:, ip  :ip  +2 );
  Xpos_r{i} = Data{i}(:, ip_r:ip_r+2 );
  Xvel{i}   = Data{i}(:, iv  :iv  +2 );
  Xvel_r{i} = Data{i}(:, iv_r:iv_r+2 );
end


%% COMPUTE ERROR MEASUREMENTS

% Data variables error cells
Error = cell(1,num_files);

for i=1:num_files
  Error{i} = TrajectoryQualityMeasurements(Time{i}, Xpos{i}, Xpos_r{i}, []);
end


%% OUTPUT ERROR MEASUREMENTS

% Filename output (change filename for saving data analysis)
filename_out = "data-analysis_raw_entire-traj.csv";
Data_out = cell(num_files+2,11);
Data_out(1,:) = { "", "Average", "", "", "", "", "Maximum", "", "", "", "" };
Data_out(2,:) = { "", "|e_x| (m)", "|e_y| (m)", "|e_th| (deg)", "|e_dist| (m)", "|e_traj| (m)", "|e_x| (m)", "|e_y| (m)", "|e_th| (deg)", "|e_dist| (m)", "|e_traj| (m)" };

for i=1:num_files
  Data_out(i+2,:) = {
    filenames(i) , ...
    Error{i}.avg_abs_x  , ...
    Error{i}.avg_abs_y  , ...
    Error{i}.avg_abs_th , ...
    Error{i}.avg_abs_dist , ...
    Error{i}.avg_trajectory , ...
    Error{i}.max_abs_x  , ...
    Error{i}.max_abs_y  , ...
    Error{i}.max_abs_th , ...
    Error{i}.max_abs_dist , ...
    Error{i}.max_trajectory
  };
end

% Write file
writecell( Data_out , folder + filename_out );


%% VISUALIZE DATA

% Index to show data (change these indexes if needed)
ivisualize = [
  
];

if (~isempty(ivisualize))
  
  for i=1:length(ivisualize)

    figure
    hold on
    plot(   Xpos{ivisualize(i)}(:,1) ,   Xpos{ivisualize(i)}(:,2) )
    plot( Xpos_r{ivisualize(i)}(:,1) , Xpos_r{ivisualize(i)}(:,2) )
    grid on
    axis equal
    xlabel('x (m) \rightarrow')
    ylabel('y (m) \rightarrow')
    legend('(x,y)','(x,y)_r')
    title_plot = title(sprintf("Trajectory (%s): X , Y", filenames(ivisualize(i))));
    set(title_plot, 'Interpreter', 'none')


    figure
    colors = get(gca,'colororder');

    subplot(1,2,1)
    hold on
    plot( Time{ivisualize(i)} ,   Xpos{ivisualize(i)}(:,1) , '-'  , 'Color' , colors(1,:) )
    plot( Time{ivisualize(i)} , Xpos_r{ivisualize(i)}(:,1) , '--' , 'Color' , colors(1,:) )
    plot( Time{ivisualize(i)} ,   Xpos{ivisualize(i)}(:,2) , '-'  , 'Color' , colors(2,:) )
    plot( Time{ivisualize(i)} , Xpos_r{ivisualize(i)}(:,2) , '--' , 'Color' , colors(2,:) )
    grid on
    xlabel('time (s) \rightarrow')
    ylabel('x | y (m) \rightarrow')
    legend('x','x_r','y','y_r')
    title_plot = title(sprintf("Trajectory (%s): X | Y (time)", filenames(ivisualize(i))));
    set(title_plot, 'Interpreter', 'none')

    subplot(1,2,2)
    hold on
    plot( Time{ivisualize(i)} , rad2deg(  Xpos{ivisualize(i)}(:,3)) , '-'  , 'Color' , colors(1,:) )
    plot( Time{ivisualize(i)} , rad2deg(Xpos_r{ivisualize(i)}(:,3)) , '--' , 'Color' , colors(1,:) )
    grid on
    xlabel('time (s) \rightarrow')
    ylabel('\theta (m) \rightarrow')
    legend('\theta','x_r')
    title_plot = title(sprintf("Trajectory (%s): \theta (time)", filenames(ivisualize(i))));
    set(title_plot, 'Interpreter', 'none')

  end
  
end
