close all
clear all
clc


%% INITIALIZATION
% folder = "../../../src/SimTwo64/MSL/log/";
folder = "../../../data/omni-control/simtwo/odo/";
filenames = [
  "sq_0.5_3_odo.csv" ,        ...1
  "sq_0.5_4_odo.csv" ,        ...2
  "sq_0.5_5_odo.csv" ,        ...3
  "sq_0.5_6_odo.csv" ,        ...4
  "sq_0.5_7_odo.csv" ,        ...5
  "sq_0.5_8_odo.csv" ,        ...6
  "sq_0.5_9_odo.csv" ,        ...7
  "sq_0.5_10_odo.csv" ,       ...8
  "sq_0.5_11_odo.csv" ,       ...9
  "sq_0.5_12_odo.csv" ,       ...10
  "sq_0.5_13_odo.csv" ,       ...11
  "sq_0.5_14_odo.csv" ,       ...12
  "sq_0.5_15_odo.csv" ,       ...13
  "sq_0.5_16_odo.csv" ,       ...14
  "sq_0.5_17_odo.csv" ,       ...15
  "sq_0.5_18_odo.csv" ,       ...16
  "sq_0.75_3_odo.csv" ,       ...17
  "sq_0.75_4_odo.csv" ,       ...18
  "sq_0.75_5_odo.csv" ,       ...19
  "sq_0.75_6_odo.csv" ,       ...20
  "sq_0.75_7_odo.csv" ,       ...21
  "sq_0.75_8_odo.csv" ,       ...22
  "sq_0.75_9_odo.csv" ,       ...23
  "sq_0.75_10_odo.csv" ,      ...24
  "sq_0.75_11_odo.csv" ,      ...25
  "sq_0.75_12_odo.csv" ,      ...26
  "sq_0.75_13_odo.csv" ,      ...27
  "sq_0.75_14_odo.csv" ,      ...28
  "sq_0.75_15_odo.csv" ,      ...29
  "sq_0.75_16_odo.csv" ,      ...30
  "sq_0.75_17_odo.csv" ,      ...31
  "sq_0.75_18_odo.csv" ,      ...32
  "sq_0.75_19_odo.csv" ,      ...33
  "sq_0.75_20_odo.csv" ,      ...34
  "sq_0.75_21_odo.csv" ,      ...35
  "sq_0.75_22_odo.csv" ,      ...36
  "sq_1.0_3_odo.csv" ,        ...37
  "sq_1.0_4_odo.csv" ,        ...38
  "sq_1.0_5_odo.csv" ,        ...39
  "sq_1.0_6_odo.csv" ,        ...40
  "sq_1.0_7_odo.csv" ,        ...41
  "sq_1.0_8_odo.csv" ,        ...42
  "sq_1.0_9_odo.csv" ,        ...43
  "sq_1.0_10_odo.csv" ,       ...44
  "sq_1.0_11_odo.csv" ,       ...45
  "sq_1.0_12_odo.csv" ,       ...46
  "sq_1.0_13_odo.csv" ,       ...47
  "sq_1.0_14_odo.csv" ,       ...48
  "sq_1.0_15_odo.csv" ,       ...49
  "sq_1.0_16_odo.csv" ,       ...50
  "sq_1.0_17_odo.csv" ,       ...51
  "sq_1.0_18_odo.csv" ,       ...52
  "sq_1.0_19_odo.csv" ,       ...53
  "sq_1.0_20_odo.csv" ,       ...54
  "sq_1.0_21_odo.csv" ,       ...55
  "sq_1.0_22_odo.csv" ,       ...56
  "sq_1.0_23_odo.csv" ,       ...57
  "sq_1.0_24_odo.csv" ,       ...58
  "sq-th_0.75_5_odo.csv" ,    ...59
  "sq-th_0.75_6_odo.csv" ,    ...60
  "sq-th_0.75_7_odo.csv" ,    ...61
  "sq-th_0.75_8_odo.csv" ,    ...62
  "sq-th_0.75_9_odo.csv" ,    ...63
  "sq-th_0.75_10_odo.csv" ,   ...64
  "sq-th_0.75_11_odo.csv" ,   ...65
  "sq-th_0.75_12_odo.csv" ,   ...66
  "sq-th_0.75_13_odo.csv" ,   ...67
  "sq-th_0.75_14_odo.csv" ,   ...68
  "sq-th_0.75_15_odo.csv" ,   ...69
  "sq-th_0.75_16_odo.csv" ,   ...70
  "sq-th_0.75_17_odo.csv" ,   ...71
  "sq-th_0.75_18_odo.csv" ,   ...72
  "sq-th_0.75_19_odo.csv" ,   ...73
  "sq-th_0.75_20_odo.csv" ,   ...74
  "sq-th_0.75_21_odo.csv" ,   ...75
  "sq-th_0.75_22_odo.csv" ,   ...76
  "8_r-1.0_0.75_5_odo.csv" ,  ...77
  "8_r-1.0_0.75_6_odo.csv" ,  ...78
  "8_r-1.0_0.75_7_odo.csv" ,  ...79
  "8_r-1.0_0.75_8_odo.csv" ,  ...80
  "8_r-1.0_0.75_9_odo.csv" ,  ...81
  "8_r-1.0_0.75_10_odo.csv" , ...82
  "8_r-1.0_0.75_11_odo.csv" , ...83
  "8_r-1.0_0.75_12_odo.csv" , ...84
  "8_r-1.0_0.75_13_odo.csv" , ...85
  "8_r-1.0_0.75_14_odo.csv" , ...86
  "8_r-1.0_0.75_15_odo.csv" , ...87
  "8_r-1.0_0.75_16_odo.csv" , ...88
  "8_r-1.0_0.75_17_odo.csv" , ...89
  "8_r-1.0_0.75_18_odo.csv" , ...90
  "8_r-1.0_0.75_19_odo.csv" , ...91
  "8_r-1.0_0.75_20_odo.csv" , ...92
  "8_r-1.0_0.75_21_odo.csv" , ...93
  "8_r-1.0_0.75_22_odo.csv" , ...94
  "8_r-1.0_1.0_8_odo.csv" ,   ...95
  "8_r-1.0_1.0_9_odo.csv" ,   ...96
  "8_r-1.0_1.0_10_odo.csv" ,  ...97
  "8_r-1.0_1.0_11_odo.csv" ,  ...98
  "8_r-1.0_1.0_12_odo.csv" ,  ...99
  "8_r-1.0_1.0_13_odo.csv" ,  ...100
  "8_r-1.0_1.0_14_odo.csv" ,  ...101
  "8_r-1.0_1.0_15_odo.csv" ,  ...102
  "8_r-1.0_1.0_16_odo.csv" ,  ...103
  "8_r-1.0_1.0_17_odo.csv" ,  ...104
  "8_r-1.0_1.0_18_odo.csv" ,  ...105
  "8_r-1.0_1.0_19_odo.csv" ,  ...106
  "8_r-1.0_1.0_20_odo.csv" ,  ...107
  "8_r-1.0_1.0_21_odo.csv" ,  ...108
  "8_r-1.0_1.0_22_odo.csv" ,  ...109
  "sq_0.5_19_odo.csv" ,       ...110
  "sq_0.5_20_odo.csv" ,       ...111
  "sq_0.5_21_odo.csv" ,       ...112
  "sq_0.5_22_odo.csv" ,       ...113
  "sq-th_0.5_3_odo.csv" ,     ...114
  "sq-th_0.5_4_odo.csv" ,     ...115
  "sq-th_0.5_5_odo.csv" ,     ...116
  "sq-th_0.5_6_odo.csv" ,     ...117
  "sq-th_0.5_7_odo.csv" ,     ...118
  "sq-th_0.5_8_odo.csv" ,     ...119
  "sq-th_0.5_9_odo.csv" ,     ...120
  "sq-th_0.5_10_odo.csv" ,    ...121
  "sq-th_0.5_11_odo.csv" ,    ...122
  "sq-th_0.5_12_odo.csv" ,    ...123
  "sq-th_0.5_13_odo.csv" ,    ...124
  "sq-th_0.5_14_odo.csv" ,    ...125
  "sq-th_0.5_15_odo.csv" ,    ...126
  "sq-th_0.5_16_odo.csv" ,    ...127
  "sq-th_0.5_17_odo.csv" ,    ...128
  "sq-th_0.5_18_odo.csv" ,    ...129
  "sq-th_0.5_19_odo.csv" ,    ...130
  "sq-th_0.5_20_odo.csv" ,    ...131
  "sq-th_0.5_21_odo.csv" ,    ...132
  "sq-th_0.5_22_odo.csv" ,    ...133
  "sq-th_0.75_3_odo.csv" ,    ...134
  "sq-th_0.75_4_odo.csv" ,    ...135
  "sq-th_1.0_3_odo.csv" ,     ...136
  "sq-th_1.0_4_odo.csv" ,     ...137
  "sq-th_1.0_5_odo.csv" ,     ...138
  "sq-th_1.0_6_odo.csv" ,     ...139
  "sq-th_1.0_7_odo.csv" ,     ...140
  "sq-th_1.0_8_odo.csv" ,     ...141
  "sq-th_1.0_9_odo.csv" ,     ...142
  "sq-th_1.0_10_odo.csv" ,    ...143
  "sq-th_1.0_11_odo.csv" ,    ...144
  "sq-th_1.0_12_odo.csv" ,    ...145
  "sq-th_1.0_13_odo.csv" ,    ...146
  "sq-th_1.0_14_odo.csv" ,    ...147
  "sq-th_1.0_15_odo.csv" ,    ...148
  "sq-th_1.0_16_odo.csv" ,    ...149
  "sq-th_1.0_17_odo.csv" ,    ...150
  "sq-th_1.0_18_odo.csv" ,    ...151
  "sq-th_1.0_19_odo.csv" ,    ...152
  "sq-th_1.0_20_odo.csv" ,    ...153
  "sq-th_1.0_21_odo.csv" ,    ...154
  "sq-th_1.0_22_odo.csv" ,    ...155
  "sq-th_1.0_23_odo.csv" ,    ...156
  "sq-th_1.0_24_odo.csv" ,    ...157
  "8_r-1.0_0.5_3_odo.csv" ,   ...158
  "8_r-1.0_0.5_4_odo.csv" ,   ...159
  "8_r-1.0_0.5_5_odo.csv" ,   ...160
  "8_r-1.0_0.5_6_odo.csv" ,   ...161
  "8_r-1.0_0.5_7_odo.csv" ,   ...162
  "8_r-1.0_0.5_8_odo.csv" ,   ...163
  "8_r-1.0_0.5_9_odo.csv" ,   ...164
  "8_r-1.0_0.5_10_odo.csv" ,  ...165
  "8_r-1.0_0.5_11_odo.csv" ,  ...166
  "8_r-1.0_0.5_12_odo.csv" ,  ...167
  "8_r-1.0_0.5_13_odo.csv" ,  ...168
  "8_r-1.0_0.5_14_odo.csv" ,  ...169
  "8_r-1.0_0.5_15_odo.csv" ,  ...170
  "8_r-1.0_0.5_16_odo.csv" ,  ...171
  "8_r-1.0_0.5_17_odo.csv" ,  ...172
  "8_r-1.0_0.5_18_odo.csv" ,  ...173
  "8_r-1.0_0.5_19_odo.csv" ,  ...174
  "8_r-1.0_0.5_20_odo.csv" ,  ...175
  "8_r-1.0_0.5_21_odo.csv" ,  ...176
  "8_r-1.0_0.5_22_odo.csv" ,  ...177
  "8_r-1.0_0.75_3_odo.csv" ,  ...178
  "8_r-1.0_0.75_4_odo.csv" ,  ...179
  "8_r-1.0_1.0_3_odo.csv" ,   ...180
  "8_r-1.0_1.0_4_odo.csv" ,   ...181
  "8_r-1.0_1.0_5_odo.csv" ,   ...182
  "8_r-1.0_1.0_6_odo.csv" ,   ...183
  "8_r-1.0_1.0_7_odo.csv" ,   ...184
  "8_r-1.0_1.0_23_odo.csv" ,  ...185
  "8_r-1.0_1.0_24_odo.csv" ,  ...186
];
num_files = length(filenames);


%% READ DATA

% Options for importing the CSV files
readdata_opts = detectImportOptions( folder + filenames(1) );
% (change the following line if needed)
readdata_opts.DataLines = 15;

% Data cell
Data = cell(1,num_files);

for i=1:num_files
  Data{i} = readmatrix( folder + filenames(i) , readdata_opts);
end


%% PROCESS DATA

% Indexes (1..n) (change these indexes if needed)
it = 1;
iv   = 22;
iv_r = 25;
ip   = 58;
ip_r = 61;

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
filename_out = "data-analysis_raw_entire-traj_odo.csv";
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
  59
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
