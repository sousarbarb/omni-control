close all
clear all
clc


%% INITIALIZATION
% folder = "../../../src/SimTwo64/MSL/log/";
folder = "../../../data/omni-control/simtwo/gt/";
filenames = [
  "sq_0.5_3_gt.csv" ,        ...1
  "sq_0.5_4_gt.csv" ,        ...2
  "sq_0.5_5_gt.csv" ,        ...3
  "sq_0.5_6_gt.csv" ,        ...4
  "sq_0.5_7_gt.csv" ,        ...5
  "sq_0.5_8_gt.csv" ,        ...6
  "sq_0.5_9_gt.csv" ,        ...7
  "sq_0.5_10_gt.csv" ,       ...8
  "sq_0.5_11_gt.csv" ,       ...9
  "sq_0.5_12_gt.csv" ,       ...10
  "sq_0.5_13_gt.csv" ,       ...11
  "sq_0.5_14_gt.csv" ,       ...12
  "sq_0.5_15_gt.csv" ,       ...13
  "sq_0.5_16_gt.csv" ,       ...14
  "sq_0.5_17_gt.csv" ,       ...15
  "sq_0.5_18_gt.csv" ,       ...16
  "sq_0.75_3_gt.csv" ,       ...17
  "sq_0.75_4_gt.csv" ,       ...18
  "sq_0.75_5_gt.csv" ,       ...19
  "sq_0.75_6_gt.csv" ,       ...20
  "sq_0.75_7_gt.csv" ,       ...21
  "sq_0.75_8_gt.csv" ,       ...22
  "sq_0.75_9_gt.csv" ,       ...23
  "sq_0.75_10_gt.csv" ,      ...24
  "sq_0.75_11_gt.csv" ,      ...25
  "sq_0.75_12_gt.csv" ,      ...26
  "sq_0.75_13_gt.csv" ,      ...27
  "sq_0.75_14_gt.csv" ,      ...28
  "sq_0.75_15_gt.csv" ,      ...29
  "sq_0.75_16_gt.csv" ,      ...30
  "sq_0.75_17_gt.csv" ,      ...31
  "sq_0.75_18_gt.csv" ,      ...32
  "sq_0.75_19_gt.csv" ,      ...33
  "sq_0.75_20_gt.csv" ,      ...34
  "sq_0.75_21_gt.csv" ,      ...35
  "sq_0.75_22_gt.csv" ,      ...36
  "sq_1.0_3_gt.csv" ,        ...37
  "sq_1.0_4_gt.csv" ,        ...38
  "sq_1.0_5_gt.csv" ,        ...39
  "sq_1.0_6_gt.csv" ,        ...40
  "sq_1.0_7_gt.csv" ,        ...41
  "sq_1.0_8_gt.csv" ,        ...42
  "sq_1.0_9_gt.csv" ,        ...43
  "sq_1.0_10_gt.csv" ,       ...44
  "sq_1.0_11_gt.csv" ,       ...45
  "sq_1.0_12_gt.csv" ,       ...46
  "sq_1.0_13_gt.csv" ,       ...47
  "sq_1.0_14_gt.csv" ,       ...48
  "sq_1.0_15_gt.csv" ,       ...49
  "sq_1.0_16_gt.csv" ,       ...50
  "sq_1.0_17_gt.csv" ,       ...51
  "sq_1.0_18_gt.csv" ,       ...52
  "sq_1.0_19_gt.csv" ,       ...53
  "sq_1.0_20_gt.csv" ,       ...54
  "sq_1.0_21_gt.csv" ,       ...55
  "sq_1.0_22_gt.csv" ,       ...56
  "sq_1.0_23_gt.csv" ,       ...57
  "sq_1.0_24_gt.csv" ,       ...58
  "sq-th_0.75_5_gt.csv" ,    ...59
  "sq-th_0.75_6_gt.csv" ,    ...60
  "sq-th_0.75_7_gt.csv" ,    ...61
  "sq-th_0.75_8_gt.csv" ,    ...62
  "sq-th_0.75_9_gt.csv" ,    ...63
  "sq-th_0.75_10_gt.csv" ,   ...64
  "sq-th_0.75_11_gt.csv" ,   ...65
  "sq-th_0.75_12_gt.csv" ,   ...66
  "sq-th_0.75_13_gt.csv" ,   ...67
  "sq-th_0.75_14_gt.csv" ,   ...68
  "sq-th_0.75_15_gt.csv" ,   ...69
  "sq-th_0.75_16_gt.csv" ,   ...70
  "sq-th_0.75_17_gt.csv" ,   ...71
  "sq-th_0.75_18_gt.csv" ,   ...72
  "sq-th_0.75_19_gt.csv" ,   ...73
  "sq-th_0.75_20_gt.csv" ,   ...74
  "sq-th_0.75_21_gt.csv" ,   ...75
  "sq-th_0.75_22_gt.csv" ,   ...76
  "8_r-1.0_0.75_5_gt.csv" ,  ...77
  "8_r-1.0_0.75_6_gt.csv" ,  ...78
  "8_r-1.0_0.75_7_gt.csv" ,  ...79
  "8_r-1.0_0.75_8_gt.csv" ,  ...80
  "8_r-1.0_0.75_9_gt.csv" ,  ...81
  "8_r-1.0_0.75_10_gt.csv" , ...82
  "8_r-1.0_0.75_11_gt.csv" , ...83
  "8_r-1.0_0.75_12_gt.csv" , ...84
  "8_r-1.0_0.75_13_gt.csv" , ...85
  "8_r-1.0_0.75_14_gt.csv" , ...86
  "8_r-1.0_0.75_15_gt.csv" , ...87
  "8_r-1.0_0.75_16_gt.csv" , ...88
  "8_r-1.0_0.75_17_gt.csv" , ...89
  "8_r-1.0_0.75_18_gt.csv" , ...90
  "8_r-1.0_0.75_19_gt.csv" , ...91
  "8_r-1.0_0.75_20_gt.csv" , ...92
  "8_r-1.0_0.75_21_gt.csv" , ...93
  "8_r-1.0_0.75_22_gt.csv" , ...94
  "8_r-1.0_1.0_8_gt.csv" ,   ...95
  "8_r-1.0_1.0_9_gt.csv" ,   ...96
  "8_r-1.0_1.0_10_gt.csv" ,  ...97
  "8_r-1.0_1.0_11_gt.csv" ,  ...98
  "8_r-1.0_1.0_12_gt.csv" ,  ...99
  "8_r-1.0_1.0_13_gt.csv" ,  ...100
  "8_r-1.0_1.0_14_gt.csv" ,  ...101
  "8_r-1.0_1.0_15_gt.csv" ,  ...102
  "8_r-1.0_1.0_16_gt.csv" ,  ...103
  "8_r-1.0_1.0_17_gt.csv" ,  ...104
  "8_r-1.0_1.0_18_gt.csv" ,  ...105
  "8_r-1.0_1.0_19_gt.csv" ,  ...106
  "8_r-1.0_1.0_20_gt.csv" ,  ...107
  "8_r-1.0_1.0_21_gt.csv" ,  ...108
  "8_r-1.0_1.0_22_gt.csv" ,  ...109
  "sq_0.5_19_gt.csv" ,       ...110
  "sq_0.5_20_gt.csv" ,       ...111
  "sq_0.5_21_gt.csv" ,       ...112
  "sq_0.5_22_gt.csv" ,       ...113
  "sq-th_0.5_3_gt.csv" ,     ...114
  "sq-th_0.5_4_gt.csv" ,     ...115
  "sq-th_0.5_5_gt.csv" ,     ...116
  "sq-th_0.5_6_gt.csv" ,     ...117
  "sq-th_0.5_7_gt.csv" ,     ...118
  "sq-th_0.5_8_gt.csv" ,     ...119
  "sq-th_0.5_9_gt.csv" ,     ...120
  "sq-th_0.5_10_gt.csv" ,    ...121
  "sq-th_0.5_11_gt.csv" ,    ...122
  "sq-th_0.5_12_gt.csv" ,    ...123
  "sq-th_0.5_13_gt.csv" ,    ...124
  "sq-th_0.5_14_gt.csv" ,    ...125
  "sq-th_0.5_15_gt.csv" ,    ...126
  "sq-th_0.5_16_gt.csv" ,    ...127
  "sq-th_0.5_17_gt.csv" ,    ...128
  "sq-th_0.5_18_gt.csv" ,    ...129
  "sq-th_0.5_19_gt.csv" ,    ...130
  "sq-th_0.5_20_gt.csv" ,    ...131
  "sq-th_0.5_21_gt.csv" ,    ...132
  "sq-th_0.5_22_gt.csv" ,    ...133
  "sq-th_0.75_3_gt.csv" ,    ...134
  "sq-th_0.75_4_gt.csv" ,    ...135
  "sq-th_1.0_3_gt.csv" ,     ...136
  "sq-th_1.0_4_gt.csv" ,     ...137
  "sq-th_1.0_5_gt.csv" ,     ...138
  "sq-th_1.0_6_gt.csv" ,     ...139
  "sq-th_1.0_7_gt.csv" ,     ...140
  "sq-th_1.0_8_gt.csv" ,     ...141
  "sq-th_1.0_9_gt.csv" ,     ...142
  "sq-th_1.0_10_gt.csv" ,    ...143
  "sq-th_1.0_11_gt.csv" ,    ...144
  "sq-th_1.0_12_gt.csv" ,    ...145
  "sq-th_1.0_13_gt.csv" ,    ...146
  "sq-th_1.0_14_gt.csv" ,    ...147
  "sq-th_1.0_15_gt.csv" ,    ...148
  "sq-th_1.0_16_gt.csv" ,    ...149
  "sq-th_1.0_17_gt.csv" ,    ...150
  "sq-th_1.0_18_gt.csv" ,    ...151
  "sq-th_1.0_19_gt.csv" ,    ...152
  "sq-th_1.0_20_gt.csv" ,    ...153
  "sq-th_1.0_21_gt.csv" ,    ...154
  "sq-th_1.0_22_gt.csv" ,    ...155
  "sq-th_1.0_23_gt.csv" ,    ...156
  "sq-th_1.0_24_gt.csv" ,    ...157
  "8_r-1.0_0.5_3_gt.csv" ,   ...158
  "8_r-1.0_0.5_4_gt.csv" ,   ...159
  "8_r-1.0_0.5_5_gt.csv" ,   ...160
  "8_r-1.0_0.5_6_gt.csv" ,   ...161
  "8_r-1.0_0.5_7_gt.csv" ,   ...162
  "8_r-1.0_0.5_8_gt.csv" ,   ...163
  "8_r-1.0_0.5_9_gt.csv" ,   ...164
  "8_r-1.0_0.5_10_gt.csv" ,  ...165
  "8_r-1.0_0.5_11_gt.csv" ,  ...166
  "8_r-1.0_0.5_12_gt.csv" ,  ...167
  "8_r-1.0_0.5_13_gt.csv" ,  ...168
  "8_r-1.0_0.5_14_gt.csv" ,  ...169
  "8_r-1.0_0.5_15_gt.csv" ,  ...170
  "8_r-1.0_0.5_16_gt.csv" ,  ...171
  "8_r-1.0_0.5_17_gt.csv" ,  ...172
  "8_r-1.0_0.5_18_gt.csv" ,  ...173
  "8_r-1.0_0.5_19_gt.csv" ,  ...174
  "8_r-1.0_0.5_20_gt.csv" ,  ...175
  "8_r-1.0_0.5_21_gt.csv" ,  ...176
  "8_r-1.0_0.5_22_gt.csv" ,  ...177
  "8_r-1.0_0.75_3_gt.csv" ,  ...178
  "8_r-1.0_0.75_4_gt.csv" ,  ...179
  "8_r-1.0_1.0_3_gt.csv" ,   ...180
  "8_r-1.0_1.0_4_gt.csv" ,   ...181
  "8_r-1.0_1.0_5_gt.csv" ,   ...182
  "8_r-1.0_1.0_6_gt.csv" ,   ...183
  "8_r-1.0_1.0_7_gt.csv" ,   ...184
  "8_r-1.0_1.0_23_gt.csv" ,  ...185
  "8_r-1.0_1.0_24_gt.csv" ,  ...186
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
filename_out = "data-analysis_raw_entire-traj_gt.csv";
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
