clear all
close all
clc

%% INITIALIZE
FSize = 3:22';

Sq050 = [
6.927763937
4.669721314
3.357474609
3.630554139
4.501764791
5.388961057
6.090325934
6.799593388
7.429921157
7.889199608
8.307568341
8.694585218
9.026105529
9.314360886
9.567637917
9.786012038
9.973325647
10.14444302
10.29848568
10.43945569
];

Sq075 = [
16.36276755
12.72601881
10.47205718
7.78463226
5.851930743
5.292674109
6.155603356
7.049474023
7.900791806
8.786381714
9.612991999
10.30388951
10.92010191
11.49691543
12.08028068
12.60611084
13.09241997
13.53315457
13.92041139
14.25356281
];

Sq100 = [
29.65137252
26.11755194
23.27245157
20.00100922
16.69295588
13.8076126
11.15587245
9.557780637
9.429494961
10.36163011
11.24557937
12.10850513
12.99660663
13.94746124
14.77773714
15.55069151
16.32067286
16.9925284
17.53428448
18.02022909
];

SqTh050 = [
6.193134785
4.459625939
3.232954099
3.730280556
4.596182837
5.416829054
6.036092012
6.648777661
7.330717421
7.854519582
8.317709803
8.733699222
9.107339792
9.442595002
9.740117095
9.982622363
10.20177253
10.39793806
10.58670809
10.76084626
];

SqTh075 = [
14.95978532
11.44658856
9.423079557
7.433801384
6.183525125
5.980963852
6.804554997
7.751836
8.656371071
9.481258941
10.25557472
11.08718605
11.85336508
12.47066805
12.99213918
13.46687427
13.90200105
14.31373606
14.69816359
15.07855294
];

SqTh100 = [
27.68064111
23.78322684
20.5821578
17.46805894
15.05333587
12.95004992
10.95407335
10.1321547
11.12074543
12.19411528
12.88383243
13.7460886
14.56481408
15.30386078
16.00686156
16.67721941
17.31863027
17.9146953
18.47026754
18.99061612
];

S8050 = [
2.181409076
1.918313072
1.876784324
1.886450916
1.910716339
1.956302969
2.045762233
2.162974264
2.377801821
2.648146887
2.913640321
3.151947455
3.420445974
3.688438691
3.959986819
4.22887037
4.490143278
4.750835743
5.01564943
5.277122207
];

S8075 = [
3.557709134
3.206412969
2.809408711
2.503179693
2.406557515
2.421974474
2.501456708
2.618024372
2.861778757
3.202973607
3.56742863
3.953826577
4.344610973
4.73408777
5.139552658
5.543994481
5.955561333
6.371351076
6.787479069
7.169596981
];

S8100 = [
5.522088561
4.957299527
4.463123534
4.042909876
3.577382995
3.344733945
3.315989603
3.360140771
3.474756779
3.807038354
4.226739437
4.681098941
5.178799558
5.733902082
6.305216605
6.845951631
7.389662183
7.930643616
8.45863821
8.997409059
];

%% VISUALIZATION

figure
colors = get(gca,'colororder');

hold on
plot(FSize,Sq050  , ':' , 'Marker' , 's' , 'Color' , colors(1,:) )
plot(FSize,Sq075  , ':' , 'Marker' , 'o' , 'Color' , colors(1,:) )
plot(FSize,Sq100  , ':' , 'Marker' , '+' , 'Color' , colors(1,:) )
plot(FSize,SqTh050, ':' , 'Marker' , 's' , 'Color' , colors(2,:) )
plot(FSize,SqTh075, ':' , 'Marker' , 'o' , 'Color' , colors(2,:) )
plot(FSize,SqTh100, ':' , 'Marker' , '+' , 'Color' , colors(2,:) )
plot(FSize,S8050  , ':' , 'Marker' , 's' , 'Color' , colors(3,:) )
plot(FSize,S8075  , ':' , 'Marker' , 'o' , 'Color' , colors(3,:) )
plot(FSize,S8100  , ':' , 'Marker' , '+' , 'Color' , colors(3,:) )
grid on
xlim([FSize(1) FSize(end)])
ylim([0.95*min([Sq050;Sq075;Sq100;SqTh050;SqTh075;SqTh100;S8050;S8075;S8100]) ...
      1.05*max([Sq050;Sq075;Sq100;SqTh050;SqTh075;SqTh100;S8050;S8075;S8100])])
xlabel('M \rightarrow')
ylabel('ε_{h} \rightarrow')
legend('sq_{0.5m/s}','sq_{0.75m/s}','sq_{1.0m/s}',...
       'sq w/\theta^W var._{0.5m/s}','sq w/\theta^W var._{0.75m/s}','sq w/\theta^W var._{1.0m/s}',...
       '8_{0.5m/s}','8_{0.75m/s}','8_{1.0m/s}')
title('Analysis of the future size M')
set(gcf, 'Position', [500,150,500,500])





figure
colors = get(gca,'colororder');

hold on
plot(FSize,Sq050  , ':' , 'Marker' , 's' , 'Color' , colors(1,:) )
plot(FSize,Sq075  , ':' , 'Marker' , 'o' , 'Color' , colors(1,:) )
plot(FSize,Sq100  , ':' , 'Marker' , '+' , 'Color' , colors(1,:) )
grid on
xlim([FSize(1) FSize(end)])
ylim([0.95*min([Sq050;Sq075;Sq100]) ...
      1.05*max([Sq050;Sq075;Sq100])])
xlabel('M \rightarrow')
ylabel('ε_{h} \rightarrow')
legend('sq_{0.5m/s}','sq_{0.75m/s}','sq_{1.0m/s}')
title('Analysis of the future size M:','2x2m square')
set(gcf, 'Position', [500,150,500,500])
saveas(gcf,'future-traj-analysis_sq','epsc')
saveas(gcf,'future-traj-analysis_sq','fig')





figure
colors = get(gca,'colororder');

hold on
plot(FSize,Sq050  , ':' , 'Marker' , 's' , 'Color' , colors(1,:) )
plot(FSize,Sq075  , ':' , 'Marker' , 'o' , 'Color' , colors(1,:) )
plot(FSize,Sq100  , ':' , 'Marker' , '+' , 'Color' , colors(1,:) )
plot(FSize,SqTh050, ':' , 'Marker' , 's' , 'Color' , colors(2,:) )
plot(FSize,SqTh075, ':' , 'Marker' , 'o' , 'Color' , colors(2,:) )
plot(FSize,SqTh100, ':' , 'Marker' , '+' , 'Color' , colors(2,:) )
grid on
xlim([FSize(1) FSize(end)])
ylim([0.95*min([Sq050;Sq075;Sq100;SqTh050;SqTh075;SqTh100]) ...
      1.05*max([Sq050;Sq075;Sq100;SqTh050;SqTh075;SqTh100])])
xlabel('M \rightarrow')
ylabel('ε_{h} \rightarrow')
legend('sq_{0.5m/s}','sq_{0.75m/s}','sq_{1.0m/s}',...
       'sq w/\theta^W var._{0.5m/s}','sq w/\theta^W var._{0.75m/s}','sq w/\theta^W var._{1.0m/s}')
title('Analysis of the future size M:','2x2m square versus 2x2m square with variable orientation')
set(gcf, 'Position', [500,150,500,500])
saveas(gcf,'future-traj-analysis_sqth','epsc')
saveas(gcf,'future-traj-analysis_sqth','fig')





figure
colors = get(gca,'colororder');

hold on
plot(FSize,Sq050  , ':' , 'Marker' , 's' , 'Color' , colors(1,:) )
plot(FSize,Sq075  , ':' , 'Marker' , 'o' , 'Color' , colors(1,:) )
plot(FSize,Sq100  , ':' , 'Marker' , '+' , 'Color' , colors(1,:) )
plot(FSize,S8050  , ':' , 'Marker' , 's' , 'Color' , colors(3,:) )
plot(FSize,S8075  , ':' , 'Marker' , 'o' , 'Color' , colors(3,:) )
plot(FSize,S8100  , ':' , 'Marker' , '+' , 'Color' , colors(3,:) )
grid on
xlim([FSize(1) FSize(end)])
ylim([0.95*min([S8050;S8075;S8100]) ...
      1.05*max([S8050;S8075;S8100])])
xlabel('M \rightarrow')
ylabel('ε_{h} \rightarrow')
legend('sq_{0.5m/s}','sq_{0.75m/s}','sq_{1.0m/s}',...
       '8_{0.5m/s}','8_{0.75m/s}','8_{1.0m/s}')
title('Analysis of the future size M:','2x2m square versus 8-shape')
set(gcf, 'Position', [500,150,500,500])
saveas(gcf,'future-traj-analysis_8','epsc')
saveas(gcf,'future-traj-analysis_8','fig')