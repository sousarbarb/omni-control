close all
clear all
clc

%% INITIALIZE
V0 = 1.5;
Vd = 0.5;
Vin  = [-15    -V0 -Vd Vd V0 15];
Wout = [-15+V0   0   0  0  0 15-V0];

%% HAMMERSTEIN
H = @(x) (( x - Vd ) + V0) .* ( x >  Vd ) + ...
         (( x + Vd ) - V0) .* ( x < -Vd ) + ...
         ( x*V0/Vd ) .* ((x <= Vd) & (x >= -Vd));

%% PLOT
VinH  = [-15+V0-Vd -Vd Vd 15-V0+Vd];
WoutH = [-15+V0      0  0 15-V0];

figure
hold on
plot(Vin, Wout)
plot(VinH, WoutH)
grid on
axis equal
xlim([-7.5,7.5])
ylim([-7.5,7.5])
xlabel('V_i (V) \rightarrow')
ylabel('dφ_i/dt (rad.s^{-1}) \rightarrow')
legend('without Hammerstein','with Hammerstein')
title('Illustrative example of the effect of a Hammerstein block')

figure
plot(Vin, H(Vin))