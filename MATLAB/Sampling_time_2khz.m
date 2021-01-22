clear; clc; close all;

L_Des_X = readmatrix('tmpdata0.txt');
L_Foot_Pos = readmatrix('tmpdata1.txt');

L = [(length(L_Des_X)),length(L_Foot_Pos)];
S = min(L);

L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, L_Des_X, 'Linewidth', 2)
plot(time, L_Foot_Pos, 'Linewidth', 2)
legend("Desired x","Desired y","Desired z","Desired roll","Desired pitch","Desired yaw","Actual x","Actual y","Actual z","Actual roll","Actual pitch","Actual yaw")