clear; clc; close all;
%% sampling 1khz
L_Des_X = readmatrix('L_Des_X_1.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_1.txt');

L = [(length(L_Des_X)),length(L_Foot_Pos)];
S = min(L);

L_Des_X = L_Des_X(18000:22000,:);
L_Foot_Pos = L_Foot_Pos(18000:22000,:);

dt = 0.001;
time = 1:4001;
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

error = L_Des_X - L_Foot_Pos;
rms(error)
%% sampling 2khz
L_Des_X = readmatrix('L_Des_X_2.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_2.txt');

L = [(length(L_Des_X)),length(L_Foot_Pos)];
S = min(L);

L_Des_X = L_Des_X(3800:7800,:);
L_Foot_Pos = L_Foot_Pos(3800:7800,:);

dt = 0.001;
time = 1:4001;
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

error = L_Des_X - L_Foot_Pos;
rms(error)
