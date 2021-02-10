clc; clear; close all;

A_Des_X = readmatrix('A_Des_X.txt');
A_Foot_Pos = readmatrix('A_Foot_Pos.txt');
A_torque = readmatrix('A_torque.txt');

G_Des_X = readmatrix('G_Des_X.txt');
G_Foot_Pos = readmatrix('G_Foot_Pos.txt');
G_torque = readmatrix('G_torque.txt');

A_Des_X = A_Des_X(1:4000,:);
A_Foot_Pos = A_Foot_Pos(1:4000,:);
A_torque = A_torque(1:4000,:);

G_Des_X = G_Des_X(1:4000,:);
G_Foot_Pos = G_Foot_Pos(1:4000,:);
G_torque = G_torque(1:4000,:);

dt = 0.001;
time = 1:4000;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position - AIR CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, A_Des_X, 'Linewidth', 2)
plot(time, A_Foot_Pos, 'Linewidth', 2)
legend("Desired x","Desired y","Desired z","Desired roll","Desired pitch","Desired yaw","Actual x","Actual y","Actual z","Actual roll","Actual pitch","Actual yaw")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - AIR CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, A_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position - GROUND CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, G_Des_X, 'Linewidth', 2)
plot(time, G_Foot_Pos, 'Linewidth', 2)
legend("Desired x","Desired y","Desired z","Desired roll","Desired pitch","Desired yaw","Actual x","Actual y","Actual z","Actual roll","Actual pitch","Actual yaw")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - GROUND CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, G_torque, 'Linewidth', 2)
legend("Ankle Roll","Ankle Pitch","Knee Pitch","Pelvis Pitch","Pelvis Roll","Pelvis Yaw")
