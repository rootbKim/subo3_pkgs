clc; clear; close all;

%% 1khz
torque = readmatrix('torque1.txt');

S = length(torque);

dt = 0.0005;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - AIR CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")


%% 2khz
torque = readmatrix('torque2.txt');

S = length(torque);

dt = 0.0005;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - AIR CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% 5khz
torque = readmatrix('torque5.txt');

S = length(torque);

dt = 0.0005;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - AIR CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")