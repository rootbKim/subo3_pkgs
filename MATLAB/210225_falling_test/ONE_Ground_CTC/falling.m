clc; close all; clear all;

torque = readmatrix("tmpdata0.txt");
Q = readmatrix("tmpdata1.txt");
QDot = readmatrix("tmpdata2.txt");

S = min([length(torque),length(Q),length(QDot)]);

torque = torque(1:S,:);
Q = Q(1:S,:);
QDot = QDot(1:S,:);

dt = 0.001;
time = 1:1:S;
time = time*dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint torque [Nm]','FontSize',14)
plot(time, torque,'linewidth',2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Angle','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Angle [rad]','FontSize',14)
plot(time, Q,'linewidth',2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Angular Velocity','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Angular velocity [rad/sec]','FontSize',14)
plot(time, QDot,'linewidth',2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");