clc; clear all; close all;

L_Des_X = readmatrix('L_Des_X.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos.txt');
L_torque_CTC = readmatrix('L_torque_CTC.txt');

L = [(length(L_Des_X)),length(L_Foot_Pos),length(L_torque_CTC)];
S = min(L);

L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);
L_torque_CTC = L_torque_CTC(1:S,:);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-x [m]','FontSize',14)
plot(time, L_Des_X(:,1), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,1), 'Linewidth', 2)
legend("Desired x","Actual x")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-y','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-y [m]','FontSize',14)
plot(time, L_Des_X(:,2), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,2), 'Linewidth', 2)
legend("Desired y","Actual y")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-z','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-z [m]','FontSize',14)
plot(time, L_Des_X(:,3), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,3), 'Linewidth', 2)
legend("Desired z","Actual z")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-roll [rad]','FontSize',14)
plot(time, L_Des_X(:,4), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,4), 'Linewidth', 2)
legend("Desired roll","Actual roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-y','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-pitch [yaw]','FontSize',14)
plot(time, L_Des_X(:,5), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,5), 'Linewidth', 2)
legend("Desired pitch","Actual pitch")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pos-z','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos-yaw [rad]','FontSize',14)
plot(time, L_Des_X(:,6), 'Linewidth', 2)
plot(time, L_Foot_Pos(:,6), 'Linewidth', 2)
legend("Desired yaw","Actual yaw")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, L_torque_CTC, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")