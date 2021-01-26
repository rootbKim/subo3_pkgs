clc; clear; close all;
%% 1khz
L_Des_X = readmatrix('L_Des_X_1.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_1.txt');
L_torque = readmatrix('L_torque_1.txt');
joint_torque = readmatrix('joint_torque_1.txt');

%% left
L = [(length(L_Des_X)),length(L_Foot_Pos),length(L_torque),length(joint_torque)];
S = min(L);
S = 3000;
L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);
L_torque = L_torque(1:S,:);
joint_torque = joint_torque(1:S,:);

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

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, L_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, joint_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% 2khz
L_Des_X = readmatrix('L_Des_X_2.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_2.txt');
L_torque = readmatrix('L_torque_2.txt');
joint_torque = readmatrix('joint_torque_2.txt');

%% left
L = [(length(L_Des_X)),length(L_Foot_Pos),length(L_torque),length(joint_torque)];
S = min(L);
S = 3000;
L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);
L_torque = L_torque(1:S,:);
joint_torque = joint_torque(1:S,:);

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

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, L_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, joint_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% 5khz
L_Des_X = readmatrix('L_Des_X_5.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_5.txt');
L_torque = readmatrix('L_torque_5.txt');
joint_torque = readmatrix('joint_torque_5.txt');

%% left
L = [(length(L_Des_X)),length(L_Foot_Pos),length(L_torque),length(joint_torque)];
S = min(L);
S = 3000;
L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);
L_torque = L_torque(1:S,:);
joint_torque = joint_torque(1:S,:);

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

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, L_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, joint_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% 5khz - Gain
L_Des_X = readmatrix('L_Des_X_5_Gain.txt');
L_Foot_Pos = readmatrix('L_Foot_Pos_5_Gain.txt');
L_torque = readmatrix('L_torque_5_Gain.txt');
joint_torque = readmatrix('joint_torque_5_Gain.txt');

%% left
L = [(length(L_Des_X)),length(L_Foot_Pos),length(L_torque),length(joint_torque)];
S = min(L);
S = 3000;
L_Des_X = L_Des_X(1:S,:);
L_Foot_Pos = L_Foot_Pos(1:S,:);
L_torque = L_torque(1:S,:);
joint_torque = joint_torque(1:S,:);

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

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, L_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, joint_torque, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")
