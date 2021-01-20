clear; clc; close all;
%% change time = 0;
Mode_Change_Torque_Fade_0 = readmatrix('Mode_Change_Torque_Fade_0.txt');
Mode_Change_Torque_Fade_0 = Mode_Change_Torque_Fade_0(1:5000,:);
S = length(Mode_Change_Torque_Fade_0);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, Mode_Change_Torque_Fade_0, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% change time = 1;
Mode_Change_Torque_Fade_1 = readmatrix('Mode_Change_Torque_Fade_1.txt');
Mode_Change_Torque_Fade_1 = Mode_Change_Torque_Fade_1(1:5000,:);
S = length(Mode_Change_Torque_Fade_1);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, Mode_Change_Torque_Fade_1, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% change time = 2;
Mode_Change_Torque_Fade_2 = readmatrix('Mode_Change_Torque_Fade_2.txt');
Mode_Change_Torque_Fade_2 = Mode_Change_Torque_Fade_2(1:5000,:);
S = length(Mode_Change_Torque_Fade_2);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, Mode_Change_Torque_Fade_2, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")

%% change time = 3;
Mode_Change_Torque_Fade_3 = readmatrix('Mode_Change_Torque_Fade_3.txt');
Mode_Change_Torque_Fade_3 = Mode_Change_Torque_Fade_3(1:5000,:);
S = length(Mode_Change_Torque_Fade_3);

dt = 0.001;
time = 1:S;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, Mode_Change_Torque_Fade_3, 'Linewidth', 2)
legend("Pelvis Yaw","Pelvis Roll","Pelvis Pitch","Knee Pitch","Ankle Pitch","Ankle Roll")
