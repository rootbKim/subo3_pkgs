clc; clear; close all;

L_Des_X = readmatrix('tmpdata0.txt');
L_Foot_Pos = readmatrix('tmpdata1.txt');
L_torque = readmatrix('tmpdata2.txt');
R_Des_X = readmatrix('tmpdata3.txt');
R_Foot_Pos = readmatrix('tmpdata4.txt');
R_torque = readmatrix('tmpdata5.txt');

L_Des_X = L_Des_X(1:50000,:);
L_Foot_Pos = L_Foot_Pos(1:50000,:);
L_torque = L_torque(1:50000,:);

R_Des_X = R_Des_X(1:50000,:);
R_Foot_Pos = R_Foot_Pos(1:50000,:);
R_torque = R_torque(1:50000,:);

dt = 0.001;
time = 1:50000;
time = time * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position - LEFT','FontSize',16)
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
title('CTC Torque - LEFT','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, L_torque, 'Linewidth', 2)
legend("Ankle Roll","Ankle Pitch","Knee Pitch","Pelvis Pitch","Pelvis Roll","Pelvis Yaw")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position - RIGHT','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, R_Des_X, 'Linewidth', 2)
plot(time, R_Foot_Pos, 'Linewidth', 2)
legend("Desired x","Desired y","Desired z","Desired roll","Desired pitch","Desired yaw","Actual x","Actual y","Actual z","Actual roll","Actual pitch","Actual yaw")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('CTC Torque - RIGHT','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, R_torque, 'Linewidth', 2)
legend("Ankle Roll","Ankle Pitch","Knee Pitch","Pelvis Pitch","Pelvis Roll","Pelvis Yaw")
