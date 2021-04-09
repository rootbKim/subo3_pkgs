clc; close all; clear all;

tmpdata0 = readmatrix("tmpdata0.txt");
tmpdata1 = readmatrix("tmpdata1.txt");
tmpdata2 = readmatrix("tmpdata2.txt");
tmpdata3 = readmatrix("tmpdata3.txt");
tmpdata4 = readmatrix("tmpdata4.txt");
tmpdata5 = readmatrix("tmpdata5.txt");
tmpdata6 = readmatrix("tmpdata6.txt");
tmpdata7 = readmatrix("tmpdata7.txt");
tmpdata8 = readmatrix("tmpdata8.txt");
tmpdata9 = readmatrix("tmpdata9.txt");
tmpdata10 = readmatrix("tmpdata10.txt");
tmpdata11 = readmatrix("tmpdata11.txt");
tmpdata12 = readmatrix("tmpdata12.txt");
tmpdata13 = readmatrix("tmpdata13.txt");
tmpdata14 = readmatrix("tmpdata14.txt");
tmpdata15 = readmatrix("tmpdata15.txt");
tmpdata16 = readmatrix("tmpdata16.txt");
tmpdata17 = readmatrix("tmpdata17.txt");
tmpdata18 = readmatrix("tmpdata18.txt");
tmpdata19 = readmatrix("tmpdata19.txt");
tmpdata20 = readmatrix("tmpdata20.txt");

S_total = min([length(tmpdata0),length(tmpdata1),length(tmpdata2),length(tmpdata3),length(tmpdata4),length(tmpdata5),length(tmpdata6),length(tmpdata7),length(tmpdata8),length(tmpdata9),length(tmpdata10),length(tmpdata11),length(tmpdata12),length(tmpdata13), length(tmpdata14),length(tmpdata15),length(tmpdata16),length(tmpdata17),length(tmpdata18),length(tmpdata19),length(tmpdata20)]);

S = S_total-2466; % 2468~2466

%%
tmpdata0 = tmpdata0(1:S,:);
tmpdata1 = tmpdata1(1:S,:);
tmpdata2 = tmpdata2(1:S,:);
tmpdata3 = tmpdata3(1:S,:);
tmpdata4 = tmpdata4(1:S,:);
tmpdata5 = tmpdata5(1:S,:);
tmpdata6 = tmpdata6(1:S,:);
tmpdata7 = tmpdata7(1:S,:);
tmpdata8 = tmpdata8(1:S,:);
tmpdata9 = tmpdata9(1:S,:);
tmpdata10 = tmpdata10(1:S,:);
tmpdata11 = tmpdata11(1:S,:);
tmpdata12 = tmpdata12(1:S,:);
tmpdata13 = tmpdata13(1:S,:);
tmpdata14 = tmpdata14(1:S,:);
tmpdata15 = tmpdata15(1:S,:);
tmpdata16 = tmpdata16(1:S,:);
tmpdata17 = tmpdata17(1:S,:);
tmpdata18 = tmpdata18(1:S,:);
tmpdata19 = tmpdata19(1:S,:);
tmpdata20 = tmpdata20(1:S,:);

dt = 0.001;
time = 1:1:S;
time = time*dt;

%% zmp

X_L0 = tmpdata0(:,1);
Y_L0 = tmpdata0(:,2);
X_R0 = tmpdata0(:,3);
Y_R0 = tmpdata0(:,4);
X_ZMP = tmpdata0(:,5);
Y_ZMP = tmpdata0(:,6);
Y_ZMP_point = tmpdata0(:,7);
L_total = tmpdata0(:,8);
L_half = tmpdata0(:,9);
L_zmp = tmpdata0(:,10);
zmp_factor = tmpdata0(:,11);

X_L0_1 = X_L0 + 0.024 + 0.092;
X_L0_2 = X_L0 + 0.024 - 0.092;
Y_L0_1 = Y_L0 + 0.07;
Y_L0_2 = Y_L0 - 0.05;

X_R0_1 = X_R0 + 0.024 + 0.092;
X_R0_2 = X_R0 + 0.024 - 0.092;
Y_R0_1 = Y_R0 + 0.05;
Y_R0_2 = Y_R0 - 0.07;

L_Fz = tmpdata1(:,1);
R_Fz = tmpdata1(:,2);
L_Mx = tmpdata1(:,3);
L_My = tmpdata1(:,4);
R_Mx = tmpdata1(:,5);
R_My = tmpdata1(:,6);

X_ZMP_point = (X_L0 + X_R0) / 2;

fig = figure;
subplot(2,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('FT Sensor','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Force [N] / Torque [Nm]','FontSize',14)
plot(time, tmpdata1)
legend("Left Fz", "Right Fz", "Left Mx", "Left My", "Right Mx", "Right My")

subplot(2,2,2);
hold on;
grid on;
box on;
set(gca,'FontSize',14)
title('ZMP Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Position [m]','FontSize',14)
plot(time, X_ZMP)
plot(time, Y_ZMP)
legend("ZMP X", "ZMP Y");

subplot(2,2,3);
hold on;
grid on;
box on;
axis([-0.2, 0.2, -0.2, 0.2]);
axis equal;
set(gca,'FontSize',14)
title('ZMP Position','FontSize',16)
xlabel('Position X [m]','FontSize',14)
ylabel('Position Y [m]','FontSize',14)

line([X_L0_1(1), X_L0_1(1)],[Y_L0_1(1), Y_L0_2(1)], 'Color', 'blue')
line([X_L0_1(1), X_L0_2(1)],[Y_L0_2(1), Y_L0_2(1)], 'Color', 'blue')
line([X_L0_2(1), X_L0_2(1)],[Y_L0_1(1), Y_L0_2(1)], 'Color', 'blue')
line([X_L0_1(1), X_L0_2(1)],[Y_L0_1(1), Y_L0_1(1)], 'Color', 'blue')
line([X_R0_1(1), X_R0_1(1)],[Y_R0_1(1), Y_R0_2(1)], 'Color', 'blue')
line([X_R0_1(1), X_R0_2(1)],[Y_R0_2(1), Y_R0_2(1)], 'Color', 'blue')
line([X_R0_2(1), X_R0_2(1)],[Y_R0_1(1), Y_R0_2(1)], 'Color', 'blue')
line([X_R0_1(1), X_R0_2(1)],[Y_R0_1(1), Y_R0_1(1)], 'Color', 'blue')

plot(X_L0, Y_L0, "Linewidth", 10);
plot(X_R0, Y_R0, "Linewidth", 10);
plot(X_ZMP, Y_ZMP);
plot(X_ZMP_point, Y_ZMP_point, "Linewidth", 10);

subplot(2,2,4);
hold on;
grid on;
box on;
set(gca,'FontSize',14)
title('ZMP Factor','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('ZMP Factor','FontSize',14)
plot(time, L_zmp./L_half);
plot(time, zmp_factor);

%%

A_L_Foot_Pos = tmpdata2;
A_R_Foot_Pos = tmpdata3;
G_L_Foot_Pos = tmpdata4;
G_R_Foot_Pos = tmpdata5;
O_L_Foot_Pos = tmpdata6;
O_R_Foot_Pos = tmpdata7;

A_L_Foot_Pos_dot = tmpdata8;
A_R_Foot_Pos_dot = tmpdata9;
G_L_Foot_Pos_dot = tmpdata10;
G_R_Foot_Pos_dot = tmpdata11;
O_L_Foot_Pos_dot = tmpdata12;
O_R_Foot_Pos_dot = tmpdata13;

A_L_torque_CTC = tmpdata14;
A_R_torque_CTC = tmpdata15;
G_L_torque_CTC = tmpdata16;
G_R_torque_CTC = tmpdata17;
O_L_torque_CTC = tmpdata18;
O_R_torque_CTC = tmpdata19;

Global_Pos = tmpdata20;

%% Pos

fig = figure;
subplot(3,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Left Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, A_L_Foot_Pos, 'linewidth', 2)
legend("x", "y", "z", "r", "p", "y")

subplot(3,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Right Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, A_R_Foot_Pos, 'linewidth', 2)

subplot(3,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Left Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, G_L_Foot_Pos, 'linewidth', 2)

subplot(3,2,4);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Right Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, G_R_Foot_Pos, 'linewidth', 2)

subplot(3,2,5);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Left Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, O_L_Foot_Pos, 'linewidth', 2)

subplot(3,2,6);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Right Foot Pos','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m] / Angular [rad]','FontSize',14)
plot(time, O_R_Foot_Pos, 'linewidth', 2)

%% Pos dot
fig = figure;
subplot(3,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Left Foot Pos dot','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, A_L_Foot_Pos_dot, 'linewidth', 2)
legend("x", "y", "z", "r", "p", "y")

subplot(3,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Right Foot Pos dot','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, A_R_Foot_Pos_dot, 'linewidth', 2)

subplot(3,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Left Foot Pos dot','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, G_L_Foot_Pos_dot, 'linewidth', 2)

subplot(3,2,4);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Right Foot Pos dot ','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, G_R_Foot_Pos_dot, 'linewidth', 2)

subplot(3,2,5);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Left Foot Pos dot','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, O_L_Foot_Pos_dot, 'linewidth', 2)

subplot(3,2,6);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Right Foot Pos dot','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear [m/s] / Angular [rad/s]','FontSize',14)
plot(time, O_R_Foot_Pos_dot, 'linewidth', 2)

%% CTC torque

fig = figure;
subplot(3,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Left joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, A_L_torque_CTC, 'linewidth', 2)
legend("pelvis yaw", "pelvis roll", "pelvis pitch", "knee pitch", "ankle pitch", "ankle roll")

subplot(3,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Pelvis CTC Right joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, A_R_torque_CTC, 'linewidth', 2)

subplot(3,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Left joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, G_L_torque_CTC, 'linewidth', 2)

subplot(3,2,4);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Two Foot CTC Right joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, G_R_torque_CTC, 'linewidth', 2)

subplot(3,2,5);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Left joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, O_L_torque_CTC, 'linewidth', 2)

subplot(3,2,6);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('One Foot CTC Right joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint Torque [Nm]','FontSize',14)
plot(time, O_R_torque_CTC, 'linewidth', 2)
