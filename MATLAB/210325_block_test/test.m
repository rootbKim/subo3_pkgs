clc; close all; clear all;


tmpdata0 = readmatrix("tmpdata0.txt");
tmpdata1 = readmatrix("tmpdata1.txt");
tmpdata2 = readmatrix("tmpdata2.txt");
tmpdata3 = readmatrix("tmpdata3.txt");
tmpdata4 = readmatrix("tmpdata4.txt");
tmpdata5 = readmatrix("tmpdata5.txt");
tmpdata6 = readmatrix("tmpdata6.txt");

S_total = min([length(tmpdata0),length(tmpdata1),length(tmpdata2),length(tmpdata3),length(tmpdata4),length(tmpdata5),length(tmpdata6)]);

S = S_total;

%% phase 1
tmpdata0 = tmpdata0(1:S,:);
tmpdata1 = tmpdata1(1:S,:);
tmpdata2 = tmpdata2(1:S,:);
tmpdata3 = tmpdata3(1:S,:);
tmpdata4 = tmpdata4(1:S,:);
tmpdata5 = tmpdata5(1:S,:);
tmpdata6 = tmpdata6(1:S,:);

dt = 0.001;
time = 1:1:S;
time = time*dt;

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

G_L_Des_X = tmpdata2;
G_R_Des_X = tmpdata3;
G_L_Foot_Pos = tmpdata4;
G_R_Foot_Pos = tmpdata5;
Global_Pos = tmpdata6;

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
fig = figure;
subplot(2,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('FT Sensor','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos [m] / [rad]','FontSize',14)
plot(time, G_L_Des_X)
plot(time, G_L_Foot_Pos)
legend("x", "y", "z", "rll", "pit", "yaw")

subplot(2,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('FT Sensor','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos [m] / [rad]','FontSize',14)
plot(time, G_R_Des_X)
plot(time, G_R_Foot_Pos)
legend("x", "y", "z", "rll", "pit", "yaw")

subplot(2,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('FT Sensor','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Pos [m] / [rad]','FontSize',14)
plot(time, Global_Pos)
legend("x", "y", "z", "rll", "pit", "yaw")
