clc; close all; clear all;


tmpdata0 = readmatrix("tmpdata0.txt");
tmpdata1 = readmatrix("tmpdata1.txt");
left_torque = readmatrix("tmpdata2.txt");
right_torque = readmatrix("tmpdata3.txt");
left_angle = readmatrix("tmpdata4.txt");
right_angle = readmatrix("tmpdata5.txt");

left_zmp_torque = readmatrix("tmpdata6.txt");
right_zmp_torque = readmatrix("tmpdata7.txt");

S_total = min([length(tmpdata0),length(tmpdata1),length(left_torque),length(right_torque),length(left_angle),length(right_angle),length(left_zmp_torque),length(right_zmp_torque)]);

S = S_total - 500;

%% phase 1
tmpdata0 = tmpdata0(8001:S,:);
tmpdata1 = tmpdata1(8001:S,:);
left_torque = left_torque(8001:S,:);
right_torque = right_torque(8001:S,:);
left_angle = left_angle(8001:S,:);
right_angle = right_angle(8001:S,:);
left_zmp_torque = left_zmp_torque(8001:S,:);
right_zmp_torque = right_zmp_torque(8001:S,:);

dt = 0.001;
time = 1:1:S-8000;
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

X_L0_1 = X_L0 + 0.1 + 0.04;
X_L0_2 = X_L0 - 0.1 + 0.04;
Y_L0_1 = Y_L0 + 0.05;
Y_L0_2 = Y_L0 - 0.05;

X_R0_1 = X_R0 + 0.1 + 0.04;
X_R0_2 = X_R0 - 0.1 + 0.04;
Y_R0_1 = Y_R0 + 0.05;
Y_R0_2 = Y_R0 - 0.05;

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

fig = figure;
subplot(2,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Left Leg Joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, left_torque, 'linewidth', 2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Right Leg Joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, right_torque, 'linewidth', 2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Left Leg Joint angle','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, left_angle, 'linewidth', 2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,4);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Right Leg Joint angle','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, right_angle, 'linewidth', 2)
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

fig = figure;
subplot(2,2,1);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14);
title('Right Leg First Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, right_zmp_torque(:,2:7));
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,2);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14);
title('Right Leg Second Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, right_zmp_torque(:,8:end));
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,3);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14);
title('Left Leg First Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, left_zmp_torque(:,2:7));
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");

subplot(2,2,4);
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14);
title('Left Leg Second Joint Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, left_zmp_torque(:,8:end));
legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");
%% phase 2
% 
% tmpdata0 = readmatrix("tmpdata0.txt");
% tmpdata1 = readmatrix("tmpdata1.txt");
% left_torque = readmatrix("tmpdata2.txt");
% right_torque = readmatrix("tmpdata3.txt");
% left_angle = readmatrix("tmpdata4.txt");
% right_angle = readmatrix("tmpdata5.txt");
% 
% tmpdata0 = tmpdata0(S:S_total,:);
% tmpdata1 = tmpdata1(S:S_total,:);
% left_torque = left_torque(S:S_total,:);
% right_torque = right_torque(S:S_total,:);
% left_angle = left_angle(S:S_total,:);
% right_angle = right_angle(S:S_total,:);
% 
% dt = 0.001;
% time = S:1:S_total;
% time = time*dt;
% 
% X_L0 = tmpdata0(:,1);
% Y_L0 = tmpdata0(:,2);
% X_R0 = tmpdata0(:,3);
% Y_R0 = tmpdata0(:,4);
% X_ZMP = tmpdata0(:,5);
% Y_ZMP = tmpdata0(:,6);
% Y_ZMP_point = tmpdata0(:,7);
% L_total = tmpdata0(:,8);
% L_half = tmpdata0(:,9);
% L_zmp = tmpdata0(:,10);
% zmp_factor = tmpdata0(:,11);
% 
% X_L0_1 = X_L0 + 0.1 + 0.04;
% X_L0_2 = X_L0 - 0.1 + 0.04;
% Y_L0_1 = Y_L0 + 0.05;
% Y_L0_2 = Y_L0 - 0.05;
% 
% X_R0_1 = X_R0 + 0.1 + 0.04;
% X_R0_2 = X_R0 - 0.1 + 0.04;
% Y_R0_1 = Y_R0 + 0.05;
% Y_R0_2 = Y_R0 - 0.05;
% 
% L_Fz = tmpdata1(:,1);
% R_Fz = tmpdata1(:,2);
% L_Mx = tmpdata1(:,3);
% L_My = tmpdata1(:,4);
% R_Mx = tmpdata1(:,5);
% R_My = tmpdata1(:,6);
% 
% X_ZMP_point = (X_L0 + X_R0) / 2;
% 
% fig = figure;
% subplot(2,2,1);
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('FT Sensor','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Force [N] / Torque [Nm]','FontSize',14)
% plot(time, tmpdata1)
% legend("Left Fz", "Right Fz", "Left Mx", "Left My", "Right Mx", "Right My")
% 
% subplot(2,2,2);
% hold on;
% grid on;
% box on;
% set(gca,'FontSize',14)
% title('ZMP Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Position [m]','FontSize',14)
% plot(time, X_ZMP)
% plot(time, Y_ZMP)
% legend("ZMP X", "ZMP Y");
% 
% subplot(2,2,3);
% hold on;
% grid on;
% box on;
% axis([-0.2, 0.2, -0.2, 0.2]);
% axis equal;
% set(gca,'FontSize',14)
% title('ZMP Position','FontSize',16)
% xlabel('Position X [m]','FontSize',14)
% ylabel('Position Y [m]','FontSize',14)
% 
% line([X_L0_1(1), X_L0_1(1)],[Y_L0_1(1), Y_L0_2(1)], 'Color', 'blue')
% line([X_L0_1(1), X_L0_2(1)],[Y_L0_2(1), Y_L0_2(1)], 'Color', 'blue')
% line([X_L0_2(1), X_L0_2(1)],[Y_L0_1(1), Y_L0_2(1)], 'Color', 'blue')
% line([X_L0_1(1), X_L0_2(1)],[Y_L0_1(1), Y_L0_1(1)], 'Color', 'blue')
% line([X_R0_1(1), X_R0_1(1)],[Y_R0_1(1), Y_R0_2(1)], 'Color', 'blue')
% line([X_R0_1(1), X_R0_2(1)],[Y_R0_2(1), Y_R0_2(1)], 'Color', 'blue')
% line([X_R0_2(1), X_R0_2(1)],[Y_R0_1(1), Y_R0_2(1)], 'Color', 'blue')
% line([X_R0_1(1), X_R0_2(1)],[Y_R0_1(1), Y_R0_1(1)], 'Color', 'blue')
% 
% plot(X_L0, Y_L0, "Linewidth", 10);
% plot(X_R0, Y_R0, "Linewidth", 10);
% plot(X_ZMP, Y_ZMP);
% plot(X_ZMP_point, Y_ZMP_point, "Linewidth", 10);
% 
% subplot(2,2,4);
% hold on;
% grid on;
% box on;
% set(gca,'FontSize',14)
% title('ZMP Factor','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('ZMP Factor','FontSize',14)
% plot(time, L_zmp./L_half);
% plot(time, zmp_factor);
% 
% fig = figure;
% subplot(2,2,1);
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Left Leg Joint torque','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Torque [Nm]','FontSize',14)
% plot(time, left_torque, 'linewidth', 2)
% legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");
% 
% subplot(2,2,2);
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Right Leg Joint torque','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Torque [Nm]','FontSize',14)
% plot(time, right_torque, 'linewidth', 2)
% legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");
% 
% subplot(2,2,3);
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Left Leg Joint angle','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Torque [Nm]','FontSize',14)
% plot(time, left_angle, 'linewidth', 2)
% legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");
% 
% subplot(2,2,4);
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Right Leg Joint angle','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Torque [Nm]','FontSize',14)
% plot(time, right_angle, 'linewidth', 2)
% legend("Pelvis Yaw", "Pelvis Roll", "Pelvis Pitch", "Knee Pitch", "Ankle Pitch", "Ankle Roll");