clc; clear; close all;

data = readmatrix("FT_Sensor_apply_force.txt");

L_Force_X = data(:,1);L_Force_Y = data(:,2);L_Force_Z = data(:,3);
L_Torque_X = data(:,4);L_Torque_Y = data(:,5);L_Torque_Z = data(:,6);
R_Force_X = data(:,7);R_Force_Y = data(:,8);R_Force_Z = data(:,9);
R_Torque_X = data(:,10);R_Torque_Y = data(:,11);R_Torque_Z = data(:,12);

time = 1:size(L_Force_X);
time = time * 0.001;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, L_Force_X, 'Linewidth', 2)
plot(time, R_Force_X, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 -10 10]);

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-y','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, L_Force_Y, 'Linewidth', 2)
plot(time, R_Force_Y, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 -10 10]);

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Force-z','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [N]','FontSize',14)
plot(time, L_Force_Z, 'Linewidth', 2)
plot(time, R_Force_Z, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 20 180]);

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque-x','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, L_Torque_X, 'Linewidth', 2)
plot(time, R_Torque_X, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 -2 2]);

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque-y','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, L_Torque_Y, 'Linewidth', 2)
plot(time, R_Torque_Y, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 -5 5]);

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque-z','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time, L_Torque_Z, 'Linewidth', 2)
plot(time, R_Torque_Z, 'Linewidth', 2)
legend('Left Leg', 'Right Leg')
axis([0 25 -2 2]);