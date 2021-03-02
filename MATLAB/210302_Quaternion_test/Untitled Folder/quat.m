clc; close all; clear all;

%%
quater = readmatrix("tmpdata0.txt");
imu = readmatrix("tmpdata1.txt");

S = min([length(quater),length(imu)]);

quater = quater(1:S,:);
imu = imu(1:S,:);


dt = 0.001;
time = 1:1:S;
time = time*dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
% title('Joint torque','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Joint torque [Nm]','FontSize',14)
plot(time, quater,'linewidth',2)
plot(time, imu,'linewidth',2)
legend("quaternion pitch", "quaternion roll", "imu pitch", "imu roll");

%%
% quater = readmatrix("tmpdata2.txt");
% imu = readmatrix("tmpdata3.txt");
% 
% S = min([length(quater),length(imu)]);
% 
% quater = quater(1:S,:);
% imu = imu(1:S,:);
% 
% 
% dt = 0.001;
% time = 1:1:S;
% time = time*dt;
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% % title('Joint torque','FontSize',16)
% % xlabel('Time [sec]','FontSize',14)
% % ylabel('Joint torque [Nm]','FontSize',14)
% plot(time, quater,'linewidth',2)
% plot(time, imu,'linewidth',2)
% legend("quaternion pitch", "quaternion roll", "imu pitch", "imu roll");