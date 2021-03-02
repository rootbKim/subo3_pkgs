clc; close all; clear all;

%%
f = readmatrix("tmpdata0.txt");
t = readmatrix("tmpdata1.txt");

S = min([length(f),length(t)]);

f = f(1:S,:);
t = t(1:S,:);


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
plot(time, f,'linewidth',2)
plot(time, t,'linewidth',2)
% legend("quaternion pitch", "quaternion roll", "imu pitch", "imu roll");
