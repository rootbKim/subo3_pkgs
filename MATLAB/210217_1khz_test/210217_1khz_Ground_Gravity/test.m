clc; clear; close all;

%%
joint_torque = readmatrix('tmpdata0.txt');
out_joint_torque = readmatrix('tmpdata1.txt');
A_L_Des_X = readmatrix('tmpdata2.txt');
A_L_Foot_Pos = readmatrix('tmpdata3.txt');
G_L_Des_X = readmatrix('tmpdata4.txt');
G_L_Foot_Pos = readmatrix('tmpdata5.txt');
O_L_Des_X = readmatrix('tmpdata6.txt');
O_L_Foot_Pos = readmatrix('tmpdata7.txt');

S = min([length(joint_torque),length(out_joint_torque),length(A_L_Des_X),length(A_L_Foot_Pos),length(G_L_Des_X),length(G_L_Foot_Pos),length(O_L_Des_X),length(O_L_Foot_Pos)]);

joint_torque = joint_torque(1:S,:);
out_joint_torque = out_joint_torque(1:S,:);
A_L_Des_X = A_L_Des_X(1:S,:);
A_L_Foot_Pos = A_L_Foot_Pos(1:S,:);
G_L_Des_X = G_L_Des_X(1:S,:);
G_L_Foot_Pos = G_L_Foot_Pos(1:S,:);
O_L_Des_X = O_L_Des_X(1:S,:);
O_L_Foot_Pos = O_L_Foot_Pos(1:S,:);

dt = 0.001;
time = 1:1:S;
time = time*dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Joint torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Joint torque [Nm]','FontSize',14)
plot(time, joint_torque,'linewidth',2)



