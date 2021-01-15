clc; clear; close all;

r11 = 0.961791;
r12 = 0.0266738;
r13 = -0.272481;
r21 = -0.026376;
r22 = 0.999641;
r23 = 0.00475656;
r31 = 0.27251;
r32 = 0.00261212;
r33 = 0.962149;

R = [r11, r12, r13; r21, r22, r23; r31, r32, r33];

%% roll, pitch, yaw
pi_1 = atan2(r21, r11)
th_1 = atan2(-r31, cos(pi_1)*r11+sin(pi_1)*r21)
csi_1 = atan2(sin(pi_1)*r13-cos(pi_1)*r23, -sin(pi_1)*r12+cos(pi_1)*r22)

pi_2 = atan2(-r21, -r11)
th_2 = atan2(-r31, cos(pi_2)*r11+sin(pi_2)*r21)
csi_2 = atan2(sin(pi_2)*r13-cos(pi_2)*r23, -sin(pi_2)*r12+cos(pi_2)*r22)

%% Euler angle(z, y, z)

th_1 = atan2(sqrt(1-r33^2),r33)
pi_1 = atan2(r23,r13)
csi_1 = atan2(r32,-r31)

th_2 = atan2(-sqrt(1-r33^2),r33)
pi_2 = atan2(-r23,-r13)
csi_2 = atan2(-r32,r31)