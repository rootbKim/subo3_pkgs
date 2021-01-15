clc; clear; close all;

R = [   0.999997  0.00155965 -0.00169007
-0.00155718    0.999998  0.00146219
 0.00169235 -0.00145955    0.999998]; % from GAZEBO

pi = atan2(R(2,1),R(1,1))
theta = atan2(-R(3,1),cos(pi)*R(1,1)+sin(pi)*R(2,1))
psi = atan2(sin(pi)*R(1,3)-cos(pi)*R(2,3), -sin(pi)*R(1,2)+cos(pi)*R(2,2))

Bmatrix = [cos(pi)/cos(theta), sin(pi)/cos(theta), 0;
    -sin(pi), cos(pi), 0;
    cos(pi)*tan(theta), sin(pi)*tan(theta), 1]

L_Bmatrix = [eye(3), zeros(3);
    zeros(3), Bmatrix]

G_Jacobian = [ 0.000478676  0.000848422    -0.544411    -0.327309      -0.1105  0.000172341
-0.000574959     0.544411  0.000848661  0.000601255  0.000172068       0.1105
           0 -0.000479571  0.000324213     0.124286 -0.000187004  -0.00016128
           0     0.999999   0.00155842   0.00155842   0.00155842     0.999997
           0  -0.00155842     0.999999     0.999999     0.999999  -0.00155718
           1            0 -0.000733538 -0.000733538 -0.000733538   0.00169235] % from GAZEBO

A_Jacobian = L_Bmatrix * G_Jacobian

Inv_A_Jacobian = inv(A_Jacobian)











