clc; clear all; close all;

Foot_Pos_G = readmatrix('Foot_Pos_G.txt');
Foot_Pos_dot_G = readmatrix('Foot_Pos_dot_G.txt');
torque_CTC_G = readmatrix('torque_CTC_G.txt');

psi = Foot_Pos_G(410,4);
theta = Foot_Pos_G(410,5);
pi = Foot_Pos_G(410,6);


L_Bmatrix = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0;
    0, 0, 0, -sin(pi), cos(pi), 0;
    0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1]

%%
Foot_Pos_A = readmatrix('Foot_Pos_A.txt');
Foot_Pos_dot_A = readmatrix('Foot_Pos_dot_A.txt');
torque_CTC_A = readmatrix('torque_CTC_A.txt');

psi = Foot_Pos_A(294,4);
theta = Foot_Pos_A(294,5);
pi = Foot_Pos_A(294,6);

L_Bmatrix = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0;
    0, 0, 0, -sin(pi), cos(pi), 0;
    0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1]