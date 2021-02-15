clc; clear all; close all;

Foot_Pos = readmatrix('Foot_Pos_G.txt');
Foot_Pos_dot = readmatrix('Foot_Pos_dot_G.txt');

psi = Foot_Pos(410,4);
theta = Foot_Pos(410,5);
pi = Foot_Pos(410,6);


L_Bmatrix = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0;
    0, 0, 0, -sin(pi), cos(pi), 0;
    0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1]