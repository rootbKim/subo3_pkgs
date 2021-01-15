clear; clc; close all;

L_Des_X = readmatrix('tmpdata0.txt');
err_pos = readmatrix('tmpdata1.txt');
L_X_CTC = readmatrix('tmpdata2.txt');
QDot = readmatrix('tmpdata3.txt');
L_A_Jacobian_dotXQDot = readmatrix('tmpdata4.txt');
L_q_CTC = readmatrix('tmpdata5.txt');
L_NE_Tau = readmatrix('tmpdata6.txt');
L_I_MatrixXL_q_CTC = readmatrix('tmpdata7.txt');
torque_CTC = readmatrix('tmpdata8.txt');

% L_Des_X = L_Des_X(1:9200, 1:6);
% err_pos = err_pos(1:9200, 1:6);
% L_X_CTC = L_X_CTC(1:9200, 1:6);
% QDot = QDot(1:9200, 1:6);
% L_A_Jacobian_dotXQDot = L_A_Jacobian_dotXQDot(1:9200, 1:6);
% L_q_CTC = L_q_CTC(1:9200, 1:6);
% L_NE_Tau = L_NE_Tau(1:9200, 1:6);
% L_I_MatrixXL_q_CTC = L_I_MatrixXL_q_CTC(1:9200, 1:6);
% torque_CTC = torque_CTC(1:9200, 1:6);

dt = 0.001;
time = 1:size(L_Des_X);
time = time * dt;

