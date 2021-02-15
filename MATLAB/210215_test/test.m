clc; clear; close all;
%%
Des_X_A = readmatrix('Des_X_A.txt');
Foot_Pos_A = readmatrix('Foot_Pos_A.txt');
Foot_Pos_dot_A = readmatrix('Foot_Pos_dot_A.txt');
joint_torque_A = readmatrix('joint_torque_A.txt');
out_joint_torque_A = readmatrix('out_joint_torque_A.txt');

Des_X_A = Des_X_A(1:650,:);
Foot_Pos_A = Foot_Pos_A(1:650,:);
Foot_Pos_dot_A = Foot_Pos_dot_A(1:650,:);
joint_torque_A = joint_torque_A(1:650,:);
out_joint_torque_A = out_joint_torque_A(1:650,:);

X_CTC_A = readmatrix('X_CTC_A.txt');
q_CTC_A = readmatrix('q_CTC_A.txt');
tmp_A = readmatrix('tmp_A.txt');
NE_Tau_A = readmatrix('NE_Tau_A.txt');
torque_CTC_A = readmatrix('torque_CTC_A.txt');

X_CTC_A = X_CTC_A(1:350,:);
q_CTC_A = q_CTC_A(1:350,:);
tmp_A = tmp_A(1:350,:);
NE_Tau_A = NE_Tau_A(1:350,:);
torque_CTC_A = torque_CTC_A(1:350,:);

index = 0;
for i=1:1:350
    if(rem(i,2) == 1)
        index = index+1;
        X_CTC__A(index,:) = X_CTC_A(i,:);
        q_CTC__A(index,:) = q_CTC_A(i,:);
        tmp__A(index,:) = tmp_A(i,:);
        NE_Tau__A(index,:) = NE_Tau_A(i,:);
        torque_CTC__A(index,:) = torque_CTC_A(i,:);
    end
end

%%
Des_X_G = readmatrix('Des_X_G.txt');
Foot_Pos_G = readmatrix('Foot_Pos_G.txt');
Foot_Pos_dot_G = readmatrix('Foot_Pos_dot_G.txt');
joint_torque_G = readmatrix('joint_torque_G.txt');
out_joint_torque_G = readmatrix('out_joint_torque_G.txt');

Des_X_G = Des_X_G(1:1300,:);
Foot_Pos_G = Foot_Pos_G(1:1300,:);
Foot_Pos_dot_G = Foot_Pos_dot_G(1:1300,:);
joint_torque_G = joint_torque_G(1:1300,:);
out_joint_torque_G = out_joint_torque_G(1:1300,:);

X_CTC_G = readmatrix('X_CTC_G.txt');
q_CTC_G = readmatrix('q_CTC_G.txt');
tmp_G = readmatrix('tmp_G.txt');
NE_Tau_G = readmatrix('NE_Tau_G.txt');
torque_CTC_G = readmatrix('torque_CTC_G.txt');

X_CTC_G = X_CTC_G(1:650,:);
q_CTC_G = q_CTC_G(1:650,:);
tmp_G = tmp_G(1:650,:);
NE_Tau_G = NE_Tau_G(1:650,:);
torque_CTC_G = torque_CTC_G(1:650,:);

index = 0;
for i=1:1:650
    if(rem(i,2) == 1)
        index = index+1;
        X_CTC__G(index,:) = X_CTC_G(i,:);
        q_CTC__G(index,:) = q_CTC_G(i,:);
        tmp__G(index,:) = tmp_G(i,:);
        NE_Tau__G(index,:) = NE_Tau_G(i,:);
        torque_CTC__G(index,:) = torque_CTC_G(i,:);
    end
end



