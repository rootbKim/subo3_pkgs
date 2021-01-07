clc; clear; close all;

data = readmatrix("GdData2.txt");

data = data(1000:end,:);


plot(data(:,3))
hold on;
axis([0 100 0 500]);

% plot(data(:,9))

% force_Z = data(:,3) + data(:,9);
% 
% plot(force_Z)