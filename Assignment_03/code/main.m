%% MMVC Assignment 03
% Global Obtimization

%% Load Data for 
clc
clear
close all

% load images
inputLeftImage = imread('..\data\InputLeftImage.png');
inputRightImage = imread('..\data\InputRightImage.png');
% load correspondance
listInputPoints = load('..\data\ListInputPoints.mat');
inputPoints = listInputPoints.ListInputPoints; % [x_i, y_i, x'_i, y'_i]
inputPointsRight = inputPoints(:,1:2);
inputPointsLeft = inputPoints(:,3:4);

% figure
% imshow([inputLeftImage, inputRightImage]), title('correspondance map');
clear listInputPoints inputPoints;

% plot the correspondance
figure; ax = axes;
showMatchedFeatures(inputRightImage,inputLeftImage, inputPointsRight, inputPointsLeft,'montage');
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

%%
% maximize the number of inlier

% model: (T_x,T_y)
% T_x_lowerbound, T_x_upperbound
% T_y_lowerbound, T_y_upperbound
theta;
ThetaLowerBound = [T_x_lowerbound,T_y_lowerbound];
ThetaUpperBound = [T_x_upperbound,T_y_upperbound];

ObjLowerBound = [];% lower bound of the nb of inliears , obtained by testing a certain model
ObjUpperBound = [];% upper bound of the nb of inliers, objtained by LP
ThetaOptimizer = [];% the model obtained by LP

% threshold: 3 pixel
delta = 3;

% cost function: f_x(model, rightpoint, leftpoint) = |x_i+T_x-x'_i|
%                f_y(model, rightpoint, leftpoint) = |y_i+T_y-y'_i|
% cost function < threshold

% binary variable: z_i
% auxilary variable: w_ix = z_i*T_x, w_iy = z_i*T_y

% x = (theta, z, w)
% x = (T_x, T_y, z_1, ... z_n, w_1x, w_1y, ... w_nx, w_ny)




%% Example 01

% objective function
% [x y z s1 s2] 
obj = [-3 2 -4 0 0];

A = [-4 -3 1 0 0;
    5 0 -1 0 0];
b = [-12; 4];
Aeq = [-4 -3 1 1 0 ;
    5 0 -1 0 1];
beq = [-12; 4];
ub = [3 inf 7 inf inf];
lb = [0 4 1 0 0];
x = linprog(obj,A,b,[],[],lb,ub);









