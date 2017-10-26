%% MMVC Assignment 03
% Global Obtimization

%% Load Data for 
clc
clear
close all

% load images
inputLeftImage = imread('../data/InputLeftImage.png');
inputRightImage = imread('../data/InputRightImage.png');
% load correspondance
listInputPoints = load('../data/ListInputPoints.mat');
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


% find number of points
numberPts = length(inputPointsLeft);
if( numberPts ~= length(inputPointsRight))
   disp('ERROR: number of points in the right and left does not match') 
   return
end

% find width and height of the image for bounds
[row, col, cha] = size(inputLeftImage);
if([row, col, cha] ~= size(inputRightImage))
   disp('Error: image size does not match') 
end
%%
ObjLowerBound = 0;% lower bound of the nb of inliears , obtained by testing a certain model
objUpperBound = numberPts;% upper bound of the nb of inliers, objtained by LP

% threshold: 3 pixel
delta = 3;

%% inequality 
% construct A
Atheta = zeros(numberPts*4, 2);
temp = [inputPointsLeft - inputPointsRight - delta, ...
       -inputPointsLeft + inputPointsRight - delta];
temp = [temp(:,1) , temp(:,3), temp(:,2), temp(:,4)];
Cell = cell(1,numberPts);
for idx = 1: numberPts
    [Cell{idx}] = deal(temp(idx,:)');
end
Az = blkdiag(Cell{:});
clear temp Cell;
Cell = cell(1,2*numberPts);
[Cell{:}] = deal([1,1]');
Aw = blkdiag(Cell{:});
clear Cell;


% bilinearity contrain
Abitheta = [0 0;1 0;0 0;-1 0;0 0;0 1;0 0;0 -1];
Abitheta = repmat(Abitheta,numberPts,1);

Abiz = [-col col -col col -row row -row row]';
Cell = cell(1,numberPts);
[Cell{:}] = deal(Abiz);
clear Abiz;
Abiz = blkdiag(Cell{:});
clear Cell;


temp = [-1 -1 1 1]';
zeroM = zeros(4,1);
Abiw = [temp zeroM; zeroM, temp];
clear temp zeroM;
Cell = cell(1,numberPts);
[Cell{:}] = deal(Abiw);
clear Abiw;
Abiw = blkdiag(Cell{:});
clear Cell;

% concat all the matrix
A = [Atheta Az Aw; Abitheta Abiz Abiw];

clear Atheta Az Aw;

% contruct b
b = zeros(numberPts*12,1);

%% upper and lower bound 
lbTheta = [-col,-row]';
lbz = zeros(numberPts,1);
lbw = [-col, -row]';
lbw = repmat(lbw,numberPts,1);

lb = [lbTheta;lbz;lbw];

ubTheta = [col,row]';
ubz = ones(numberPts,1);
ubw = [col, row]';
ubw = repmat(ubw,numberPts,1);

ub = [ubTheta;ubz;ubw];
%% objective function
fTheta = zeros(1,2);
fz = ones(1,numberPts);
fw = zeros(1,2*numberPts);
f = -[fTheta fz fw];

%%

[x,opt]=linprog(f,A,b,[],[],lb,ub);
%%
% % maximize the number of inlier
% 
% % model: (T_x,T_y)
% % T_x_lowerbound, T_x_upperbound
% % T_y_lowerbound, T_y_upperbound
% theta;
% ThetaLowerBound = [T_x_lowerbound,T_y_lowerbound];
% ThetaUpperBound = [T_x_upperbound,T_y_upperbound];
% 
% ThetaOptimizer = [];% the model obtained by LP
% 
% % threshold: 3 pixel
% delta = 3;
% 
% % cost function: f_x(model, rightpoint, leftpoint) = |x_i+T_x-x'_i|
% %                f_y(model, rightpoint, leftpoint) = |y_i+T_y-y'_i|
% % cost function < threshold
% 
% % binary variable: z_i
% % auxilary variable: w_ix = z_i*T_x, w_iy = z_i*T_y
% 
% % x = (theta, z, w)
% % x = (T_x, T_y, z_1, ... z_n, w_1x, w_1y, ... w_nx, w_ny)
% 
% 
% 
% 
%% Example 01

% objective function
% [x y z s1 s2] 
obj = [-3 2 -4];

A = [-4 -3 1;
    5 0 -1];
b = [-12; 4];
Aeq = [-4 -3 1;
        5 0 -1];
beq = [-12; 4];
ub = [3 inf 7 ];
lb = [0 4 1];
x = linprog(obj,A,b,[],[],lb,ub);

% x =
% 
%     2.2000
%     4.0000
%     7.0000

%% Example 02

