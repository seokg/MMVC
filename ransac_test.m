% GCT722 
% 20164332 KWANGGUN SEO
% 2017 Sept 15 
% RANSAC data generation for line and circle

%% Line point
clc
clear
% generate points
slope = 1.5;
b = 10;
x = 0:0.5:100;
y = slope * x + b;

% generate noise
noise = randi(100,1,201)/100-0.5;
y = y + noise;


% generate outliers
maxX= floor(max(x));
maxY= floor(max(y));

numOutlier = 30;
outlierX = randi(maxX, 1, numOutlier);
outlierY = randi(maxY, 1, numOutlier);

figure, plot(x,y,'o',outlierX,outlierY,'*')


%% Circle point
clc
clear

% generate points
centerX = 10;
centerY = 20;
radius = 5;

step = 2*pi/360;
phase = 0:step:2*pi;

x = radius*cos(phase) + centerX;
y = radius*sin(phase) + centerY;

% generate noise
noise = normrnd(0,1,[1,length(y)]);
y = y+noise;

% generate outliers
maxX= floor(max(x));
maxY= floor(max(y));

numOutlier = 30;
outlierX = randi(maxX*2, 1, numOutlier);
outlierY = randi(maxY*2, 1, numOutlier);

figure, plot(x,y,'o',outlierX,outlierY,'*')


