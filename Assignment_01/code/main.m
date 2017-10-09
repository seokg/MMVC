%% [GCT722] MMVC
% 20164332 Kwanggun Seo
% Exercise 1- Robust Estimation
%
%
clc;
clear;
close all;

%% EXERCISE PART 1: RANSAC FOR CIRCLE FITTING

center = [1,-1];
xc = center(1);
yc = center(2);
rad = 3;
N = 100;


% 5%, 20%, 30%, 70% 
ratio005 = 0.05;
ratio020 = 0.20;
ratio030 = 0.30;
ratio070 = 0.70;

figure

%% outlier ratio 0.05
% Generating Data
[inlier,outlier] = genCircleData(N,center ,rad ,ratio005);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio005;
s = 3;
disp('number of iteration')
disp( log(1-p) / log(1-(1-ep)^s));
disp('time to compute RANSAC')
[ ransacRad, ransacCenter, ransacptInlier, ransacptOutlier ] = funcRANSAC(pt, p, ep, s);

% Exhaust Search
disp('time to compute Exhaust Search')
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);

% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;

for i = 1:1000
    [ Rad, Center, in,out ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(in);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
%         ransacRad = Rad;
%         ransacCenter = Center;
%         ransacptInlier = in;
%         ransacptOutlier = out;
    end

end

subplot(2,4,1);
hist(histData,0:100)
xlabel('Nb of detected inliers') % x-axis label
ylabel('Nb of experiments') % y-axis label


% draw points and circle
theta = 0:2*pi/360:2*pi;
xFit = ransacRad * cos(theta) + ransacCenter(1); 
yFit = ransacRad * sin(theta) + ransacCenter(2);

xtruth = rad * cos(theta) + center(1); 
ytruth = rad * sin(theta) + center(2);

subplot(2,4,5);
plot(ransacptInlier(1,:),ransacptInlier(2,:),'bo',ransacptOutlier(1,:),ransacptOutlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 5%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.20
% Generating Data
[inlier,outlier] = genCircleData(N,center ,rad ,ratio020);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio020;
s = 3;
disp('number of iteration')
disp( log(1-p) / log(1-(1-ep)^s));
disp('time to compute RANSAC')
[ ransacRad, ransacCenter, ransacptInlier, ransacptOutlier  ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
disp('time to compute Exhaust Search')
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);
% generate histrogram
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, in,out ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(in);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
%         ransacRad = Rad;
%         ransacCenter = Center;
%         ransacptInlier = in;
%         ransacptOutlier = out;
    end

end
subplot(2,4,2);
hist(histData,0:100)
xlabel('Nb of detected inliers') % x-axis label
ylabel('Nb of experiments') % y-axis label


% draw points and circle
theta = 0:2*pi/360:2*pi;
xFit = ransacRad * cos(theta) + ransacCenter(1); 
yFit = ransacRad * sin(theta) + ransacCenter(2);

xtruth = rad * cos(theta) + center(1); 
ytruth = rad * sin(theta) + center(2);

subplot(2,4,6);
plot(ransacptInlier(1,:),ransacptInlier(2,:),'bo',ransacptOutlier(1,:),ransacptOutlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 20%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.30
% Generating Data
[inlier,outlier] = genCircleData(N,center ,rad ,ratio030);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio030;
s = 3;
disp('number of iteration')
disp( log(1-p) / log(1-(1-ep)^s));
disp('time to compute RANSAC')
[ ransacRad, ransacCenter, ransacptInlier, ransacptOutlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
disp('time to compute Exhaust Search')
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);
% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, in,out ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(in);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
%         ransacRad = Rad;
%         ransacCenter = Center;
%         ransacptInlier = in;
%         ransacptOutlier = out;
    end

end
subplot(2,4,3);
hist(histData,0:100)
xlabel('Nb of detected inliers') % x-axis label
ylabel('Nb of experiments') % y-axis label


% draw points and circle
theta = 0:2*pi/360:2*pi;
xFit = ransacRad * cos(theta) + ransacCenter(1); 
yFit = ransacRad * sin(theta) + ransacCenter(2);

xtruth = rad * cos(theta) + center(1); 
ytruth = rad * sin(theta) + center(2);

subplot(2,4,7);
plot(ransacptInlier(1,:),ransacptInlier(2,:),'bo',ransacptOutlier(1,:),ransacptOutlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 30%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.70
% Generating Data
[inlier,outlier] = genCircleData(N,center ,rad ,ratio070);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio070;
s = 3;
disp('number of iteration')
disp( log(1-p) / log(1-(1-ep)^s));
disp('time to compute RANSAC')
[ ransacRad, ransacCenter, ransacptInlier, ransacptOutlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
disp('time to compute Exhaust Search')
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);
% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, in,out ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(in);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
%         ransacRad = Rad;
%         ransacCenter = Center;
%         ransacptInlier = in;
%         ransacptOutlier = out;
    end

end
subplot(2,4,4);
hist(histData,0:100)
xlabel('Nb of detected inliers') % x-axis label
ylabel('Nb of experiments') % y-axis label


% draw points and circle
theta = 0:2*pi/360:2*pi;
xFit = ransacRad * cos(theta) + ransacCenter(1); 
yFit = ransacRad * sin(theta) + ransacCenter(2);

xtruth = rad * cos(theta) + center(1); 
ytruth = rad * sin(theta) + center(2);

subplot(2,4,8);
plot(ransacptInlier(1,:),ransacptInlier(2,:),'bo',ransacptOutlier(1,:),ransacptOutlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 70%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')





%% EXERCISE PART 2: IRLS and norms for line fitting
clear;
figure;

N = 100;
ratio000 = 0.00;
ratio010 = 0.10;

slope = 0.65;
b = -1;

%%
[inlier,outlier] = genLineData(N, slope, b, ratio010);

% draw line
step = 20/ 1000;
x= -10:step:10;
y = slope * x + b;


subplot(1,2,1);
plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])
hold on
plot(x,y,'g-');

xinlier = inlier(1,:);
yinlier = inlier(2,:);
xoutlier = outlier(1,:);
youtlier = outlier(2,:);

xdata = [xinlier.';xoutlier.'];
ydata = [yinlier.';youtlier.'];

% IRLS
d = 0.0001;
tolerance = 0.1;
winit = 1;
iter=20;
theta = funcIRLS(ydata, xdata, iter, winit, d, tolerance);


% draw line
y = theta(1) * x +theta(2);
plot(x,y,'b-');


% L infinity norm
f = [0 0 1];
[theta,error] = funcLInfinity(xdata, ydata, f);
y = theta(1) * x +theta(2);
plot(x,y,'r-');

% L1 norm
f = [0 0 1];
[theta,error] = funcLOne(xdata, ydata);
y = theta(1) * x +theta(2);
plot(x,y,'k-');
legend('data inlier','data outlier' , 'synth. model', 'IRLS L-1', 'LP L-infinity model', 'LP L-1 model','southoutside')
%%

[inlier,outlier] = genLineData(N, slope, b, ratio000);

% draw line
step = 20/ 1000;
x= -10:step:10;
y = slope * x + b;


subplot(1,2,2);
plot(inlier(1,:),inlier(2,:),'bo')
hold on
axis([-10 10 -10 10])
plot(x,y,'g-');




xinlier = inlier(1,:);
yinlier = inlier(2,:);
xoutlier = outlier(1,:);
youtlier = outlier(2,:);

xdata = [xinlier.';xoutlier.'];
ydata = [yinlier.';youtlier.'];

% IRLS
d = 0.0001;
tolerance = 0.1;
winit = 1;
iter=20;
theta = funcIRLS(ydata, xdata, iter, winit, d, tolerance);


% draw line
y = theta(1) * x +theta(2);
plot(x,y,'b-');


% L infinity norm
f = [0 0 1];
[theta,error] = funcLInfinity(xdata, ydata, f);
y = theta(1) * x +theta(2);
plot(x,y,'r-');

% L1 norm
f = [0 0 1];
[theta,error] = funcLOne(xdata, ydata);
y = theta(1) * x +theta(2);
plot(x,y,'k-');
legend('data inlier', 'synth. model', 'IRLS L-1', 'LP L-infinity model', 'LP L-1 model','southoutside')