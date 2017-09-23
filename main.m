%% [GCT722] MMVC
% 20164332 Kwanggun Seo
% Exercise 1- Robust Estimation
%
%
clc;
clear;

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
[inlier,outlier] = genData(N,center ,rad ,ratio005);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio005;
s = 3;
[ ransacRad, ransacCenter, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);

% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(ransacptInlier);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
        ransacRad = Rad;
        ransacCenter = Center;
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
plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 5%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.20
% Generating Data
[inlier,outlier] = genData(N,center ,rad ,ratio020);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio020;
s = 3;
[ ransacRad, ransacCenter, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);

% generate histrogram
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(ransacptInlier);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
        ransacRad = Rad;
        ransacCenter = Center;
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
plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 20%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.30
% Generating Data
[inlier,outlier] = genData(N,center ,rad ,ratio030);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio030;
s = 3;
[ ransacRad, ransacCenter, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);

% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(ransacptInlier);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
        ransacRad = Rad;
        ransacCenter = Center;
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
plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 30%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')

%% outlier ratio 0.70
% Generating Data
[inlier,outlier] = genData(N,center ,rad ,ratio070);

% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = ratio070;
s = 3;
[ ransacRad, ransacCenter, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
% Exhaust Search
[exhaustRad, exhaustCenter, exhaustInlier ] = funcExhaustSearch(pt, s);

% generate histrogram
histData = zeros(1,1000);
maxNumInlier = 0;
for i = 1:1000
    [ Rad, Center, ransacptInlier ] = funcRANSAC(pt, p, ep, s);
    numInlier = length(ransacptInlier);
    histData(i) = numInlier;
    if maxNumInlier < numInlier
        maxNumInlier = numInlier;
        ransacRad = Rad;
        ransacCenter = Center;
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
plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])

hold on
plot(xFit,yFit,'k-')
hold on
plot(xtruth,ytruth,'g-')
title('RANSAC with outlier ratio 70%')
legend('RANSAC inlier','RANSAC outlier' , 'RANSAC model', 'synth. model', 'Location', 'southoutside')





%% EXERCISE PART 2: IRLS and norms for line fitting