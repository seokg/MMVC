%% [GCT722] MMVC
% 20164332 Kwanggun Seo
% Exercise 1- Robust Estimation
%
%

%% EXERCISE PART 1: RANSAC FOR CIRCLE FITTING
%% Generating Data
center = [0,0];
xc = center(1);
yc = center(2);
rad = 3;
ratio = 0.5;
N = 100;

[inlier,outlier] = genData(N,center ,rad ,ratio);

figure, plot(inlier(1,:),inlier(2,:),'bo',outlier(1,:),outlier(2,:),'ro')
axis([-10 10 -10 10])

%% RANSAC
pt = [inlier, outlier];
p = 0.99;
ep = 1-ratio;
s = 3;
[ finalRad, finalCenter ] = funcRANSAC(pt, p, ep, s);

% generate circle
theta = 0:2*pi/360:2*pi;
xFit = finalRad * cos(theta); 
yFit = finalRad * sin(theta);

hold on
plot(xFit,yFit,'g-')

%% Exhaust Search
[rad, center ] = funcExhaustSearch(pt, s);


%% EXERCISE PART 2: IRLS and norms for line fitting