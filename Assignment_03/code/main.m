%% MMVC Assignment 03
%  Obtimization

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
global inputPointsRight;
global inputPointsLeft;
inputPointsLeft = inputPoints(:,1:2);
inputPointsRight = inputPoints(:,3:4);
% figure
% imshow([inputLeftImage, inputRightImage]), title('correspondance map');
clear listInputPoints inputPoints;

% find width and height of the image for bounds
[row, col, cha] = size(inputLeftImage);
if([row, col, cha] ~= size(inputRightImage))
   disp('Error: image size does not match') 
end

% plot the correspondance
f=figure();

subplot(4,1,1)
imshow([inputRightImage inputLeftImage])
hold on
padding = repmat([col,0],[length(inputPointsLeft),1]);
paddedInputPointsLeft = inputPointsLeft + padding;
showMatch(inputPointsRight,paddedInputPointsLeft,'b')
title('input correspondences')
hold off

% find number of points
numberPts = length(inputPointsLeft);
if( numberPts ~= length(inputPointsRight))
   disp('ERROR: number of points in the right and left does not match') 
   return
end


%%
ObjLowerBound = 0;% lower bound of the nb of inliears , obtained by testing a certain model
ObjUpperBound = numberPts;% upper bound of the nb of inliers, objtained by LP

% threshold: 3 pixel
delta = 3;

%% inequality
[A,b]=constructEq(-col,-row,col,row,numberPts,delta);

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
global inlierLowerBound;
global inlierUpperBound;
global candidateLowerBound;
global candidateUpperBound;
inlierLowerBound = [];
inlierUpperBound = [];
candidateLowerBound = [];
candidateUpperBound = [];

global candidateData;
global numberNodes;
candidateData = zeros(7, 2^16-1);
numberNodes = 0;
%[feasibility; inlier lb; inlier ub; modelTx lb; modelTx ub; modelTy lb; modelTy ub; ]

global iteration;
iteration = 0;

%% 
modelBound = BranchNBound(f,A,b,lb,ub,1);
optlb = lb;
optub = ub;
optlb(1) = modelBound(1);
optlb(2) = modelBound(3);
optub(1) = modelBound(2);
optub(2) = modelBound(4);
[optA,optb] = constructEq(modelBound(1),modelBound(3),modelBound(2),modelBound(4),length(inputPointsLeft),3);
[x,opt,~]=linprog(f,optA,optb,[],[],optlb,optub);

[inlierIdx,outlierIdx] = findInlierIdx(x(3:numberPts+2));

fprintf('Translation X: %i\nTranslation Y: %i\n',x(1),x(2))
disp('Inlier index:')
disp(inlierIdx)
disp('Outlier index:')
disp(outlierIdx)
%% plot the correspondance
subplot(4,1,2)
imshow([inputRightImage inputLeftImage])
hold on
showMatch(inputPointsRight(inlierIdx,:),paddedInputPointsLeft(inlierIdx,:),'g')
showMatch(inputPointsRight(outlierIdx,:),paddedInputPointsLeft(outlierIdx,:),'r')
title('inlier and outlier correspondences')
hold off

subplot(4,1,3)
imshow([inputRightImage inputLeftImage])
hold on
showMatch(inputPointsRight(inlierIdx,:),paddedInputPointsLeft(inlierIdx,:),'g')
title('inlier correspondences')
hold off

subplot(4,1,4)
imshow([inputRightImage inputLeftImage])
hold on
showMatch(inputPointsRight(outlierIdx,:),paddedInputPointsLeft(outlierIdx,:),'r')
title('Outlier Correspondences')
hold off

%% plot lower and upper bound convergence
figure,
hold on
plot(1:length(inlierLowerBound),inlierLowerBound,'b')
plot(1:length(inlierUpperBound),inlierUpperBound,'r')
title('Convergence of Bounds')
xlabel('iteration') % x-axis label
ylabel('upper and lower bounds') % y-axis label
legend('upper bound','lower bound')
hold off

%%
figure; ax = axes;
subplot(2,2,1);
showMatchedFeatures(inputRightImage,inputLeftImage,inputPointsRight(inlierIdx,:),inputPointsLeft(inlierIdx,:),'falsecolor')
title('Inlier Point Matches');
legend('right image point','left image point');

%% 
subplot(2,2,2);
% using red for image A, green for image B, and yellow for areas of similar intensity between the two images
% imshow(imfuse(inputRightImage,imtranslate(inputLeftImage,[x(1),x(2)]),'falsecolor','Scaling','joint','ColorChannels',[1 2 0]));
imshow(imfuse(imtranslate(inputLeftImage,[x(1),x(2)]),inputRightImage,'falsecolor','Scaling','joint','ColorChannels',[1 2 0]));
title('Infused Image');

subplot(2,2,3)
imshowpair(inputRightImage,imtranslate(inputLeftImage,[x(1),x(2)]),'diff');
title('difference in pair Image');

subplot(2,2,4);
imshow([inputRightImage inputLeftImage]);
title('Original Image');