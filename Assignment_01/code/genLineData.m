function [inlier,outlier] = genLineData(N, slope, b, ratio)
    
%% generating data for line with noise and outliers
%
% y = a x + b
% distance between the data points and the circle is defined as the
% geometrical distance (Euclidean distance)
%
% data points on a circle in the domain [-10,10] x [-10,10]
% random noise is added between -0.1 and 0.1 
%
% ratio r of outlier
% 0 and 10%
%
% Define Tao = 0.1 as the inlier distance threshold

%% input
% N: number of points (default:100)
% ratio: ratio of outlier

%% output
% output : generated points

%%

% random seed for rand function
rng('shuffle','twister');

% number of outlier
numOutlier = N*ratio;
numInlier =  round(N *(1.0-ratio));

% generate line data
minX = -10;
maxX = 10;
step = 20 / numInlier;
xInlier = [minX:step:maxX-0.00001];

yInlier = slope * xInlier + b;

% adding noise
% theta = rand([1, numInlier])* 2 * pi;
% noiseScale = rand([1, numInlier]) * 0.1;
% xNosie = noiseScale .* cos(theta);
% yNoise = noiseScale .* sin(theta);

% xInlier = xInlier + xNosie;
% yInlier = yInlier + yNoise;
yNosie =(rand([1, numInlier])-0.5)*0.1;
yInlier = yInlier + yNosie;

% generate ouliers
[xOutlier,yOutlier] = genOutlier(numOutlier, slope ,b );

inlier = [xInlier ; yInlier ];
outlier = [xOutlier; yOutlier];
end

function [xOutlier,yOutlier] = genOutlier(numOutlier, slope ,b )
% initialization
numFalseOutlier = 1000000;
xOutlier = [];
yOutlier = [];

iter = 0;
rng('shuffle','twister');
while numFalseOutlier ~= 0
    % generate ouliers
    xOutlier = [xOutlier, rand([1, numOutlier])* 20 - 10];
    yOutlier = [yOutlier, rand([1, numOutlier])* 20 - 10];
    
    % check wheter it is truely outliers
    dist = abs(xOutlier * slope + b -yOutlier);
%     dist = abs(slope*xOutlier - yOutlier + b) / sqrt(slope^2+1);
    trueflaseMatrix = dist < 0.1;  
    numFalseOutlier = nnz(trueflaseMatrix);
    if (numFalseOutlier>0)
        disp('discard false outliers')
        iter = iter+1;
    end
    
    % removing outliers
    xOutlier(find(trueflaseMatrix))=[];
    yOutlier(find(trueflaseMatrix))=[];
    
    numOutlier = numOutlier - numFalseOutlier;              
end
 disp(iter)

end

