function [inlier,outlier] = genData(N,center ,rad ,ratio )
%% generating data for circles with noise and outliers
%
% (x-x_c)^2 + (y-y_c)^2 = R^2
% distance between the data points and the circle is defined as the
% geometrical distance (Euclidean distance)
%
% data points on a circle in the domain [-10,10] x [-10,10]
% random noise is added between -0.1 and 0.1 
%
% ratio r of outlier
% 5, 20, 30, and 70%
%
% Define Tao = 0.1 as the inlier distance threshold

%% input
% N: number of points (default:100)
% center: centor of circle (default: [0,0])
% rad: radius of circle (default: 3)
% ratio: ratio of outlier

%% output
% output : generated points

%% 
xc = center(1);
yc = center(2);

% random seed for rand function
rng('shuffle','twister');

% number of outlier
numOutlier = N*ratio;
numInlier =  round(N *(1.0-ratio));

% generate circle
theta = rand([1, numInlier])* 2 * pi;
xTemp = rad * cos(theta); 
yTemp = rad * sin(theta);

xInlier = xTemp + xc;
yInlier = yTemp + yc;

% adding noise
theta = rand([1, numInlier])* 2 * pi;
noiseScale = rand([1, numInlier]) * 0.1;
xNosie = noiseScale .* cos(theta);
yNoise = noiseScale .* sin(theta);

xInlier = xInlier + xNosie;
yInlier = yInlier + yNoise;

% generate ouliers
[xOutlier,yOutlier] = genOutlier(numOutlier, center ,rad );

inlier = [xInlier ; yInlier ];
outlier = [xOutlier; yOutlier];

end

function [xOutlier,yOutlier] = genOutlier(numOutlier, center ,rad )
% initialization
numFalseOutlier = 1000000;
xOutlier = [];
yOutlier = [];

xc = center(1);
yc = center(2);
iter = 0;
rng('shuffle','twister');
while numFalseOutlier ~= 0
    % generate ouliers
    xOutlier = [xOutlier, rand([1, numOutlier])* 20 - 10];
    yOutlier = [yOutlier, rand([1, numOutlier])* 20 - 10];
    % check wheter it is truely outliers
    trueflaseMatrix = ((xOutlier - xc).^2 + (yOutlier -yc).^2 > (rad - 0.1).^2) & ...
                      ((xOutlier - xc).^2 + (yOutlier -yc).^2 < (rad + 0.1).^2) ;  
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
