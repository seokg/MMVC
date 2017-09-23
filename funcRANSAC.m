function [ finalRad, finalCenter ] = funcRANSAC(pt, p, ep, s)

p = 0.99;
ep = 0.7;
s = 3;
% calc number of iternation
N = log(1-p) / log(1-(1-ep)^s);

% sample data randomly
% estimate parameters using sampled data
% calculate error of Data
% count # of inlier candidates
% maximum number of inlier candidates until now?
% keep the paramters as the final estimation
% enough number of iteration?
% return the final estimation

rng('shuffle','twister');
numPts = length(pt);
maxNumInlier = 0;
for iter = 1:N
    
    % sample data randomly
    index = ceil(rand([1,s])*numPts);
%     disp(index)
    xSelect = pt(1,index); ySelect = pt(2,index);

    % estimate parameters using sampled data
    x1= xSelect(1);
    x2= xSelect(2);
    x3= xSelect(3);
    y1= ySelect(1);
    y2= ySelect(2);
    y3= ySelect(3);
    ma = (y2 - y1) / (x2 - x1);
    mb = (y3 - y2) / (x3 - x2);
    
    xCenter = (ma* mb * (y1 - y3) + mb * (x1 + x2) - ma * ( x2 + x3)) / (2 * mb - ma);
    yCenter = -1/ma * (xCenter - (x1 + x2) / 2) + (y1 + y2) / 2;
    
    rad = sqrt( (xCenter - x1).^2 + (yCenter - y1).^2 );
    
    % calculate error of Data
    trueflaseMatrix = (((pt(1,:) - xCenter).^2 + (pt(2,:) -yCenter).^2) > (rad - 0.1).^2) & ...
                      (((pt(1,:) - xCenter).^2 + (pt(2,:) -yCenter).^2) < (rad + 0.1).^2); 
    % count # of inlier candidates
    numInlier = nnz(trueflaseMatrix);     
    
    % maximum number of inlier candidates until now?
    if maxNumInlier < numInlier 
        maxNumInlier = numInlier;
        
        % keep the paramters as the final estimation
        finalRad = rad;
        finalCenter = [xCenter, yCenter];
    end
end

fprintf('RANSAC number of inlier: %i\n', maxNumInlier);
end

