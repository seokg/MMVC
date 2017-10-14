function [finalRad, finalCenter, ptInlier] = funcExhaustSearch(pt, s )
%% Exhaust Search
% input
% pt: points
% s: points needed to fit model
tic
len = length(pt);
index = 1:1:len;
combination = nchoosek(index, s);
maxNumInlier = 0;
iter = length(combination);
disp(iter)
for idx = 1 : iter
    xSelect = pt(1,combination(idx,:));
    ySelect = pt(2,combination(idx,:));
    
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
        finalRad  = rad;
        finalCenter  = [xCenter, yCenter];
        
        % save the inlier points
        idxInlier = find(trueflaseMatrix == 1);
        xinlier = pt(1,idxInlier);
        yinlier = pt(2,idxInlier);
        ptInlier = [xinlier; yinlier];
    end
    
    
end
toc
fprintf('EXHAUST SEARCH number of inlier: %i\n', maxNumInlier);

end

