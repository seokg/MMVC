function [inlierIdx, outlierIdx] = findInlierIdx(z)
%% find non interger value for z (binary: 0,1)
% input
%   z: binary value of 1 and 0 for inlier and outlier
% output
%   inlierIdx: index of the inlier points
%   outlierIdx: idex of the outlier points
inlierIdx = find(z > (1-10^-8));
outlierIdx = find(z < 10^-8);
end

