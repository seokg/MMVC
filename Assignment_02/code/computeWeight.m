function output = computeWeight(source, target, X, Y,imgsize ,alpha)

%% compute weight
% w_i = 1 / (p_i - v)^(2*alpha)
row=imgsize(1);
col=imgsize(2);
numPts = size(source,2);
weight = zeros(row, col, numPts); % [width, height, number of control points]

% [width, height, x, ]
for idx = 1:numPts
    denom_x = source(1,idx) - X;
    denom_y = source(2,idx) - Y;
    weight(:,:,idx) = 1./ sqrt(denom_x.^2+denom_y.^2).^(2*alpha);
end
weight(weight==inf) = 2^64-1;
% check the weight value
% surf(X,Y,sum(weight(:,:,:),3))

output = weight; 
end