function output = computeStar(weight, source,imgsize)
%% compute point_star
% p_star = sum_for_i( w_i*p_i) / sum_for_i (w_i)

row = imgsize(1);
col = imgsize(2);
numPts = size(source,2);
temppStar = zeros([row,col,2,numPts]);
denompStar =sum(weight,3);
for idx = 1:numPts
    temppStar(:,:,1,idx) = weight(:,:,idx) .* source(1,idx);
    temppStar(:,:,2,idx) = weight(:,:,idx) .* source(2,idx);

end
numerpStar = sum(temppStar,4); % [width, height, (x,y)] 
pStar = numerpStar./repmat(denompStar,[1,1,2]); % [width, height, (x,y)]
output = pStar;

end