function output = computeHat(source,star,imgsize)
%% compute point_hat
% phat = p - pstar
numPts = size(source,2);
row=imgsize(1);
col=imgsize(2);
pHat = zeros([row,col,2,numPts]);
for idx = 1:numPts
    pHat(:,:,:,idx) = repmat(reshape(source(:,idx),[1,1,2]),[row,col]) - star;
end
output = pHat;

end