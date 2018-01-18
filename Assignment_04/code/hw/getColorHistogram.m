function colorHist = getColorHistogram(img,seed, histRes)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute a color histograms based on selected points from an image
% 
% INPUT
% - I       : color image
% - seed    : Nx2 matrix [x,y] containing the position of pixels which will be
%             uset to compute the color histogram
% - histRes : resolution of the histogram (the exercise sheet says bin of 32)
% 
% OUTPUT
% - hist : color histogram
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

colorHist = zeros(histRes,histRes,histRes); % initialize for R,G,B

%% TODO
colorData = zeros(length(stroke),3);
for i = 1:length(stroke)
    colorData(i,:) = img(seed(i,1),seed(i,2),:);
end
colorData(:,1) = Quantize(colorData(:,1), 32);
colorData(:,2) = Quantize(colorData(:,2), 32);
colorData(:,3) = Quantize(colorData(:,3), 32);

colorHist = hist3D(colorData,32);

% Normalize data
totalSum = sum(sum(sum(colorHist)));
colorHist = colorHist / totalSum;

% handle zero
colorHist(colorHist == 0) = power(10,-10);

% Smooth Hist
colorHist = smooth3(colorHist,'gaussian',7);

% take the log
colorHist = -log10(colorHist);
end
