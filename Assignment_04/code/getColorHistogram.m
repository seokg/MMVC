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

% TODO
% Seperate the channel
% R = img(:,:,1);
% G = img(:,:,2);
% B = img(:,:,3);
seedColor = img(seed(:,1), seed(:,2),:);

% numbPixel = length(seed);
r = quantizedColor(seedColor(:,:,1),histRes);
g = quantizedColor(seedColor(:,:,2),histRes);
b = quantizedColor(seedColor(:,:,3),histRes);

histR = hist(r);
histG = hist(g);
histB = hist(b);

colorHist(colorHist == 0) = power(10,-10);

end

function quantizedColor = Quantize(inputColor, histRes)
% quantize the color value such that it will have certain resolution

% len = length(inputColor);
stepSize = 256/histRes;

if mod(stepSize,1) ~= 0 
    disp('Warning reset histResolution that it will have a uniform step')
end

for i = 1:histRes
    idx = (inputColor < stepSize*i -1) && (inputColor > stepSize*(i-1));
    inputColor(idx) = stepSize * (i-1) + stepSize/2;
end

end
