function unaries = getUnaries(img,lambda,hist_fg,hist_bg, seed_fg, seed_bg)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get Unaries for all pixels in inputImg, using the foreground and
% background color histograms, and enforcing hard constraints on pixels
% marked by the user as foreground and background
% 
% INPUT :
% - I       : color image
% - hist_fg : foreground color histogram
% - hist_bg : background color histogram
% - seed_fg : pixels marked as foreground by the user
% - seed_bg : pixels marked as background by the user
% 
% OUTPUT :
% - unaries : Nx2 (FG cost and BG cost) matrix containing the unary cost for every pixels in I
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO

% Extract indicies
[row,col] = size(img);
[X,Y] = meshgrid(1:col,1:row);
numData = row*col;

unaries= zeros(2,numData);
% Get color value and quantize
colorData = zeros([numData,3]);
for i =1:numData
    colorData(i,:) = img(X(i),Y(i),:);
end
colorData(:,1) = Quantize(colorData(:,1), 32);
colorData(:,2) = Quantize(colorData(:,2), 32);
colorData(:,3) = Quantize(colorData(:,3), 32);

% Get prob from the hist data

idx_bg = coordinate2index(seed_bg,image);
idx_fg = coordinate2index(seed_fg,image);
for i = 1:numData
    % forground
    if nnz(i == idx_fg) > 0% seed
       unaries(1,i) = inf;
    else
    unaries(1,i) = hist_fg(colorData(i,1), colorData(i,2), colorData(i,3));
        
    end
    % background
    if nnz(i == idx_bg) > 0
        unaries(2,i) = inf;
    else
    unaries(2,i) = hist_bg(colorData(i,1), colorData(i,2), colorData(i,3));
        
    end
end

end

function idx = coordinate2index(x,image)
%% change x y coordinate to index
% input 
%     coordiate (x,y)
%     image
% output index

%%
len = length(x);
[row,~] = size(image);

idx = zeros(len,1);
for i = 1:len
   idx(i) = x(i,1) + (x(i,2) -1) * row;
end

end








