function output = hist3D(data,bin)
%% make a 3d histogram for color
% input
%   data: color value
%   bin: resolution
% output
%   3d heatmap of the histogram

%%
stepSize = 256/bin;
len=length(data);
output = zeros(bin,bin,bin);
for i = 1:bin
    for j = 1:bin
        for k = 1:bin
             idx = data == repmat([stepSize * (i-1) + stepSize/2,...
                            stepSize * (j-1) + stepSize/2,...
                            stepSize * (k-1) + stepSize/2], [len,1]);
             output(i,j,k) = nnz(idx(:,1)&...
                                 idx(:,2)&...
                                 idx(:,3) );    
        end
        
    end
    
end


end