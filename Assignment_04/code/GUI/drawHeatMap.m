function drawHeatMap(data)
[~,~,N] = size(data);    
figure
    for n=1:N
        imagesc(data(:,:,n));
        colormap('copper');
        colorbar;
        pause(0.5);
    end
end