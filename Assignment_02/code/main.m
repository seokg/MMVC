clc;
clear;
close all;

% reading image
% img = imread('ginger.png');
img = imread('nupjuk.jpeg');
img = imresize(permute(img,[2,1,3]),0.1);
figure, subplot(2,2,1)
imshow(img), title('original image')
[row,col,~] = size(img);
[X,Y]=meshgrid(1:col,1:row);



% select the control points
hold on
controlpts = zeros(7,2);
for idx = 1:7
   [x,y] = ginput(1);
   controlpts(idx,:) = [x,y];
   plot(x,y,'go');
   
end

% select the target points
targetpts = zeros(7,2);
for idx = 1:7
    [x,y] = ginput(1);
    targetpts(idx,:) = [x,y];
    plot(x,y,'rx')
end
hold off

%% loading data
% clc, clear, close all;
% load('data_01.mat')

%% affine
density = 1;
alpha= 1;
coordinate = affineDeform(img, controlpts, targetpts, alpha);
affimg = makeDeformImg(coordinate,img);

subplot(2,2,2);
imshow(affimg),title('affine deformation');
hold on 
plot(controlpts(:,1),controlpts(:,2),'go')
plot(targetpts(:,1),targetpts(:,2),'rx')
%% similarity
coordinate = similarityDeform(img, controlpts, targetpts, alpha);
simimg = makeDeformImg(coordinate,img);

subplot(2,2,3);
imshow(simimg),title('similarity deformation');
hold on 
plot(controlpts(:,1),controlpts(:,2),'go')
plot(targetpts(:,1),targetpts(:,2),'rx')
%% rigid
coordinate = rigidDeform(img, controlpts, targetpts, alpha);
rigimg = makeDeformImg(coordinate,img);

subplot(2,2,4);
imshow(rigimg),title('rigid deformation');
hold on 
plot(controlpts(:,1),controlpts(:,2),'go')
plot(targetpts(:,1),targetpts(:,2),'rx')