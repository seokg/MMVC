function output = makeDeformImg(coordinate,img)
%% make deformed image
% input
%   coordinate: coordinate of the new points
%   img: original image
% output 
%   output: new deformed image
[row,col,~]=size(img);
[X,Y]=meshgrid(1:col,1:row);
x = reshape(coordinate(1,1,:,:),[row,col]);
y = reshape(coordinate(1,2,:,:),[row,col]);

red = img(:,:,1);
green = img(:,:,2);
blue = img(:,:,3);
affred   = uint8(griddata(x,y,double(red)  ,X,Y,'cubic'));
affgreen = uint8(griddata(x,y,double(green),X,Y,'cubic'));
affblue  = uint8(griddata(x,y,double(blue) ,X,Y,'cubic'));

affimg = uint8(zeros(row,col,3));
affimg(:,:,1) = affred;
affimg(:,:,2) = affgreen;
affimg(:,:,3) = affblue;

output = affimg;
end