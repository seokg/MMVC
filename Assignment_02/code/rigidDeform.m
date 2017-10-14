function output = rigidDeform(image, source, target, alpha)
%% image deformation using mls rigid transform 
% input
%     source: point p (control point)
%     target: point q (deformed point)
%     alpha: 
%  
% output
%     output:
%% tune the input values
[row,col,~] = size(image);
[X,Y]=meshgrid(1:col,1:row);
imgsize = [row,col];
[numPts,~]=size(source);

% change source and target points as vector
% [(x,y), number of control points]
if(size(source,1)~=2)
    source = source';
    target = target';
end

%% compute weight
% size: [row,col, numControlPts]
weight = computeWeight(source, target, X,Y,imgsize,alpha); 

%% compute p_star, q_star
% size: [row,col, (x,y)]
pStar = computeStar(weight, source,imgsize);
qStar = computeStar(weight, target,imgsize);

%% compute phat, qhat
% size: [row,col,2,numControlPts]
pHat = computeHat(source,pStar,imgsize);
qHat = computeHat(target,qStar,imgsize);

%% compute A
% A_i = w_i (  phat_i             ) (  v - pstar               )^T
%           (-qhat_i^perpendicular) (-(v - pstar)^perpendicular)
A = zeros([2,2,row,col,numPts]);
for idx = 1:numPts
    for height=1:row
        for width = 1:col 
            phat_i = reshape(pHat(height,width,:,idx),[1,2]); %[1,2]
            phat_i_perp = (perpendicular(phat_i)); %[1,2]
            v = [width,height];
            pstar_temp = reshape(pStar(height,width,:),[1,2]);
            A(:,:,height,width,idx) = weight(height,width,idx)*[phat_i;-phat_i_perp] * [v - pstar_temp; -perpendicular(v -pstar_temp)].';
        end
    end
end

%% compute f_r^arrow
% f_r^arrow(v) = sum of i (qhat_i * A_i)
farrowr = zeros([1,2,row,col]);
for idx = 1:numPts
    for height=1:row
        for width = 1:col 
            farrowr(:,:,height, width) = farrowr(:,:,height, width)+reshape(qHat(height,width,:,idx),[1,2])*A(:,:,height,width,idx);
        end
    end
end


%% compute f_r(v)
% f_r(v) = |v-pstar| *  farrow_r(b)/|farrow_r(b)| + qstar
normfarrow = zeros(row,col);
for height=1:row
    for width = 1:col 
        normfarrow(height,width) = sqrt(sum(farrowr(:,:,height,width).^2));
    end
end

% compute v-pstar
vminuspstar = zeros(1,2,row,col);
vminuspstar(1,1,:,:) = X - pStar(:,:,1);
vminuspstar(1,2,:,:) = Y - pStar(:,:,2);


fr = zeros(1,2,row,col);
for height=1:row
    for width = 1:col 
        fr(:,:,height,width) = sqrt(sum(vminuspstar(:,:,height,width).^2)) ...
            * farrowr(:,:,height,width) / normfarrow(height,width) ...
            + reshape(qStar(height,width,:),[1,2]);
    end
end

output = fr;
end
