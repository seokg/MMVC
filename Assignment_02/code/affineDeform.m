function output = affineDeform(image, source, target, alpha)
%% image deformation using mls affine transform 
% input
%     source: point p (control point)
%     target: point q (deformed point)
%     alpha: 
%  
% output
%     output:
%% tune the input values
[row,col,channel] = size(image);
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
% w_i = 1 / (p_i - v)^(2*alpha)
weight = computeWeight(source, target, X,Y,imgsize,alpha); % [width, height, number of control points]


%% compute p_star, q_star
% p_star = sum_for_i( w_i*p_i) / sum_for_i (w_i)
% q_star = sum_for_i( w_i*q_i) / sum_for_i (w_i)
pStar = computeStar(weight, source,imgsize);
qStar = computeStar(weight, target,imgsize);


%% compute phat, qhat
% phat = p - pstar
% qhat = q - qstar

pHat = computeHat(source,pStar,imgsize);
qHat = computeHat(target,qStar,imgsize);


%% compute A
% compute phat^T * w * phat
phattwphat = zeros(2,2,row,col,numPts);
finalwphatqhat = zeros(2,2,row,col);
for idx = 1:numPts
    
    reshapeWeight = repmat(weight(:,:,idx),[1,1,2]);

    pHatW = pHat(:,:,:,idx) .* reshapeWeight;
    pHatWpHat_xx = pHatW(:,:,1) .* pHat(:,:,1,idx);
    pHatWpHat_xy = pHatW(:,:,1) .* pHat(:,:,2,idx);
    pHatWpHat_yx = pHatW(:,:,2) .* pHat(:,:,1,idx);
    pHatWpHat_yy = pHatW(:,:,2) .* pHat(:,:,2,idx);
    for height = 1:row
        for width = 1:col
            phattwphat(1,1,height,width,idx) = pHatWpHat_xx(height,width); 
            phattwphat(1,2,height,width,idx) = pHatWpHat_xy(height,width);
            phattwphat(2,1,height,width,idx) = pHatWpHat_yx(height,width);
            phattwphat(2,2,height,width,idx) = pHatWpHat_yy(height,width);
        end
    end
end

% compute inverse of sum phat^T * w * phat
sumphattwphat = sum(phattwphat,5);
invsumphattwphat = zeros(size(sumphattwphat));
for height = 1:row
    for width = 1:col
        invsumphattwphat(:,:,height,width) = inv(sumphattwphat(:,:,height,width));
    end 
end
invsumphattwphat(invsumphattwphat==inf) = 2^64-1;
% compute v-pstar
vminuspstar = zeros(2,row,col);
vminuspstar(1,:,:) = X - pStar(:,:,1);
vminuspstar(2,:,:) = Y - pStar(:,:,2);

% compute w * phat^T * qhat
finalwphatqhat = zeros(2,2,row,col,numPts);
for idx = 1:numPts
    reshapeWeight = repmat(weight(:,:,idx),[1,1,2]);
    wpHat = pHat(:,:,:,idx) .* reshapeWeight;
    
    wHatqHat_xx = wpHat(:,:,1) .* qHat(:,:,1,idx);
    wHatqHat_xy = wpHat(:,:,1) .* qHat(:,:,2,idx);
    wHatqHat_yx = wpHat(:,:,2) .* qHat(:,:,1,idx);    
    wHatqHat_yy = wpHat(:,:,2) .* qHat(:,:,2,idx);
    for height = 1:row
        for width = 1:col
            finalwphatqhat(1,1,height,width,idx) = wHatqHat_xx(height,width); 
            finalwphatqhat(1,2,height,width,idx) = wHatqHat_xy(height,width);
            finalwphatqhat(2,1,height,width,idx) = wHatqHat_yx(height,width);
            finalwphatqhat(2,2,height,width,idx) = wHatqHat_yy(height,width);
        end
    end
end
sumwphatqhat = sum(finalwphatqhat,5);

%% f_a(v)
output = zeros(1,2,row,col);
for height = 1:row
    for width = 1:col
        output(:,:,height,width)=vminuspstar(:,height, width)'*invsumphattwphat(:,:,height,width) * sumwphatqhat(:,:,height,width) + reshape(qStar(height, width,:),[1,2]);    
    end    
end

end
