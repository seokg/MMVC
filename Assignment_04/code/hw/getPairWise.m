function pairWise = getPairWise(I,lamda)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get pairwise terms for each pairs of pixels on image I and for
% regularizer lambda.
% 
% INPUT :
% - I      : color image
% - lambda : regularizer parameter
% 
% OUTPUT :
% - pairwise : sparse square matrix containing the pairwise costs for image
%              I and parameter lambda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pairWise=[];
%TODO
% B{p,q} = exp(-(Ip-Iq)^2/(2sigma^2)) * 1/dist(p,q)
sigma = 5;
[row, col]= size(I)
idxX = zeros(row*col*2,1);
idxY = zeros(row*col*2,1);
B = zeros(row*col*2,1);
for i = 1:row
    for j = 1:col 
        Iorigin = I(i,j);
        Ixright = I(i,j+1);
        Iydown = I(i+1,j);
        B1 = computeBoundary(Iorigin,Ixright);
        B2 = computeBoundary(Iorigin,Iydown);
        idxX(row*(i-1)*2+(j-1)*2+1:row*(i-1)*2+(j)*2) = [(i-1)*row+j;(i-1)*row+j+1];
        idxY(row*(i-1)*2+(j-1)*2+1:row*(i-1)*2+(j)*2) = [(i)*row+j;(i)*row+j];
        B(row*(i-1)*2+(j-1)*2+1:row*(i-1)*2+(j)*2) = [B1;B2];
    end
end
m = row*col*2;
n = row*col*2;
pairWise = sparse(idxX,idxY,B,m,n);

end

function B = computeBoundary(p,q,sigma)
    B = exp( -(p-q)^2 / (2*sigma)^2 );

end







