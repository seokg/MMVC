function pairWise = getPairWise(I)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get pairwise terms for each pairs of pixels on image I and for
% regularizer lambda.
% 
% INPUT :
% - I      : color image
% 
% OUTPUT :
% - pairwise : sparse square matrix containing the pairwise costs for image
%              I and parameter lambda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TODO
% B{p,q} = exp(-(Ip-Iq)^2/(2sigma^2)) * 1/dist(p,q)
% compare with 8 neighboring pixels

sigma = 5;
[row, col,~]= size(I);
nodeP = zeros((row)*(col),1);
nodeQ = zeros((row)*(col),1);
B = zeros((row)*(col),1);
iter = 1; 

for j =1:col

    for i =1:row

        % initialize
        downflag = false;
        rightflag = false;
        diagDownflag = false;      
        diagUpflag = false;        
        
%         upflag = false;
%         leftflag = false;
%         diagDownLeftflag = false;      
%         diagUpLeftflag = false;  
        
        
        Iorigin = I(i,j,:);
        % down pixel
        if i ~= row
            Iydown = I(i+1,j,:);
            downflag = true;
        end
        % right pixel
        if j ~= col
            Ixright = I(i,j+1,:);
            rightflag = true;
        end
        % diagonal down pixel right
        if j ~= col && i ~= row 
            Idiagdown = I(i+1,j+1,:);
            diagDownflag = true;        
        end
        % diagonal up pixel right
        if j ~= col && i ~= 1 
            Idiagup = I(i-1,j+1,:);
            diagUpflag = true;        
        end
        
        if downflag
            B(iter) = computeBoundary(Iorigin,Iydown,sigma) / computeDist([i+1,j],[i,j]);
            nodeP(iter) = (j-1)*row + i;
            nodeQ(iter) = (j-1)*row + i+1;
            iter = iter + 1;
        end
        if rightflag
            B(iter) = computeBoundary(Iorigin,Ixright,sigma) / computeDist([i,j+1],[i,j]);
            nodeP(iter) = (j-1)*row + i;
            nodeQ(iter) = (j)*row + i;
            iter = iter + 1;
        end
        if diagDownflag
            B(iter) = computeBoundary(Iorigin,Idiagdown,sigma) / computeDist([i+1,j+1],[i,j]);
            nodeP(iter) = (j-1)*row + i;
            nodeQ(iter) = (j)*row + i+1;
            iter = iter + 1;
        end
        if diagUpflag
            B(iter) = computeBoundary(Iorigin,Idiagup,sigma) / computeDist([i-1,j+1],[i,j]);
            nodeP(iter) = (j-1)*row + i;
            nodeQ(iter) = (j)*row + i-1;
            iter = iter + 1;
        end       
        
%%        
%         % up pixel
%         if i ~= 1
%             Iyup = I(i-1,j,:);
%             upflag = true;
%         end
%         % left pixel
%         if j ~= 1
%             Ixleft = I(i,j-1,:);
%             leftflag = true;
%         end
%         % diagonal down pixel left
%         if j ~= 1 && i ~= row 
%             Idiagdownleft = I(i+1,j-1,:);
%             diagDownLeftflag = true;        
%         end        
%         % diagonal up pixel left
%         if j ~= 1 && i ~= 1 
%             Idiagupleft = I(i-1,j-1,:);
%             diagUpLeftflag = true;        
%         end          
%%        
%         if upflag
%             B(iter) = computeBoundary(Iorigin,Iyup,sigma) / computeDist([i-1,j],[i,j]);
%             nodeP(iter) = (j-1)*row + i;
%             nodeQ(iter) = (j-1)*row + i-1;
%             iter = iter + 1;
%         end
%         if leftflag
%             B(iter) = computeBoundary(Iorigin,Ixleft,sigma) / computeDist([i,j-1],[i,j]);
%             nodeP(iter) = (j-1)*row + i;
%             nodeQ(iter) = (j-2)*row + i;
%             iter = iter + 1;            
%         end
%         if diagDownLeftflag
%             B(iter) = computeBoundary(Iorigin,Idiagdownleft,sigma) / computeDist([i+1,j-1],[i,j]);
%             nodeP(iter) = (j-1)*row + i;
%             nodeQ(iter) = (j-2)*row + i+1;
%             iter = iter + 1;
%         end
%         if diagUpLeftflag
%             B(iter) = computeBoundary(Iorigin,Idiagupleft,sigma) / computeDist([i-1,j-1],[i,j]);
%             nodeP(iter) = (j-1)*row + i;
%             nodeQ(iter) = (j-2)*row + i-1;
%             iter = iter + 1;
%         end       
    end
end
m = row*col;
n = row*col;
pairWise = sparse(nodeP,nodeQ,B,m,n);

end

function B = computeBoundary(p,q,sigma)
% B{p,q} = exp(-(Ip-Iq)^2/(2sigma^2)) * 1/dist(p,q)
diff = double(p) - double(q);
% diffsqr = sum(diff.^2);
diffsqr = (diff.^2);
expoterm = - diffsqr./ (2*sigma.^2);
B = sum(exp(expoterm));

% B = exp( - sum( (double(p)-double(q)).^2 ) ./ (2*sigma.^2) );
% if B <= 0.1
% disp(diff)
% end
end

function dist = computeDist(p,q)  
dist = sqrt( sum( (p - q).^2 ) );
if dist == 1 || dist == sqrt(2)

else
    disp('')
end
end


