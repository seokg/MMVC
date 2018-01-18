function modelBound = BranchNBound(f,A,b,lb,ub,flag)
%% Branch and Bound Algorithm
% input 
% f: objective function
% A,b: inqaulity 
% lb: lower bound
% ub: upper bound

%%
% output value initialization
modelBound = [];
global inputPointsLeft;
global inputPointsRight;
pt1 = inputPointsLeft;
pt2 = inputPointsRight;

% update lower and upper bound for inliers
global inlierLowerBound;
global inlierUpperBound

% global candidateLowerBound;
% global candidateUpperBound;

% manage candidate data using nodes
global candidateData;
global numberNodes;

global iteration;
iteration = iteration + 1;

if(flag == 1) 
    [x,opt,exitflag]=linprog(f,A,b,[],[],lb,ub);
    % FEASIBILITY on RELAXED SOLUTION
    % infeasible case detected: no such solution       
    if(exitflag == 1)
        numberNodes = numberNodes + 1;
        candidateData(:,numberNodes) = [0;countNumInlier2(x(1:2),pt1, pt2, 3);opt;...
            lb(1); ub(1); lb(2); ub(2)];
        inlierLowerBound = [inlierLowerBound, countNumInlier2(x(1:2),pt1, pt2, 3)];
%         candidateData(:,numberNodes) = [0;countNumInlier(lb(1:2),ub(1:2),pt1, pt2, 3);opt;...
%             lb(1); ub(1); lb(2); ub(2)];
%         inlierLowerBound = [inlierLowerBound, countNumInlier(lb(1:2),ub(1:2),pt1, pt2, 3)];
        inlierUpperBound = [inlierUpperBound, opt];
        % FEASIBILITY on ORIGINAL SOLUTION
        % segment
        [~, z, ~] = segmentX(x);
        % Check feasibility on P
        flag = isFeasible(z);
        if(flag)
            disp('EXIT FLAG: Optimal value for BnB is found in first iteration\nExiting program')
            return
        end
    else
        fprintf('exitflag: %i\n',exitflag)
        disp('ERROR: the constrain for linear programming might be wrong. ')
    end
end

% fprintf('number of inliears: %i\n',-inlierLowerBound(end));
%% Divide the bound into two 
lb1 = lb;
ub1 = ub;
lb2 = lb;
ub2 = ub;
if ((abs(lb(2)-ub(2)) >1) && (abs(lb(1)-ub(1)) > 1 )) 
    if((abs(lb(1)-ub(1)) >= abs(lb(2)-ub(2))))
        ub1(1) = floor((lb(1)+ub(1))/2);
        lb2(1) = ceil((lb(1)+ub(1))/2);        
    elseif(abs(lb(1)-ub(1)) < abs(lb(2)-ub(2)))
        ub1(2) = floor((lb(2)+ub(2))/2);
        lb2(2) = ceil((lb(2)+ub(2))/2);
    end
elseif ((abs(lb(2)-ub(2)) <1) && (abs(lb(1)-ub(1)) > 1 ))
        ub1(1) = floor((lb(1)+ub(1))/2);
        lb2(1) = ceil((lb(1)+ub(1))/2);            
elseif ((abs(lb(2)-ub(2)) >1) && (abs(lb(1)-ub(1)) < 1 )) 
        ub1(2) = floor((lb(2)+ub(2))/2);
        lb2(2) = ceil((lb(2)+ub(2))/2);
else
    disp('EXIT FLAG: bound cannot be branched further')
    return        
end

%% Contruct new inequality
[A1,b1] = constructEq(lb1(1),lb1(2),ub1(1),ub1(2),length(inputPointsLeft),3);
[A2,b2] = constructEq(lb2(1),lb2(2),ub2(1),ub2(2),length(inputPointsLeft),3);    

%%  Check lower and upper bound of inliers 
%    using linprog and inlier calc equation
% [~,opt1,exitflag1]=linprog(f,A1,b1,[],[],lb1,ub1);
% [~,opt2,exitflag2]=linprog(f,A2,b2,[],[],lb2,ub2);
% lowerBound1 = countNumInlier(lb1(1:2),ub1(1:2),pt1, pt2, 3);
% lowerBound2 = countNumInlier(lb2(1:2),ub2(1:2),pt1, pt2, 3);
[x1,opt1,exitflag1]=linprog(f,A1,b1,[],[],lb1,ub1);
[x2,opt2,exitflag2]=linprog(f,A2,b2,[],[],lb2,ub2);
lowerBound1 = countNumInlier2(x1(1:2),pt1, pt2, 3);
lowerBound2 = countNumInlier2(x2(1:2),pt1, pt2, 3);
numberNodes = numberNodes + 1;
node1 = numberNodes;
candidateData(:,node1) = [0;lowerBound1;opt1;lb1(1); ub1(1); lb1(2); ub1(2)];
numberNodes = numberNodes + 1;
node2 = numberNodes;
candidateData(:,node2) = [0;lowerBound2;opt2;lb2(1); ub2(1); lb2(2); ub2(2)];
% initialize furthur inspection flag

%% omit this section--------------------------
if (lowerBound1 == -15 || lowerBound2 == -15)
    disp('');
end
%--------------------------------------------

%% VERSION 2
% add the candiate lower and upper bound
if inlierLowerBound(end) < -opt1 && exitflag1 == 1
    candidateData(1,node1) = 1;
end
if inlierLowerBound(end) < -opt2 && exitflag2 == 1
    candidateData(1,node2) = 1;
end

% find bad bound for the current OPT inlier lower bound
% inlier lower bound > candidate inlier upper bound
badBoundIdx = find(inlierLowerBound(end) > -candidateData(3,:));
candidateData(1,badBoundIdx) = 0;

% find the next best candidate point
idx = find(candidateData(1,:) == 1);
realCandidateData = candidateData(:,idx);
[currentLowerBound,idx1] = max(-realCandidateData(2,:));
[currentUpperBound,idx2] = max(-realCandidateData(3,:));
% pop out the best result from the list
% candidateData(1,idx(idx1)) = 0;
candidateData(1,idx(idx2)) = 0;
% save it to the final list
inlierLowerBound = [inlierLowerBound -currentLowerBound];
inlierUpperBound = [inlierUpperBound -currentUpperBound] ;

% if current upper and lower bound's difference is less than 1 stop
if(currentUpperBound - currentLowerBound < 1)
   disp('Branch and Bound has reached its optimal bound')
   modelBound = candidateData(4:end,idx(idx2));
   return
end

%% Recursion: further inspection using Branch and Bound
newlbTx = candidateData(4,idx(idx2));
newubTx = candidateData(5,idx(idx2));
newlbTy = candidateData(6,idx(idx2));
newubTy = candidateData(7,idx(idx2));
newlb = lb;
newub = ub;
newlb(1) = newlbTx;
newlb(2) = newlbTy;
newub(1) = newubTx;
newub(2) = newubTy;
[newA,newb] = constructEq(newlbTx,newlbTy,newubTx,newubTy,length(inputPointsLeft),3);
modelBound = BranchNBound(f,newA,newb,newlb,newub,0);

end


function [theta z w] = segmentX(x)
%% segment vectors
    theta = x(1:2);
    z = x(3:57); 
    w = x(58:end);
end

function lowerBound = countNumInlier(lb,ub,pt1, pt2, threshold)
%% countNumInliers: count the number of inliers for branch and bound
% input
%   lb: lower bound
%   ub: upper bound
%   pt1: left points
%   pt2: right points
%    threshold: inlier threshold value
% output
%   lowerBound: number of inliers
%%
avgLowerUpper= repmat(((lb + ub)/2)',length(pt1),1);
tempBoolInliers = (pt2 - (pt1+avgLowerUpper) <= threshold) .* (pt2 - (pt1+avgLowerUpper) >= -threshold); 
boolInliers =tempBoolInliers(:,1) .*tempBoolInliers(:,2);
lowerBound = -nnz(boolInliers);
end

function lowerBound = countNumInlier2(theta,pt1, pt2, threshold)
%% countNumInliers: count the number of inliers for branch and bound
% input
%   lb: lower bound
%   ub: upper bound
%   pt1: left points
%   pt2: right points
%    threshold: inlier threshold value
% output
%   lowerBound: number of inliers
%%
tempTheta= repmat(theta',length(pt1),1);
tempBoolInliers = (pt2 - (pt1+tempTheta) <= threshold) .* (pt2 - (pt1+tempTheta) >= -threshold); 
boolInliers =tempBoolInliers(:,1) .*tempBoolInliers(:,2);
lowerBound = -nnz(boolInliers);
end


function boolean = isFeasible(z)
%% find non interger value for z (binary: 0,1)
    boolean = true;
    idx = find(z < 10^-8);
    z(idx) = 0;
    idx = find(z > (1-10^-8));
    z(idx) = 1;
%     z = single(z);
    numNonFeasible = nnz(mod(z,1)~=0 );
    
    if( numNonFeasible > 0 )
        boolean = false;
    else
        disp('is feasible');
    end

end