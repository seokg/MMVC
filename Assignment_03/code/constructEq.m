function [A,b] = constructEq(lbTx, lbTy, ubTx,ubTy,numberPts,delta)
%% Contruct A and b for linear programming
% input
%   lbTx, lbTy: lower bound of the model Theta (Tx Ty)
%   ubTy, ubTy: upper bound of the model Theta (Tx Ty)
%   numberPts: number of data points 
%   delta: inlier threshold value
% output
%   A,b: inequality Ax <= b

%% inequality 
% construct A
global inputPointsLeft
global inputPointsRight
Atheta = zeros(numberPts*4, 2);
temp = [inputPointsLeft - inputPointsRight - delta, ...
       -inputPointsLeft + inputPointsRight - delta];
temp = [temp(:,1) , temp(:,3), temp(:,2), temp(:,4)];
Cell = cell(1,numberPts);
for idx = 1: numberPts
    [Cell{idx}] = deal(temp(idx,:)');
end
Az = blkdiag(Cell{:});
clear temp Cell;
Cell = cell(1,2*numberPts);
[Cell{:}] = deal([1,-1]');
Aw = blkdiag(Cell{:});
clear Cell;


% bilinearity contrain
Abitheta = [0 0;1 0;0 0;-1 0;0 0;0 1;0 0;0 -1];
Abitheta = repmat(Abitheta,numberPts,1);

Abiz = [lbTx ubTx -ubTx -lbTx lbTy ubTy -ubTy -lbTy]';
Cell = cell(1,numberPts);
[Cell{:}] = deal(Abiz);
clear Abiz;
Abiz = blkdiag(Cell{:});
clear Cell;


temp = [-1 -1 1 1]';
zeroM = zeros(4,1);
Abiw = [temp zeroM; zeroM, temp];
clear temp zeroM;
Cell = cell(1,numberPts);
[Cell{:}] = deal(Abiw);
clear Abiw;
Abiw = blkdiag(Cell{:});
clear Cell;

% concat all the matrix
A = [Atheta Az Aw; Abitheta Abiz Abiw];

clear Atheta Az Aw;

% contruct b
bbi = repmat([0 ubTx 0 -lbTx 0 ubTy 0 -lbTy]',numberPts,1);
b = [zeros(numberPts*4,1); bbi];
clear bbi;
end