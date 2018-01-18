%% Graph Cut Sand Box
%
% Use the following functions
%   BK_create
%   BK_SetUnary
%   BK_SetPairwise
%   BK_MInimize
%   BK_GetLabeling

clear all 
clc 
close all
%%
% hypothetical 2x2 image with 4-neighborhood: 1 | 3
%                                             2 | 4
% pairs = [1 1 2 3; 2 3 4 4];

% unary costs: q1(1) = q1(2) = q1(3) = 1, q1(4) = 4
%              q2(1) = q2(2) = q2(3) = 2, q2(4) = 1
% cost_unary = [1 1 1 4; 2 2 2 1];

% pairwise costs: r(1,2) = r(1,3) = r(2,4) = r(3,4) = 1
pairs = [1 2;2 3];
% cost_pair1 = [3 5];
% cost_pair2 = [2 1];
cost_pair1 = [5 3];
cost_pair2 = [1 2];
% cost_unary = [9 7 5;4 7 8];
cost_unary = [5 7 9;8 7 4];


num_vars = size(cost_unary, 2);
num_pairs = size(cost_pair1, 2);
graph = BK_Create(num_vars, num_pairs);
BK_SetUnary(graph, cost_unary);
pot_zero = zeros(num_pairs, 1);
edges = [pairs' pot_zero cost_pair1' cost_pair2' pot_zero];
BK_SetPairwise(graph, edges);

% find optimum labeling
BK_Minimize(graph);
lab = BK_GetLabeling(graph)';
lab = double(lab);
disp('TASK 01')
disp('Handling Max Flow')
fprintf('labeling is:\n\t%i | %i | %i\n', ...
	lab(1), lab(2), lab(3) );