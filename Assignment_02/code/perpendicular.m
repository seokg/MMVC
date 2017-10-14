function output = perpendicular(pt)
%% compute the perpendicular points for input pt
% input
%   pt: input point (size:[2 1])
%output
%   output: perpendicular point
output = zeros(size(pt));
output(2) = pt(1);
output(1) = -pt(2);
end