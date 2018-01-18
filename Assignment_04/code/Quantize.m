function quantizedColor = Quantize(inputColor, histRes)
% quantize the color value such that it will have certain resolution

% len = length(inputColor);
stepSize = 256/histRes;

if mod(stepSize,1) ~= 0 
    disp('Warning reset histResolution that it will have a uniform step')
end
quantizedColor = zeros(size(inputColor));
for i = 1:histRes
    idx = (inputColor <= stepSize*i -1) & (inputColor > stepSize*(i-1));
%     inputColor(idx) = stepSize * (i-1) + stepSize/2;
    quantizedColor(idx) = i;
end

end

