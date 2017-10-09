function [output,error] = funcLInfinity(xdata, ydata, f)
    % Constrain
    % |ax + b - y| < eps
    % -eps< ax + b - y < eps
    %
    % ax + b - y < eps
    % ax + b - y >- eps
    %
    %  ax + b - y < eps
    % -ax - b + y < eps
    %
    %  ax + b - eps < y
    % -ax - b - eps < -y
    temp_a = [xdata; -xdata];
    pos = ones(length(xdata),1);
    neg = -ones(length(xdata),1);
    a = [temp_a, [pos;neg], [neg;neg]];
    b = [ydata;-ydata];

    % objective function
    % a, b,  eps
    f = [0 0 1];

    [output,error] = linprog(f,a,b);
end