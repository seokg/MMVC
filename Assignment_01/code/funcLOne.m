function [output,error] = funcLOne(xdata, ydata)
    % Constrain
    % |ax + b - y| =  |eps| = a
    % a > eps,  a>-eps
    % eps = ax = b - y
    %
    
    temp_a = ([diag(xdata); diag(-xdata)]);
    pos = diag(ones(length(xdata),1));
    neg = diag(-ones(length(xdata),1));
    one = ones(length(xdata),1);
    aIneq = [zeros(2*length(xdata),1), zeros(2*length(xdata),1),[pos;neg], [neg;neg]];
    bIneq = zeros(2*length(xdata),1);
    
    aEq = [xdata one pos zeros(length(xdata),length(xdata))];
    bEq = ydata;
    
    %objective function
    f = [0 0 zeros(1,length(xdata)) ones(1,length(xdata))];

    [output,error] = linprog(f,aIneq,bIneq,aEq,bEq);
end