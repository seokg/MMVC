function output = funcIRLS(Y, X, iter, winit, d, tolerance)

    
    % editing data in vector format representation
    [r, c] = size(X);
    X= [X ones(r,c)];
%     disp(r);
    
    % init weight as 1
    delta = d*ones(1,r);
    W = eye(r);
    
    % calc value for theta
    % and init
    theta = inv(X.' * (W*X)) * (X.' * (W*Y));
    disp(theta);
    
    
    for idx = 1: iter
       thetaprev = theta;
       wprev = abs(Y-X*theta).';
       w = 1 ./ max(delta, wprev);
       W = diag(w);
       theta = inv(X.' * (W*X)) * (X.' * (W*Y));
       tol = sum( abs( theta - thetaprev ) );
       fprintf('Tolerance = %i\n',tol)
       if tol < tolerance
           output = theta;
           return;
       end        
    end
    output = theta;
end