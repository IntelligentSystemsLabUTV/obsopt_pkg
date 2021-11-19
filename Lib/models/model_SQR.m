%% eq. 3.3 elegant chaos
function x_dot = model_SQR(t,x,params)

    params.a = x(4);
    params.b = x(5);
    
    X = x(1);
    Y = x(2);
    Z = x(3);
    
    x_dot = zeros(length(x),1);
    
    x_dot(1) = params.a-Y;
    x_dot(2) = params.b +Z;
    x_dot(3) = X*Y-Z;
    
    % params dynamics
    x_dot(4) = 0;
    x_dot(5) = 0;
    
    % speed up
    x_dot = params.eps_coef*(x_dot);
end