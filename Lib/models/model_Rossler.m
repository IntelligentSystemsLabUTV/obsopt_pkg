%% eq. 3.3 elegant chaos
function x_dot = model_Rossler(t,x,params)

    params.a = x(4);
    params.b = x(5);
    params.c = x(6);
    
    X = x(1);
    Y = x(2);
    Z = x(3);
    
    x_dot = zeros(length(x),1);
    
    x_dot(1) = -Y -Z;
    x_dot(2) = X + params.a*Y;
    x_dot(3) = params.b + Z*(X-params.c);
    
    % params dynamics
    x_dot(4) = 0;
    x_dot(5) = 0;
    x_dot(6) = 0;
    
    % speed up
    x_dot = params.eps_coef*(x_dot);
end