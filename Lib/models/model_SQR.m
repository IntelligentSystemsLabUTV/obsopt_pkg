%% eq. 3.3 elegant chaos
function x_dot = model_SQR(t,x,params,obs)
    
    x_dot = zeros(length(x),1);
    
    x_dot(1) = params.a-x(2);
    x_dot(2) = params.b +x(3);
    x_dot(3) = x(1)*x(2)-x(3);
    
    % speed up
    x_dot = params.eps_coef*(x_dot);
    
    % params dynamics
%     x_dot(4) = 0 + 0.1*randn(1);    % random walk
%     x_dot(4) = -0.01;    % exp decay
%     x_dot(4) = 0.1;    % linear growth
end