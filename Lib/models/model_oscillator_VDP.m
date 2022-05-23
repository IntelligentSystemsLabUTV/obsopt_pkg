%% mock up model
function x_dot = model_oscillator_VDP(t,x,params,obs)

    x_dot = zeros(length(x),1);
    
    params.u = params.input(t,x,params);
%     params.u = params.input(t,y,yhat,params);
    
    x_dot(1) = params.eps*(x(2) + params.u(1));
    x_dot(2) = params.eps*(-x(1) + params.mu*(1-x(1)^2)*x(2) + params.u(2));
    
    % params dynamics
%     x_dot(3) = 0.1 + 0.05*randn(1);    % random walk
end