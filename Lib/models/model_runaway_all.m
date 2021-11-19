%% Runaway electrons model
% x = [T W]'
function x_dot = model_runaway_all(t,x,params)

    % state init
    x_dot = zeros(length(x),1);
    
    % get gamma
    params.Q = x(3); 
    params.S = x(4); 
    params.gamma = x(5); 
    params.gamma1 = x(6); 
    params.ni = x(7); 
    params.Wt = x(8);
%     params.eps_coef = x(9);
    
    % just for reading ease
    T = x(1);
    W = x(2);

    x_dot(1) = params.eps_coef*(-2*T*W - 2*params.S + params.Q);
    x_dot(2) = params.eps_coef*(-params.ni*W + params.gamma*(T*W + params.S) - params.gamma1*W/(1+W/params.Wt));
    
    % parameters dynamics
    x_dot(3:end) = 0;
end