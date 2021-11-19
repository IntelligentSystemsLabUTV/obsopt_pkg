%% model function 
function x_dot = model_pendulum_mass(t,x,params)

%     x_dot = zeros(length(x),1);
    
    params.M = x(3);
    
    x_dot(1) = x(2);
    x_dot(2) = (params.Lt*params.M*params.g*sin(x(1)) - params.u - params.Fc*x(2))/params.If;
    
    % mass dynamics
    x_dot(3) = 0;
end