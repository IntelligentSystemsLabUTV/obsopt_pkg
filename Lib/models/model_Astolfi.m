%% model for pendulum
function x_dot = model_Astolfi(t,x,params)

    x_dot = zeros(length(x),1);
    u = params.u;
    
    x_dot(1) = x(2);
    x_dot(2) = 1/params.T*(params.Pm - params.D*x(2) - params.E*params.V*x(3)*sin(x(1)));
    x_dot(3) = 1/params.Tdc*(-x(3)+params.Lambda_star) + u;
end