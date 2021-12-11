%% mock up model
function x_dot = model_double_integrator(t,x,params)

    x_dot = zeros(length(x),1);
    params.K = x(3);
    
    params.u = params.input(x,params);
    
    x_dot(1) = -x(2);
    x_dot(2) = params.u;
end