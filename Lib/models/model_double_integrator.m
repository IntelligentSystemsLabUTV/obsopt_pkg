%% mock up model
function x_dot = model_double_integrator(t,x,params)

    x_dot = zeros(length(x),1);
    
    params.K1 = x(3);
    params.K2 = x(4);
    
    params.u = params.input(x,params);
    
    x_dot(1) = x(2);
    x_dot(2) = params.u;
    
    x_dot(3:end) = 0;
end