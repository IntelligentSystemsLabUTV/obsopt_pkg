%% mock up model
function x_dot = model_Tesi_01(t,x,params)

    x_dot = zeros(length(x),1);
    
    params.u = params.input(t,x,params);

    A = [0.1344, 0.2155, -0.1084; ...
         0.4585, 0.0797, 0.0857; ...
         -0.5647, -0.3269, 0.8946];
    B = [0.9298, 0.9143, -0.7162; ...
         -0.6848, -0.0292, -0.1565; ...
         0.9412, 0.6006, 0.8315];
    
    x_dot(1:params.dim_state) = A*x(1:params.dim_state) + B*params.u;
    x_dot(params.dim_state+1:end) = x(params.dim_state+1:end);
end