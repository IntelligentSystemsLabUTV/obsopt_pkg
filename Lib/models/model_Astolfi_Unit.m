%% double pendlum model
function x_dot = model_Astolfi_Unit(t, x, params)

    x_dot = zeros(length(x),1);
    
    x_dot = params.A*x + params.B*(x(1)-x(1)^3);
    
end