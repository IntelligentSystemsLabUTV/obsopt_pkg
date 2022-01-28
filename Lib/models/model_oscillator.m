%% mock up model
function x_dot = model_oscillator(t,x,params)

    x_dot = zeros(length(x),1);
    
    x_dot(1) = params.speed*x(2);
    x_dot(2) = -params.speed*x(1) ;
end