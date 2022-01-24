%% mock up model
function x_dot = model_oscillator(t,x,params)

    x_dot = zeros(length(x),1);
    
    x_dot(1) = x(2);
    x_dot(2) = -x(1) ;
end