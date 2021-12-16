%% mock up model
function x_dot = model_reference(t,x,params)

    x_dot = zeros(length(x),1);
   
    
    x_dot(1) = -x(2);
    x_dot(2) = x(1);
    
    x_dot(3:end) = 0;
end