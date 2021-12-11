%% mock up model
function x_dot = model_mockup(t,x,params)

    x_dot = zeros(length(x),1);
    
    params.theta = x(2);
    
%     x_dot(1) = -abs(params.theta)*x(1);
    x_dot(1) = -params.theta*x(1);
    x_dot(2:end) = 0;
end