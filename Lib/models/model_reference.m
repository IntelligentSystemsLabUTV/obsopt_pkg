%% mock up model
function x_dot = model_reference(t,x,params)

    x_dot = zeros(length(x),1);
   
    % model reference - double integrator
%     x_dot(1) = -0.5*x(1);
    
    % model reference - double pendulum
%     x_dot(1) = 0;
%     x_dot(2) = 0;

    % model reference - Tesi 01
    A = 0.2*eye(params.dim_state);
    B = 0.8*eye(params.dim_state);
    x_dot(1:params.dim_state) = A*x(1:params.dim_state) + B*params.r_story(:,params.ActualTimeIndex);
end