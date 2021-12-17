%% measure function
function u = control(x,params)

    % control law - double integrator
%     u = [params.K1, params.K2, params.K3]*x(1:3);
    
    % control law - double pendulum
    u = [params.K1; params.K2];
    
    % input enable
    u = params.input_enable*u;
end