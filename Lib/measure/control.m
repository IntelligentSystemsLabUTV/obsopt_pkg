%% measure function
function u = control(x,params)

    % control law - double integrator
%     u = [params.K1, params.K2, params.K3]*x(1:3);
    
    % control law - double pendulum
%     u = [params.K1; params.K2];
    u(1) = params.K1+ [params.K2, params.K3]*[x(1); x(3)];
    u(2) = params.K4+ [params.K5, params.K6]*[x(2); x(4)];
    
    % input enable
    u = params.input_enable*u;
end