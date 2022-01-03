%% measure function
function u = control(x,params)

    % control law - double integrator
%     u = [params.K1, params.K2, params.K3]*x(1:3);
    
    % control law - double pendulum
    % single torque
%     u = [params.K1; params.K2];

    % decoupled linear
    u(1) = params.K1+ [params.K2, params.K3]*[x(1); x(3)];
    u(2) = params.K4+ [params.K5, params.K6]*[x(2); x(4)];
    
    % coupled linear
%     u(1) = params.K1+ [params.K2, params.K3, params.K4, params.K5]*x(1:4);
%     u(2) = params.K6+ [params.K7, params.K8, params.K9, params.K10]*x(1:4);
    
    % input enable
    u = params.input_enable*u;
end