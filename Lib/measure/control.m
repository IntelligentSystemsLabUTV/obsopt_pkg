%% measure function
function u = control(x,params)

    %% control law - double integrator
%     u = [params.K1, params.K2, params.K3]*x(1:3);
    
    %% control law - double pendulum
    % single torque
%     u = [params.K1; params.K2];

    % decoupled linear
%     u(1) = params.K1+ [params.K2, params.K3]*[x(1); x(3)];
%     u(2) = params.K4+ [params.K5, params.K6]*[x(2); x(4)];
    
    % coupled linear
%     u(1) = params.K1+ [params.K2, params.K3, params.K4, params.K5]*x(1:4);
%     u(2) = params.K6+ [params.K7, params.K8, params.K9, params.K10]*x(1:4);

    %% control law - bioreactor
%     u(1) = -params.u1*x(1);
%     u(2) = -params.u1*x(2);
%     u(3) = -params.u1*x(3) + params.u1*params.u2; 

    %% control law - Astolfi
%     u = 5;
    
    % input enable
    if ~params.input_enable
        u = zeros(params.dim_input,1);
    end
end