%% CONTROL
% file: control.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements the control law
% INPUT:
% t: time instant
% drive: error variable
% params: structure with all the necessary parameters 
% OUTPUT:
% u: control variable
function u = control(t,drive,params)

    % check if the input is enabled
    if params.input_enable
        
    % !REMARK!
    % simply uncomment or add the law you need, according to params

    %%%% control law - double integrator %%%%
%     u = [params.K1, params.K2, params.K3]*x(1:3);
    
    %%%% control law - double pendulum %%%%
    %%%%% single torque %%%%
%     u = params.K*[drive(1:2);zeros(2,1)];
%     u = params.K*drive;

    %%%% decoupled linear %%%%
%     u(1) = params.K1+ [params.K2, params.K3]*[x(1); x(3)];
%     u(2) = params.K4+ [params.K5, params.K6]*[x(2); x(4)];
    
    %%%%% coupled linear %%%%
%     u(1) = params.K1+ [params.K2, params.K3, params.K4, params.K5]*x(1:4);
%     u(2) = params.K6+ [params.K7, params.K8, params.K9, params.K10]*x(1:4);

    %%%% control law - sin %%%%
%     u(1) = sin(t);
%     u(2) = -sin(t);

    %%%% cntrol law - VDP %%%%
%     u = params.KX*x(1:params.dim_state) + params.KR*params.r_story(:,params.ActualTimeIndex);
%     u = params.r_story(:,params.ActualTimeIndex);

    %%%% control law - battery %%%%
%     u = sin(t);
        
    else
        u = zeros(params.dim_input,1);
    end
end