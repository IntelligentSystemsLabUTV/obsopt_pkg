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
function u = control_ID(t,drive,params)

    % check if the input is enabled
    if params.input_enable
        
    % !REMARK!
    % simply uncomment or add the law you need, according to params

    %%%% control law - battery %%%%
%     u = [sin(t); zeros(size(t))];
    
    %%%% control law - control test %%%%
    % this first is u_ecc
%     u(1,:) = 1*(sin(1*t)+sin(200*t));
    p = 20;
    u(1,:) = 1*(mod(t,p)<p/2);
    % this second is the stabilising input
%     u(2,:) = [params.K1, params.K2]*drive(3:4,:);
%     u(2,:) = params.K3*drive(3,:);
%     u(3,:) = params.K4*drive(4,:);
%     u(4,:) = params.K5*drive(5,:);


    %%%% control law - pendulum %%%%
%     u = [params.K1, params.K2, params.K3, params.K4;...
%          params.K5, params.K6, params.K7, params.K8]*drive(1:4);

%     u = [params.K1, params.K2;...
%          params.K3, params.K4]*drive(1:2);
        
    else
        u = zeros(params.dim_input,1);
    end
end