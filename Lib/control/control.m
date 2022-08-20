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
function u = control(t,drive,params,obs)

    % check if the input is enabled
    if params.input_enable
        
    % !REMARK!
    % simply uncomment or add the law you need, according to params

    %%%% control law - battery %%%%
%     u = [sin(t); zeros(size(t))];
    
    %%%% control law - control test %%%%    
    % (r-y) 
    e = drive(end-obs.setup.Nfilt,:);
    % output from controller - input to plant
    % C*x_c + D*(r-y)
    u(1,:) = [params.c0 params.c1]*drive(3:4) + params.d0*e;  
    % error signal (r-y) - input to controller
    u(2,:) = e;
    % input for refefence model
    u(3,:) = params.a*(mod(t,params.p)<params.p*params.dc)+params.offset;
    % input for system identification
    u(4,:) = params.rescale*u(3,:);


    %%%% control law - pendulum %%%%
%     u = [params.K1, params.K2, params.K3, params.K4;...
%          params.K5, params.K6, params.K7, params.K8]*drive(1:4);

%     u = [params.K1, params.K2;...
%          params.K3, params.K4]*drive(1:2);
        
    else
        u = zeros(params.dim_input,1);
    end
end