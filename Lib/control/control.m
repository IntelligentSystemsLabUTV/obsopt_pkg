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

    % init input
    u = zeros(1,length(t));
    % params.dim_input

    % check if the input is enabled
    if params.input_enable
        
        % !REMARK!
        % simply uncomment or add the law you need, according to params

        %%%% control law - battery %%%%
        % sine wave
        u(1,:) = -1 * interp1(params.input_time,params.input_current,t,'previous');
%         u(1,:) = sin(0.5*t);

%         period = 300;
%         if mod(t,period) > period/3
%            u(1,:) = 2; 
%         else
%            u(1,:) = 0;
%         end
           
    end
end