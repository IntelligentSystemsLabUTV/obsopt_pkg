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

    %%%% control law - battery %%%%
    u = [sin(t); zeros(size(t))];
        
    else
        u = zeros(params.dim_input,1);
    end
end