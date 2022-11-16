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

    % init input
    u = zeros(1,length(t));
    % params.dim_input
            

    % check if the input is enabled
    if params.input_enable                
           
    end
end