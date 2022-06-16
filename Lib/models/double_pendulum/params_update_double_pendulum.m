%% PARAMS_UPDATE_DOUBLE_PENDULUM
% file: params_update_double_pendulum.m
% author: Federico Oliva
% date: 15/06/2022
% description: this function updates the estimated parameters on a double 
% pendulum
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_double_pendulum(params,x)

    % assign params
    params_out = params;
    
    % update parameters
%     params.c1 = x(5);
%     params.c2 = x(6);
end