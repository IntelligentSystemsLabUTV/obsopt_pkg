%% PARAMS_UPDATE_BATTERY
% file: params_update_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function updates the estimated parameters on a Van der 
% Pol oscillator
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_battery(params,x)

    % assign params
    params_out = params;
    
    % update the nonlinearity
    % params_out.mu = x(3);
    
    params_out.Voc = x(3);
    params_out.R0 = x(4);
    params_out.R1 = x(5);
    params_out.C1 = x(6);
end