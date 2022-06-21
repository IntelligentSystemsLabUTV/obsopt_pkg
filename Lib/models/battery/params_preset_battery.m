%% PARAMS_PRESET_BATTERY
% file: params_preset_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function updates the battery parameters before the
% model computation
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_preset_battery(params,x)

    % assign params
    params_out = params;
    
    % compute the parameters
    params_out.Voc = params_out.alpha*x(1,:).^([1:length(params_out.alpha)]');
    params_out.R0 = params_out.beta_1*x(1,:).^([1:length(params_out.beta_1)]');
    params_out.R1 = params_out.beta_2*x(1,:).^([1:length(params_out.beta_2)]');
    params_out.C1 = params_out.beta_3*x(1,:).^([1:length(params_out.beta_3)]');
end