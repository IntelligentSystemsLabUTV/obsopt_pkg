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
function params_out = params_update_battery_fedeoli(params,x,t)

    % assign params
    params_out = params;    
      
    params_out.Rx = x(2);
    params_out.Voc = x(3);
    params_out.C = x(4);
    params_out.Rb = x(5);
        
end