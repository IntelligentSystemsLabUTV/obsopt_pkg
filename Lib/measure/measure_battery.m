%% MEASURE_BATTERY
% file: measure_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function implements the output mapping of a battery
% model
% INPUT:
% x: state vector
% params: structure with all the necessary parameters 
% t: time instant (may be not used)
% OUTPUT:
% y: output measurement
function y = measure_battery(x,params,t)

    % compute the control - I
    params.u = params.input(t,x,params);
    
    % preset the params
    % params = params_preset_battery(params,x);
    
    % get the observed components of the state vector
    y = params.Voc-x(2,:)-params.u(1,:).*params.R0;
end