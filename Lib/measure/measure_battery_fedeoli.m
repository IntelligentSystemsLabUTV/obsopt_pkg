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
function y = measure_battery_fedeoli(x,params,t)

    % compute the control - I
    params.u = params.input(t,x,params);    
    
    % get the observed components of the state vector
    y = x(1,:)-params.u(1,:).*x(5,:);
end