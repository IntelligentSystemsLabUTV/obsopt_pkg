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
function y = measure_battery_tushar_SIM(x,u,t)    
    
    % get the observed components of the state vector
    y(1,:) = x(3,:) - x(2,:) -(x(4,:).*u);

end