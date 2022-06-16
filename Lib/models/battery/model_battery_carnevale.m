%% MODEL_BATTERY
% file: model_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function describes the dynamics equation of a Van der 
% Pol oscillator
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_battery_carnevale(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % compute the control - Ib
    params.u = params.input(t,x,params);
    
    % model dynamics (carnevale 2019)
    % Vp
    x_dot(1) = -x(1)-x(2)+x(3)-params.u(1)*x(4);
    % all the following are constant    
    
end